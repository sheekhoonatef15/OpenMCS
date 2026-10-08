"""Hardware overlap check: compares a 1-pass histogram against a 2-pass
histogram (same geometry) to verify each accepted pass reinforces the same
trigger-relative bins, for both a short and a long acquisition window.

Does not modify the Teensy firmware or the GUI; only talks to the existing
serial protocol they already implement.
"""
import struct
import sys
import time

import numpy as np
import serial

MCS_MAGIC = b"MCS6"
LIVE_BLOCK_MAGIC = b"MCB1"
MCS_TRAILER = b"DONE"
MAX_BINS = 16_384
MIN_DWELL_CYCLES = 3
EXPECTED_CLOCK_HZ = 100_000_000
PORT = "COM6"
BAUD = 115200


class FrameParser:
    def __init__(self):
        self.buffer = bytearray()

    def feed(self, data):
        self.buffer.extend(data)
        frames = []
        while True:
            candidates = [i for i in (self.buffer.find(MCS_MAGIC),
                                       self.buffer.find(LIVE_BLOCK_MAGIC)) if i >= 0]
            magic_index = min(candidates) if candidates else -1
            if magic_index < 0:
                preserve = len(MCS_MAGIC) - 1
                if len(self.buffer) > preserve:
                    del self.buffer[:len(self.buffer) - preserve]
                break
            if magic_index:
                del self.buffer[:magic_index]
            live_block = bytes(self.buffer[:4]) == LIVE_BLOCK_MAGIC
            header_length = 23 if live_block else 19
            if len(self.buffer) < header_length:
                break
            if live_block:
                (bin_count, block_start, block_count, dwell_cycles,
                 clock_hz, pass_count, flags) = struct.unpack_from("<HHHIIIB", self.buffer, 4)
                geometry_ok = (1 <= bin_count <= MAX_BINS and 1 <= block_count <= bin_count
                               and block_start < bin_count and block_start + block_count <= bin_count)
                transmitted_words = block_count
            else:
                bin_count, dwell_cycles, clock_hz, pass_count, flags = struct.unpack_from(
                    "<HIIIB", self.buffer, 4)
                block_start = 0
                block_count = bin_count
                transmitted_words = bin_count
                geometry_ok = 1 <= bin_count <= MAX_BINS
            if not (geometry_ok and dwell_cycles >= MIN_DWELL_CYCLES and clock_hz == EXPECTED_CLOCK_HZ):
                del self.buffer[0]
                continue
            counts_offset = header_length
            total_offset = counts_offset + transmitted_words * 4
            trailer_offset = total_offset + 8
            frame_length = trailer_offset + 4
            if len(self.buffer) < frame_length:
                break
            if bytes(self.buffer[trailer_offset:frame_length]) != MCS_TRAILER:
                del self.buffer[0]
                continue
            counts = np.frombuffer(bytes(self.buffer[counts_offset:total_offset]), dtype="<u4").copy()
            del self.buffer[:frame_length]
            frames.append((counts, {
                "bin_count": bin_count, "dwell_cycles": dwell_cycles,
                "clock_hz": clock_hz, "pass_count": pass_count,
                "live_block": live_block, "block_start": block_start,
                "block_count": block_count,
            }))
        return frames


def capture_pass(ser, parser, bins, dwell, target_pass_count, timeout_s):
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        data = ser.read(4096)
        if data:
            for counts, meta in parser.feed(data):
                if (not meta["live_block"] and meta["bin_count"] == bins
                        and meta["dwell_cycles"] == dwell
                        and meta["pass_count"] == target_pass_count):
                    return counts
    raise TimeoutError(
        f"Timed out waiting for bins={bins} dwell={dwell} pass_count={target_pass_count}"
    )


def run_case(ser, parser, bins, dwell, timeout_s):
    ser.write(b"p\n")
    time.sleep(0.05)
    ser.write(b"z\n")
    time.sleep(0.05)
    ser.reset_input_buffer()
    parser.buffer.clear()

    ser.write(b"m 1\n")
    ser.write(f"y {dwell} {bins}\n".encode())
    single = capture_pass(ser, parser, bins, dwell, 1, timeout_s)

    ser.write(b"p\n")
    time.sleep(0.05)
    ser.write(b"z\n")
    time.sleep(0.05)
    ser.reset_input_buffer()
    parser.buffer.clear()

    ser.write(b"m 2\n")
    ser.write(f"y {dwell} {bins}\n".encode())
    double = capture_pass(ser, parser, bins, dwell, 2, timeout_s)

    ser.write(b"p\n")
    return single.astype(np.int64), double.astype(np.int64)


def report(name, single, double):
    predicted = 2 * single
    diff = double - predicted
    max_abs_diff = int(np.max(np.abs(diff)))
    total_single = int(single.sum())
    total_double = int(double.sum())
    print(f"[{name}] single-pass total={total_single} double-pass total={total_double} "
          f"max|double-2*single|={max_abs_diff}")
    aligned = max_abs_diff <= max(2, int(0.05 * max(1, total_single)))
    print(f"[{name}] {'OVERLAP OK' if aligned else 'OVERLAP FAILED'}")
    return aligned


def main():
    dwell = 10  # 100 ns/bin at 100 MHz
    ser = serial.Serial(PORT, BAUD, timeout=0.2)
    parser = FrameParser()
    try:
        small_single, small_double = run_case(ser, parser, bins=62, dwell=dwell, timeout_s=5.0)
        small_ok = report("bins=62", small_single, small_double)

        large_single, large_double = run_case(ser, parser, bins=16000, dwell=dwell, timeout_s=15.0)
        large_ok = report("bins=16000", large_single, large_double)
    finally:
        ser.close()

    if small_ok and large_ok:
        print("RESULT: overlap holds regardless of bin count")
        return 0
    print("RESULT: overlap mismatch detected")
    return 1


if __name__ == "__main__":
    sys.exit(main())
