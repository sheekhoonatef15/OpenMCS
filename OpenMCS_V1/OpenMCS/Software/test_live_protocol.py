import struct
import unittest

import numpy as np

from OpenMCS_GUI import FrameParser


def make_frame(magic, header, words):
    payload = b"".join(struct.pack("<I", int(value)) for value in words)
    total = struct.pack("<Q", sum(int(value) for value in words))
    return magic + header + payload + total + b"DONE"


class LiveProtocolTests(unittest.TestCase):
    def test_fragmented_mixed_stream(self):
        block = make_frame(
            b"MCB1",
            struct.pack("<HHHIIIB", 16_000, 300, 3,
                        100_000, 100_000_000, 1, 0x08),
            [10, 20, 30],
        )
        complete = make_frame(
            b"MCS6",
            struct.pack("<HIIIB", 4, 100_000, 100_000_000, 1, 0),
            [1, 2, 3, 4],
        )
        parser = FrameParser()
        frames = []
        stream = b"discard-me" + block + complete
        for offset in range(0, len(stream), 7):
            frames.extend(parser.feed(stream[offset:offset + 7]))
        self.assertEqual(len(frames), 2)
        self.assertTrue(frames[0][1]["live_block"])
        self.assertEqual(frames[0][1]["block_start"], 300)
        np.testing.assert_array_equal(frames[0][0], [10, 20, 30])
        self.assertFalse(frames[1][1]["live_block"])
        self.assertTrue(all(meta["total_ok"] for _, meta in frames))

    def test_16000_bin_block_coverage(self):
        ranges = []
        start = 0
        while start < 16_000:
            count = min(300, 16_000 - start)
            ranges.extend(range(start, start + count))
            start += count
        self.assertEqual(ranges, list(range(16_000)))
        self.assertEqual(len(ranges[-100:]), 100)


if __name__ == "__main__":
    unittest.main()
