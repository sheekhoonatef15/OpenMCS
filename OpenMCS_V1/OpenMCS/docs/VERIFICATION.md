# Verification and example data

## Package integrity

From the repository root:

```sh
python Verification/verify_package.py
```

This checks SHA-256 hashes, Python syntax, and project source references. It uses the Python standard library and does not connect to hardware. Hashes detect changed distribution files; they do not prove FPGA timing closure or correspondence between source and prebuilt images.

## Included measurement and analysis

`Verification/` contains:

| File | Purpose |
| --- | --- |
| `500 ns_OpenMCS_100MHz_300MHz_test.csv` | Recorded histogram with time in ns in column 1 and counts in column 2 |
| `500 ns_OpenMCS_100MHz_300MHz_test.png` | Accompanying measurement image |
| `MCS tests.nb` | Mathematica analysis notebook |
| `OpenMCS_Peak_Analysis.py` | Standalone Python peak/period/frequency analysis |
| `requirements.txt` | Python analysis dependencies |

Run the Python analysis:

```sh
python -m pip install -r Verification/requirements.txt
python Verification/OpenMCS_Peak_Analysis.py "Verification/500 ns_OpenMCS_100MHz_300MHz_test.csv"
```

Omit the CSV argument to open a file picker. Add `--no-show` to save without opening plot windows. Results are written to a new timestamped folder beside the input: peak table CSV, text/JSON summary, and a three-panel PNG. Use `--help` for column, unit, threshold, and smoothing options.

Dwell comes from the time column; frequency is calculated from peak intervals and a linear fit of peak time versus peak number. Peak detection approximates Mathematica's method and can differ for overlapping/noisy peaks. The notebook's third FindPeaks argument is sharpness, despite its variable name; the Python script treats it as sharpness and exposes a separate optional spacing filter.

The CSV contains a literal backslash-n between metadata and headings; the Python importer handles it. The notebook also contains saved outputs; its original local file-selection path is not needed to use the included CSV.

## Protocol and hardware checks

`python Software/test_live_protocol.py` runs offline parser tests after installing the GUI requirements. It does not open a serial port.

`Software/verify_bin_overlap.py` is a hardware test that opens its configured port (default `COM6`) and controls acquisition. It requires a connected instrument and the correct port setting. It was not run during this package review.

## Hardware acceptance

After initial assembly or any HDL, firmware, toolchain, or wiring change:

1. Check supply/logic voltages, common ground, source selection, and every connection before applying pulses.
2. Run the PWM self-test and confirm approximately 100 µs peak spacing. Repeat cleared acquisitions with identical settings and compare counts/positions.
3. Confirm an exact finite accepted-pass target. Stop partway, resume toward the same total, then check Clear resets accumulation.
4. Request Stop during an active pass and wait for the atomic final frame; confirm no later passes are accepted.
5. Deliberately send triggers during an active pass and verify they do not become queued passes.
6. In Counter mode, compare both inputs to a known pulse source over several integration intervals.
7. Export CSV and PNG, then reopen the CSV and inspect its headers, units, counts, and pass metadata.
8. Verify changed geometry preserves counts as documented, and explicit Clear removes the old accumulation.

Use an independently calibrated source to characterize clock accuracy, input pulse-width limits, rate limits, and timing offsets. A self-test checks functionality but is not a calibration standard.

## Build evidence

The route report identifies Gowin V1.9.11.03 Education and 38/56 BSRAM usage. HTML timing reports and a detailed timing-path file are included in `reports/`. A new build must be assessed using its own generated reports. No fresh FPGA build, RTL simulation, or physical measurement was performed during this package review.
