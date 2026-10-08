# OpenMCS

An FPGA multichannel scaler and dual-input pulse counter built with a **Sipeed Tang Primer 25K**, **Teensy 4.0**, and **SN74LVC245A** input buffer. A Python desktop application controls acquisition, displays the histogram and count rates, and exports measurements.

A trigger starts a pass through a sequence of time bins. Event pulses are counted in those bins, and repeated passes accumulate into the same histogram. The FPGA performs acquisition; the Teensy bridges the FPGA to the computer over USB.

## Features

| Feature | What it provides |
| --- | --- |
| Triggered MCS | 1â€“16,384 bins with repeated-pass accumulation |
| Adjustable dwell | 30 ns minimum; 10 ns steps at the nominal 100 MHz acquisition clock |
| Wide count range | 32-bit histogram bins with saturation and overflow reporting; 64-bit cumulative input counters |
| Trigger selection | External trigger, Teensy-generated internal trigger, or single test trigger; rising/falling trigger edge selection |
| Run control | Finite absolute pass target or continuous acquisition; Stop completes the active pass |
| Live display | Running previews and a coherent final histogram after acquisition stops |
| Counter mode | Event and trigger CPS traces with adjustable 1 msâ€“50 s displayed dwell |
| Analysis | Signal/background regions, net counts, uncertainty, SNR, frozen reference, and background-subtracted display |
| Plot controls | Histogram steps, time/bin axis, log Y, Fit Y, zoom/pan, and independently resizable overview inset |
| Data export | CSV, plot images, experiment folder/run naming, and configurable MCS export columns |
| Usability | Count-rate history, elapsed/remaining-time display, clock calibration, themes, layout controls, and keyboard shortcuts |
| Built-in stimulus | Teensy PWM self-test: nominal 500 Hz trigger and 10 kHz event signal |

The input sampler runs at **300 MHz**. This is distinct from the **100 MHz histogram clock**: minimum bin width is **30 ns**, not 3.33 ns. Sampling frequency is not a guaranteed maximum count rate.

## Get started

1. Assemble the connections in [Hardware and wiring](OpenMCS_V1/docs/HARDWARE.md).
2. Program the FPGA and Teensy using [Build and installation](OpenMCS_V1/docs/BUILD.md).
3. Install Python 3.10+ and start the application from the repository root:

   ```sh
   python -m pip install -r OpenMCS_V1/OpenMCS_V1/Software/requirements.txt
   python OpenMCS_V1/OpenMCS_V1/Software/OpenMCS_GUI.py
   ```

4. Select the **Teensy USB serial port** and click **Connect**.
5. Select **MCS**, open **Setup**, set bins/dwell/pass target, and click **Apply configuration**.
6. Use **Run internal** with Teensy pin 8 connected to the trigger input, or **Run external** with an external trigger source.
7. Click **Stop**, wait for the completed final histogram, then save the data.

On Windows, `OpenMCS_V1/OpenMCS_V1/Software/Run_OpenMCS.bat` can launch the GUI after dependencies are installed.

**Changing bins or dwell preserves existing histogram memory and the accumulated pass count.** Clear accumulation explicitly before starting a new measurement with different timing. Otherwise counts acquired under different settings are mixed.

## Documentation

- [Hardware and wiring](OpenMCS_V1/docs/HARDWARE.md): every acquisition/control connection, Dock connector positions, buffer wiring, and external/internal source selection.
- [Build and installation](OpenMCS_V1/docs/BUILD.md): FPGA build settings, programming, Teensy upload, and Python setup.
- [Operation and features](OpenMCS_V1/docs/OPERATION.md): acquisition workflow, self-test, analysis, export, and troubleshooting.
- [Verification and example data](OpenMCS_V1/docs/VERIFICATION.md): integrity checks, sample measurement, and offline peak analysis.

## Repository contents

| Path | Contents |
| --- | --- |
| `src/` | FPGA Verilog, PLL, pin assignments, and timing constraints |
| `OpenMCS_Primer25K.gprj` | Gowin FPGA project |
| `impl/OpenMCS_Primer25K_process_config.json` | Supplied build settings retained for rebuilding |
| `OpenMCS_Teensy4_0_100MHz/` | USB/UART bridge and test-signal firmware |
| `Software/` | Main GUI, dependencies, launcher, and hardware-check utilities |
| `bitstreams/` | Supplied `.fs` and `.bin` FPGA programming files |
| `reports/` | Supplied route/timing evidence |
| `Verification/` | Example CSV/PNG, Mathematica notebook, Python analysis, and integrity checker |

The repository includes source code and prebuilt FPGA programming images. Build instructions and hardware verification procedures are provided in the guides above.


