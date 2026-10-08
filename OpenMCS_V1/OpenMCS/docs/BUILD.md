# Build and installation

## 1. Program the FPGA

The included image can be used without rebuilding:

`bitstreams/OpenMCS_Primer25K.fs`

1. Seat the Tang Primer 25K core board on its Dock and connect the Dock USB-C data cable.
2. Install Gowin EDA and the programmer/debugger driver appropriate for the Dock. Follow [Sipeed's setup guide](https://wiki.sipeed.com/hardware/en/tang/tang-primer-25k/primer-25k.html).
3. Open Gowin Programmer, detect the cable/device, and select the **GW5A-25, version A** target matching `GW5A-LV25MG121NC1/I0`.
4. Select the JTAG **SRAM Program** operation, choose the supplied `.fs`, and program it. Confirm successful completion.
5. SRAM programming is temporary: reload after power loss. For standalone startup, follow Sipeed's **Download to External FLASH** instructions for this board and programmer version, then power-cycle and verify acquisition. Flash and SRAM operations are different; do not assume a successful SRAM load saved the design permanently.

The `.bin` is retained as an alternative programming artifact. Use the `.fs` for the SRAM workflow above.

### Rebuild from source

The supplied route report identifies **Gowin V1.9.11.03 Education**. Use that version where possible; different tools/settings may produce different placement and timing.

1. Open `OpenMCS_Primer25K.gprj` in Gowin EDA.
2. Confirm device `GW5A-LV25MG121NC1/I0`, version A, and top module **`mcs_step8`**.
3. Confirm all files in the project are enabled, including both `.cst` and `.sdc` constraints. Keep the supplied PLL and matched input placement constraints.
4. Check the retained `impl/OpenMCS_Primer25K_process_config.json` settings. If the IDE does not import them, apply the relevant settings manually:

   | Setting | Supplied value |
   | --- | --- |
   | Synthesis | GowinSyn; Verilog 2001 |
   | Dual-purpose CPU pin | Use as regular I/O (`CPU: true`); needed for clock pin E2 |
   | Place option / route option | 2 / 1 |
   | Timing-driven routing | Enabled |
   | Correct hold violations | Enabled |
   | Replicate resources | Enabled |
   | Route maximum fanout | 8 |
   | Input/output I/O register placement | Enabled |
   | Promote physical-constraint warnings to errors | Enabled |
   | Convert SDP32/36 to SDP16/18 | Enabled |

5. Run **Synthesis**, then **Place & Route**. Resolve errors and inspect setup/hold timing, clock definitions, and unconstrained paths before programming.
6. The rebuilt image is normally `impl/pnr/OpenMCS_Primer25K.fs`. Program that file when testing your rebuild; `bitstreams/` remains the supplied image.
7. Repeat the hardware checks in [Verification](VERIFICATION.md).

The design uses the board's 50 MHz input, a 100 MHz acquisition clock, and a 300 MHz sampler clock. The retained filename `gowin_pll_150.v` is historical; it does not mean the current acquisition clock is 150 MHz. No external RAM is required.

## 2. Program the Teensy

1. Install Arduino IDE with [PJRC Teensy support](https://www.pjrc.com/teensy/td_download.html).
2. Open:

   `OpenMCS_Teensy4_0_100MHz/OpenMCS_Teensy4_0_100MHz.ino`

3. Select **Teensy 4.0**, **USB Type: Serial**, and the normal **600 MHz** CPU setting.
4. Connect the Teensy USB data cable and upload. Use its Program button if the loader requests it.
5. Close Serial Monitor before connecting the OpenMCS GUI.
6. After programming, switch off power and connect Dock +5 V to Teensy VIN and join grounds as described in [Hardware and wiring](HARDWARE.md). Isolate VIN from USB VUSB before using external power alongside USB. If the VIN–VUSB link is already cut, supply external VIN power during upload as well.

The sketch uses the Teensy core APIs; no additional Arduino library is needed. Keep the sketch folder and `.ino` basename matching.

## 3. Install and run the GUI

Python 3.10+ is required. From the repository root:

```sh
python -m venv .venv
```

Activate on Windows PowerShell:

```powershell
.venv\Scripts\Activate.ps1
```

Or on Linux/macOS:

```sh
source .venv/bin/activate
```

Then:

```sh
python -m pip install -r Software/requirements.txt
python Software/OpenMCS_GUI.py
```

Dependencies are NumPy, pyserial, PyQt6, and pyqtgraph. Alternatively, the Windows launcher uses the installed `py -3`/`python` interpreter; install dependencies for that interpreter if using the launcher outside the virtual environment.

Select the **Teensy** port in the GUI, not the Dock debugger port. The GUI checks the firmware identity. If the OS denies access, close other serial applications and check local serial-device permissions.

## Offline measurement analysis

Install the separate analysis dependencies with `python -m pip install -r Verification/requirements.txt`. See [Verification and example data](VERIFICATION.md) for the included measurement and analysis command.
