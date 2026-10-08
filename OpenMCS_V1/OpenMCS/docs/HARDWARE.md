# Hardware and wiring

## Parts

- Tang Primer 25K core board and 25K Dock; project target `GW5A-LV25MG121NC1/I0`, device version A.
- Teensy 4.0 with USB data cable.
- SN74LVC245A and suitable breakout/socket, plus a 100 nF supply bypass capacitor.
- Dock USB-C data/power cable, short signal wiring, and common ground connections.
- External digital event/trigger sources, or the Teensy outputs for testing.

The mappings below come from `src/OpenMCS_Primer25K.cst`, the Teensy sketch, PJRC's UART table, and Sipeed's **Dock 60033** schematic. `H2`, `H4`, etc. are **FPGA package pin names**, not connector pin numbers. `J3 pin 18` means the physical connector contact in that schematic. Confirm the connector designator, pin-1 mark, and board revision before wiring; use continuity checking when uncertain.

## FPGA–Teensy connections

| Purpose / FPGA signal | FPGA pin | Dock connector | Teensy 4.0 pin | Direction |
| --- | --- | --- | --- | --- |
| UART command input, `uart_rx` | L11 | J5 pin 2 | 14 / TX3 | Teensy → FPGA |
| UART data output, `uart_tx` | E10 | J4 pin 2 | 15 / RX3 | FPGA → Teensy |
| Accepted-pass toggle, `pass_toggle_out` | K5 | J5 pin 3 | 13 | FPGA → Teensy |
| Trigger acceptance enable, `accept_enable_in` | A10 | J4 pin 8 | 16 | Teensy → FPGA |
| Stop request, `stop_request_in` | C10 | J4 pin 4 | 22 | Teensy → FPGA |
| Stop acknowledgement, `stop_ack_out` | B10 | J4 pin 6 | 23 | FPGA → Teensy |
| Trigger input, `trigger_in` | H2 | J3 pin 18 | 8, when using internal/test trigger | Source → buffer → FPGA |
| Event input, `event_in` | H4 | J3 pin 35 | 9, when using test stimulus | Source → buffer → FPGA |
| Power after programming | +5 V rail | J3 pin 39 | VIN (5 V power input) | Dock → Teensy |
| Common ground | GND | J3 pin 37, or J4/J5 pin 9 or 10 | GND | Shared reference |

The UART and control lines are direct **3.3 V logic** connections. Cross TX to RX. The FPGA UART is **5,000,000 baud, 8-N-1**. Teensy `Serial3` uses TX pin 14 and RX pin 15; USB `Serial` is the separate PC connection.

Pin 13 monitors a toggle on each **accepted** pass; it is not a trigger output. Pin 16 high permits trigger acceptance; pin 22 high requests stop. Pin B10 reports `~run_enabled`, so it is not proof that histogram readout has finished. The supplied firmware configures pin 23 as an input but does not wait on it; the GUI recognizes completion from received frames.

## SN74LVC245A input buffer

**Reference two-channel wiring:** the A1/A2 assignments below define the buffer layout described in this guide. A schematic of the original physical assembly is not included. These pin numbers apply to the conventional 20-pin N/DW/DB/NS/PW/DGV packages; check your exact package or breakout labels.

| Buffer pin | Connect to |
| --- | --- |
| 20 — VCC | Dock 3.3 V, e.g. J4 pin 11 or 12 |
| 10 — GND | Power after programming | +5 V rail | J3 pin 39 | VIN (5 V power input) | Dock → Teensy |
| Common ground |
| 1 — DIR | 3.3 V, selecting A → B |
| 19 — active-low OE | GND, enabling outputs while powered |
| 2 — A1 | Selected trigger source |
| 18 — B1 | FPGA H2 / Dock J3 pin 18 |
| 3 — A2 | Selected event source |
| 17 — B2 | FPGA H4 / Dock J3 pin 35 |
| 4–9 — unused A inputs | GND |
| 11–16 — unused B outputs | Leave unconnected |

Place **100 nF between pins 20 and 10** close to the IC. Both used channels are non-inverting. At a 3.3 V supply, this part accepts input levels up to 5.5 V under its specified operating conditions and produces 3.3 V logic outputs. Do not apply 5 V to Teensy/FPGA signal pins, and do not power this buffer at 5 V.

Use clean positive logic pulses within the datasheet limits. The buffer is not a discriminator for raw detector signals or a negative-pulse/NIM converter. Keep input and output leads short, provide nearby ground returns, and match event/trigger cable lengths when relative timing matters. Do not float the selected source inputs when unplugged; provide a defined low level, for example with 10 kΩ pulldowns at A1/A2.

## Choose one source per input

| Use | Trigger source → A1 | Event source → A2 | GUI action |
| --- | --- | --- | --- |
| Built-in self-test | Teensy pin 8 | Teensy pin 9 | Enable Test mode / Self test, then Start |
| Internal-trigger experiment | Teensy pin 8 | External event pulses | Apply configuration, Run internal |
| External-trigger experiment | External trigger pulses | External event pulses | Apply configuration, Run external |
| Continuous counter | External trigger pulses, if measured | External event pulses | COUNTER, Start |

Use removable jumpers or a source-selection switch. **Never join an external generator output to Teensy pin 8 or 9.** The firmware keeps these pins as outputs even when their stimulus is inactive. Disable self-test and physically select the desired source before using external signals. All sources share the instrument ground.

## Power, clock, and auxiliary outputs

After programming, the instrument connects the **FPGA Dock +5 V rail to the Teensy 5 V power input (VIN)**, with **Dock GND connected to Teensy GND**. On the Dock 60033 schematic, +5 V is available at J3 pin 39. The Dock supplies the Teensy; its supply must support both boards and attached circuitry. Make the power connection with both boards switched off.

The Teensy USB cable remains connected to the PC for GUI data. **Isolate USB VUSB from VIN when powering VIN externally** to avoid tying the Dock supply to USB power. PJRC provides a cuttable VIN–VUSB link on the underside of Teensy 4.0; follow the [official board instructions](https://www.pjrc.com/store/teensy40.html) and [back-side pinout](https://www.pjrc.com/teensy/card10b_rev2_web.pdf). Once that link is cut, external VIN power is also required while programming over USB.

Do not parallel the boards' regulated 3.3 V outputs. Power the buffer from the Dock 3.3 V rail. The 5 V interconnect is a power connection only; all FPGA/Teensy signal wiring remains 3.3 V logic.

| FPGA signal | Pin | Purpose |
| --- | --- | --- |
| `clk_50m` | E2 | Board's existing 50 MHz oscillator input; no Teensy clock wire |
| `led0_n` | L6 | Active-low PLL-lock indication; this Dock net is also USB D+ |
| `led1_n` | A11 / J4 pin 7 | Active-low armed indication |

These HDL LED names do not imply dedicated user LEDs are fitted at those locations. No external LED connection is needed. Do not attach a USB peripheral to the Dock USB-A connector while this design drives its shared L6 net. USB-C is used for FPGA programming/power; GUI data travels through the Teensy USB cable. The Dock debugger's default UART pins B3/C3 are not this design's UART pins.

## Manufacturer references

- [Sipeed Dock 60033 schematic, sheet 1](https://dl.sipeed.com/fileList/TANG/Primer_25K/02_Schematic/Tang_Primer_25K_Dock_60033_Schematic.pdf)
- [Tang Primer 25K documentation](https://wiki.sipeed.com/hardware/en/tang/tang-primer-25k/primer-25k.html)
- [PJRC hardware UART assignments](https://www.pjrc.com/teensy/td_uart.html)
- [PJRC Teensy pinout cards](https://www.pjrc.com/teensy/pinout.html)
- [TI SN74LVC245A datasheet](https://www.ti.com/lit/ds/symlink/sn74lvc245a.pdf)
