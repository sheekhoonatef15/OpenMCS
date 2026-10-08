# Operation and features

## MCS acquisition

Set **Bins**, **Dwell**, the trigger edge, and the total pass target in Setup. The “Actual” timing label shows the quantized dwell and pass duration. At 100 MHz:

- Dwell is an integer number of 10 ns clocks, minimum 3 clocks (30 ns).
- Maximum dwell is 4,294,967,295 clocks, approximately 42.95 seconds **per bin**.
- Pass duration = number of bins × actual dwell.

Click **Apply configuration** before a normal internal/external run whenever the geometry changes. This GUI does not automatically apply changed MCS settings on Start. Choose **Run internal** for Teensy-generated triggers or **Run external** to arm for a physical external trigger. Single trigger emits one pulse from Teensy pin 8; acquisition must be armed to accept it.

The FPGA accepts a trigger only when armed and idle. Triggers during an active pass are ignored, not queued. Event pulses count on rising edges; the selectable edge applies to the trigger input. Internal trigger frequency is software-generated and its GUI range is not a guarantee of achievable pulse rate under every workload.

The finite pass limit is an **absolute accumulated target**. For example, after stopping at 200 passes with a target of 1,000, restarting requests the remaining 800. Once the target has been reached, increase it or Clear before another acquisition. Continuous mode removes the finite limit.

## Stop, resume, and clear

**Stop** rejects new triggers and allows an active MCS pass and pending accumulation work to finish. The GUI then accepts the atomic final histogram. A long pass can therefore make Stop appear slow. This is not an immediate pass abort.

**Clear accumulation** stops and clears FPGA histogram/pass accumulation. **Apply configuration preserves them**, including when bins or dwell change. Preserved bins retain their previous counts; these counts are not rebinned or converted to the new time scale. For a new measurement, save, stop, wait for completion, apply settings, and clear explicitly.

Closing the GUI attempts to send Stop before closing the USB connection. It does not wait indefinitely for a final frame and cannot guarantee stopping after a crash, cable disconnection, or power failure. Stop and save before closing when the measurement matters.

## Live histogram and CPS

Running previews can combine bins read at different times. The stopped **atomic final** frame is the coherent measurement to save. Slow dwells (30 µs/bin and above at nominal clock) enable block previews; faster acquisitions use full previews as pass counts advance. Display update timing depends on acquisition and serial throughput; it is not a fixed guaranteed refresh rate.

MCS CPS is estimated from the slope of cumulative histogram counts versus completed passes, divided by active time per pass. It excludes time waiting for triggers. The GUI rebases the fit on restart, retains history, and excludes final frames from the live fit to avoid an artificial endpoint drop. Short acquisitions may not supply enough live samples for a useful estimate. **Clear CPS** resets that history independently of the histogram.

Counter mode measures event and trigger rates from cumulative counters, with separate traces/axes and an adjustable displayed integration interval. Increasing the interval reduces statistical fluctuation but responds more slowly. It does not require MCS triggers to start passes.

## Self-test

1. Connect Teensy 8 through the buffer to H2 and Teensy 9 through the buffer to H4. Disconnect external source outputs from those two paths.
2. Select MCS and enable **Test mode / Self test**.
3. For an initial check, use **3,000 bins × 500 ns**, a **1,000-pass test target**, and rising trigger edge. Clear previous accumulation, then Start.
4. Look for repeated event peaks approximately **100 µs apart**. Exact bin positions depend on PWM phase and path delay. The 1.5 ms pass fits inside the nominal 2 ms trigger period.
5. Stop if needed, wait for the final frame, and export it. Disable self-test before reconnecting external sources.

The active test generator is hardware PWM: nominal **500 Hz H2**, **10 kHz H4**, with narrow pulses. Legacy GUI text mentioning particular peak bins or an n² pulse train does not describe this firmware's active self-test. The stimulus can keep running after the pass target is reached; the FPGA independently stops accepting passes.

The **Internal signal** checkbox sets Teensy pin 9 high/low; it is not a continuous pulse generator. Use Self test for a periodic event train.

## Analysis and display

| Control | Use |
| --- | --- |
| ROI | Select signal and background bin ranges; show gross counts, scaled background, net counts, uncertainty, and SNR |
| Difference | Display signal after subtracting average background per bin |
| Freeze reference | Overlay a captured histogram for comparison; use matching acquisition geometry |
| Time / bin axis | Switch between time and bin number |
| Log Y / Fit Y | Logarithmic counts or live vertical fitting |
| Overview inset | Navigate the full histogram and resize the inset with its handle |
| Clock frequency | Enter measured acquisition-clock frequency for time/rate conversion; does not retune the FPGA PLL |
| Edit layout / themes | Customize panel visibility, sizing/order, labels, and appearance |
| Help / Shortcuts | Show available keyboard controls |

The ROI uncertainty calculation assumes counting statistics and independent signal/background regions; use non-overlapping regions for that interpretation. MCS rate history is an estimate from previews, not a replacement for final histogram analysis.

## Save measurements

Choose an experiment directory and run name, then **Save Data**. Enable **Save image** to also write a PNG; run naming supports incrementing numbered names. **Export CSV** and **Export image** provide separate exports.

MCS **Data Settings** controls column inclusion, names, and order, including Bin, Time (ns), Count, Reference count, and ROI flags. You can omit the Bin column. Counter CSV contains elapsed seconds plus signal and trigger CPS. Save with unchanged geometry and record the clock calibration used.

One retained export quirk: the MCS metadata writer uses a literal `\n` before the CSV column names, so metadata and headings can share a physical line. Inspect the file when choosing import/header handling.

## Troubleshooting

| Symptom | Check |
| --- | --- |
| No connection | Teensy USB data cable/port, firmware upload, serial monitor closed |
| Connected but no FPGA data | FPGA programmed; common ground; TX3 14 → L11 and E10 → RX3 15 |
| No accepted passes | Correct source at H2, edge selection, pin 16 → A10, pin 22 → C10, and target not already reached |
| Passes but no counts | Event pulses at H4, buffer direction/enable, voltage levels, and pulses inside the pass window |
| Stop takes a long time | Active pass duration; wait for its end and final transfer |
| Unexpected peaks after changing dwell | Preserved old histogram; Clear before the next measurement |
| Self-test bins differ from GUI hint | Use the actual PWM frequencies and spacing described above |
| Overflow indication | Distinguish saturated histogram bins from preview stream overrun; reduce load and repeat the measurement as appropriate |
| Works until FPGA power cycle | SRAM image is temporary; reload or program external flash |
