#!/usr/bin/env python3
"""Python adaptation of 'MCS tests.nb'; requires numpy, scipy, matplotlib.

Run without arguments for a file picker, or:
    python OpenMCS_Peak_Analysis.py measurement.csv
    python OpenMCS_Peak_Analysis.py measurement.csv --time-column 2 --counts-column 3
    python OpenMCS_Peak_Analysis.py measurement.csv --no-show

Uses the TIME column, never a bin column. Column numbers below are 1-based.
Saves the table, summary, and three-panel plot in a NEW directory beside the input.
Peak detection is an explicitly approximate SciPy adaptation of Wolfram FindPeaks,
not a bit-for-bit replacement. The downstream statistical formulas are preserved.
"""
from __future__ import annotations

# USER SETTINGS -- match the notebook's selected columns and time unit.
TIME_COLUMN = 1
COUNTS_COLUMN = 2
TIME_UNIT = "ns"                    # s, ms, us, ns
SMOOTHING_RADIUS_BINS = 1           # GaussianFilter radius, not sigma
PEAK_SCALE_BINS = 1.0               # FindPeaks second argument
MINIMUM_PEAK_SHARPNESS = 5.0         # Notebook's misleadingly named separation value
MINIMUM_PEAK_SEPARATION_BINS = 0     # 0 disables this EXTRA spacing filter
THRESHOLD_FRACTION = 0.0            # median + fraction * (maximum - median)

import argparse
import csv
from datetime import datetime
import io
import json
from pathlib import Path
import re
import sys

import numpy as np
from scipy.ndimage import convolve1d, gaussian_filter1d
from scipy.signal import find_peaks
from scipy.special import ive

UNIT_SCALE = {"s": 1.0, "ms": 1e-3, "us": 1e-6, "ns": 1e-9}


def number(text):
    """Read ordinary, scientific, and Wolfram-style numeric text, without eval."""
    text = text.strip().replace("\u2212", "-")
    text = re.sub(r"`[\d.]*", "", text).replace("*^", "e")
    return float(text)


def read_data(path, time_column=1, counts_column=2, unit="ns"):
    if min(time_column, counts_column) < 1 or time_column == counts_column:
        raise ValueError("Select distinct positive, 1-based time and counts columns.")
    if unit not in UNIT_SCALE:
        raise ValueError("Time unit must be s, ms, us, or ns.")
    text = Path(path).read_text(encoding="utf-8-sig")
    # Some OpenMCS exports place a literal backslash-n between metadata and headings.
    text = text.replace("\\n", "\n")
    lines = [line for line in text.splitlines()
             if line.strip() and not line.lstrip().startswith(("#", "//"))]
    if not lines:
        raise ValueError("No data found in the selected file.")
    # CSV, tab-separated, semicolon-separated, or plain whitespace numeric data.
    delimiter = next((d for d in (",", "\t", ";") if any(d in s for s in lines[:20])), None)
    rows = csv.reader(io.StringIO("\n".join(lines)), delimiter=delimiter) if delimiter else (
        re.split(r"\s+", line.strip()) for line in lines)
    values, skipped = [], 0
    for row in rows:
        try:
            t, c = number(row[time_column - 1]), number(row[counts_column - 1])
        except (ValueError, IndexError):
            skipped += 1
            continue
        if not (np.isfinite(t) and np.isfinite(c)):
            skipped += 1
            continue
        values.append((t * UNIT_SCALE[unit], c))
    if len(values) < 3:
        raise ValueError("Fewer than three numeric rows. Check column settings and delimiter.")
    data = np.asarray(values, dtype=float)
    data = data[np.argsort(data[:, 0], kind="stable")]
    t, counts = data.T
    if np.any(counts < 0):
        raise ValueError("Negative counts found; select a raw-count histogram column.")
    gaps = np.diff(t)
    if np.any(gaps <= 0):
        raise ValueError("Duplicate time values found. Each histogram row must have a distinct time.")
    dwell = float(np.median(gaps))
    notes = []
    if skipped:
        notes.append(f"Skipped {skipped} nonnumeric/header/nonfinite row(s).")
    if not np.allclose(gaps, dwell, rtol=1e-5, atol=dwell * 1e-9):
        notes.append("Time spacing is nonuniform: dwell is the median positive spacing; "
                     "smoothing is by row index. Check for missing bins or wrong columns.")
    return t, counts, dwell, notes


def smooth_counts(counts, radius):
    """Normalized discrete Bessel Gaussian, sigma=radius/2, fixed-edge padding.

    Approximates Mathematica GaussianFilter's default discrete-kernel behavior.
    No claim of exact kernel/edge agreement with a particular Wolfram release.
    """
    if radius == 0:
        return counts.copy()
    sigma = radius / 2.0
    offsets = np.arange(-radius, radius + 1)
    weights = ive(np.abs(offsets), sigma * sigma)
    weights /= weights.sum()
    return convolve1d(counts, weights, mode="nearest")


def detect_peaks(smoothed, scale, sharpness, threshold, separation):
    """Approximate FindPeaks scale survival, back-mapping, and sharpness.

    Find maxima after scale smoothing. Within each regularized peak's basin,
    map back to the nearest maximum of the pre-regularized signal. Use the
    negative Gaussian second derivative for sharpness. Plateaus use their
    fractional center, matching the notebook's linear position interpolation.
    Boundary peaks are excluded; visually inspect the first/last peak.
    """
    raw_idx, props = find_peaks(smoothed, plateau_size=(1, None))
    positions = (props["left_edges"] + props["right_edges"]) / 2.0
    if not positions.size:
        return positions
    if scale > 0:
        regularized = gaussian_filter1d(smoothed, scale, mode="reflect")
        regular_idx, _ = find_peaks(regularized)
        # Subtract a constant before differentiating to avoid truncated-kernel DC bias.
        curvature = -gaussian_filter1d(smoothed - np.median(smoothed), scale,
                                      order=2, mode="reflect")
        selected = []
        for k, peak in enumerate(regular_idx):
            left = (regular_idx[k - 1] + peak) / 2 if k else 0
            right = (peak + regular_idx[k + 1]) / 2 if k + 1 < len(regular_idx) else len(smoothed) - 1
            candidates = positions[(positions >= left) & (positions <= right)]
            if candidates.size:
                selected.append(candidates[np.argmin(np.abs(candidates - peak))])
        positions = np.unique(selected)
    else:
        curvature = -np.gradient(np.gradient(smoothed))
    grid = np.arange(len(smoothed))
    heights = np.interp(positions, grid, smoothed)
    curvatures = np.interp(positions, grid, curvature)
    positions = positions[(heights > threshold) & (curvatures >= sharpness)]
    if separation > 0:
        # Optional extra filter: keep higher peaks first, then restore time order.
        ranked = sorted(positions, key=lambda p: np.interp(p, grid, smoothed), reverse=True)
        kept = []
        for p in ranked:
            if all(abs(p - q) >= separation for q in kept):
                kept.append(p)
        positions = np.sort(kept)
    return np.asarray(positions, dtype=float)


def analyze(t, counts, dwell, *, radius=SMOOTHING_RADIUS_BINS,
            scale=PEAK_SCALE_BINS, sharpness=MINIMUM_PEAK_SHARPNESS,
            fraction=THRESHOLD_FRACTION, separation=MINIMUM_PEAK_SEPARATION_BINS):
    if not isinstance(radius, (int, np.integer)) or radius < 0:
        raise ValueError("Smoothing radius must be a nonnegative integer.")
    if not all(np.isfinite(v) for v in (scale, sharpness, fraction, separation)):
        raise ValueError("Detection settings must be finite numbers.")
    if min(scale, sharpness, separation) < 0 or not 0 <= fraction <= 1:
        raise ValueError("Scale, sharpness and spacing must be >= 0; threshold fraction must be 0–1.")
    smoothed = smooth_counts(counts, radius)
    baseline = float(np.median(smoothed))
    threshold = baseline + fraction * (float(np.max(smoothed)) - baseline)
    pos = detect_peaks(smoothed, scale, sharpness, threshold, separation)
    if len(pos) < 2:
        raise ValueError("Fewer than two peaks detected. Check columns/units, then reduce "
                         "--sharpness, --threshold-fraction, --peak-scale, or --min-distance.")
    grid = np.arange(len(t))
    peak_t = np.interp(pos, grid, t)
    peak_counts = np.interp(pos, grid, counts)  # Counts from original data, like notebook
    periods = np.diff(peak_t)
    n = np.arange(len(pos), dtype=float)       # Fit starts at n=0; table starts at peak 1
    centered_n = n - n.mean()
    relative_t = peak_t - peak_t[0]
    slope = float(np.dot(centered_n, relative_t - relative_t.mean()) / np.dot(centered_n, centered_n))
    intercept_relative = float(relative_t.mean() - slope * n.mean())
    fitted_relative = intercept_relative + slope * n
    residual = relative_t - fitted_relative
    ss_total = float(np.sum((relative_t - relative_t.mean()) ** 2))
    r_squared = 1.0 - float(np.dot(residual, residual)) / ss_total
    summary = {
        "data_points": len(t), "dwell_s": dwell, "dwell_ns": dwell * 1e9,
        "time_range_us": float((t[-1] - t[0]) * 1e6), "detected_peaks": len(pos),
        "baseline_counts": baseline, "peak_threshold_counts": threshold,
        "median_period_s": float(np.median(periods)),
        "median_frequency_hz": float(1 / np.median(periods)),
        "mean_period_s": float(np.mean(periods)),
        "mean_frequency_hz": float(1 / np.mean(periods)),
        "period_sample_std_s": float(np.std(periods, ddof=1)) if len(periods) > 1 else None,
        "fitted_period_s": slope, "fitted_frequency_hz": 1 / slope,
        "fit_intercept_s": float(peak_t[0] + intercept_relative), "r_squared": r_squared,
    }
    return {"time": t, "counts": counts, "smoothed": smoothed, "positions": pos,
            "peak_times": peak_t, "peak_counts": peak_counts, "periods": periods,
            "frequencies": 1 / periods, "peak_numbers": n,
            "fitted_times": peak_t[0] + fitted_relative, "summary": summary}


def summary_text(result, source, notes):
    s = result["summary"]
    std = s["period_sample_std_s"]
    lines = [f"Imported file: {source}", "", "ACQUISITION INFORMATION",
             f"Number of data points: {s['data_points']}",
             f"Calculated dwell: {s['dwell_s']:.10g} s/bin = {s['dwell_ns']:.9f} ns/bin",
             f"Total time range: {s['time_range_us']:.9f} us", "", "PEAK / FREQUENCY ANALYSIS",
             f"Number of detected peaks: {s['detected_peaks']}"]
    for prefix in ("median", "mean", "fitted"):
        label = "Linear-fit" if prefix == "fitted" else prefix.title()
        p = s[f"{prefix}_period_s"]
        lines += [f"{label} period: {p:.12g} s = {p * 1e6:.9f} us",
                  f"{label} frequency: {s[f'{prefix}_frequency_hz']:.12g} Hz"]
    lines += ["Period standard deviation (sample): " +
              (f"{std:.12g} s = {std * 1e6:.9g} us" if std is not None else
               "undefined (only one interval)"), f"Linear-fit R-squared: {s['r_squared']:.12g}",
              "", "Peak detection: approximate Wolfram-style SciPy implementation.",
              "Mean frequency means 1 / mean(periods), not mean(1 / periods).",
              "Missing/extra peaks bias the frequency estimate; inspect the marked spectrum."]
    lines += [f"NOTE: {note}" for note in notes]
    return "\n".join(lines) + "\n"


def table_rows(result):
    for i, (t, c) in enumerate(zip(result["peak_times"], result["peak_counts"])):
        p = result["periods"][i - 1] if i else None
        yield [i + 1, float(t), float(t * 1e6), float(c),
               float(p) if p is not None else "",
               float(p * 1e6) if p is not None else "",
               float(1 / p) if p is not None else ""]


HEADINGS = ["Peak", "Time (s)", "Time (us)", "Counts", "Period (s)", "Period (us)", "Frequency (Hz)"]


def make_plot(result):
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(3, 1, figsize=(11, 11), layout="constrained")
    s = result["summary"]
    ax = axes[0]
    ax.plot(result["time"] * 1e6, result["counts"], color="royalblue", lw=0.9, label="Counts")
    ax.scatter(result["peak_times"] * 1e6, result["peak_counts"], color="crimson", s=23,
               zorder=3, label="Detected peaks")
    ax.set(title="OpenMCS spectrum and detected peaks", xlabel="Time after trigger (µs)", ylabel="Counts")
    ax.legend(loc="upper right")
    ax = axes[1]
    ax.scatter(np.arange(1, len(result["frequencies"]) + 1), result["frequencies"], s=25, color="royalblue")
    ax.set(title=f"Peak-to-peak frequency — median {s['median_frequency_hz']:.10g} Hz",
           xlabel="Peak interval", ylabel="Frequency (Hz)")
    # Avoid magnifying floating-point roundoff for a perfectly periodic signal.
    freq = result["frequencies"]
    center = float(np.median(freq))
    if np.ptp(freq) < 1000 * np.finfo(float).eps * max(abs(center), 1.0):
        margin = max(abs(center) * 1e-6, 1e-6)
        ax.set_ylim(center - margin, center + margin)
    ax.ticklabel_format(axis="y", style="plain", useOffset=False)
    ax = axes[2]
    ax.scatter(result["peak_numbers"], result["peak_times"] * 1e6, s=25, label="Peak times")
    ax.plot(result["peak_numbers"], result["fitted_times"] * 1e6, color="crimson", label="Linear fit")
    ax.set(title=f"Peak-time fit — {s['fitted_frequency_hz']:.10g} Hz; R² = {s['r_squared']:.9g}",
           xlabel="Peak number (fit starts at 0)", ylabel="Peak time (µs)")
    ax.legend()
    for ax in axes:
        ax.grid(alpha=0.22)
    return fig


def choose_file():
    try:
        import tkinter as tk
        from tkinter import filedialog
        root = tk.Tk()
        root.withdraw()
        try:
            downloads = Path.home() / "Downloads"
            return filedialog.askopenfilename(title="Select an OpenMCS histogram",
                    initialdir=str(downloads if downloads.exists() else Path.home()),
                    filetypes=[("Data files", "*.csv *.txt *.tsv"), ("All files", "*.*")])
        finally:
            root.destroy()
    except (ImportError, RuntimeError) as exc:
        raise ValueError("File picker unavailable. Supply the CSV path on the command line.") from exc
    except Exception as exc:
        raise ValueError("Cannot open file picker. Run with a CSV path argument.") from exc


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("file", nargs="?", help="CSV/text histogram; omitted opens file picker")
    parser.add_argument("--time-column", type=int, default=TIME_COLUMN)
    parser.add_argument("--counts-column", type=int, default=COUNTS_COLUMN)
    parser.add_argument("--time-unit", choices=UNIT_SCALE, default=TIME_UNIT)
    parser.add_argument("--smoothing-radius", type=int, default=SMOOTHING_RADIUS_BINS)
    parser.add_argument("--peak-scale", type=float, default=PEAK_SCALE_BINS)
    parser.add_argument("--sharpness", type=float, default=MINIMUM_PEAK_SHARPNESS)
    parser.add_argument("--min-distance", type=float, default=MINIMUM_PEAK_SEPARATION_BINS)
    parser.add_argument("--threshold-fraction", type=float, default=THRESHOLD_FRACTION)
    parser.add_argument("--output-parent", type=Path, help="Parent folder for a new results directory")
    parser.add_argument("--no-show", action="store_true", help="Save results/plots without opening windows")
    args = parser.parse_args()
    try:
        filename = args.file or choose_file()
        if not filename:
            print("File selection cancelled.")
            return 0
        source = Path(filename).expanduser().resolve()
        t, c, dwell, notes = read_data(source, args.time_column, args.counts_column, args.time_unit)
        result = analyze(t, c, dwell, radius=args.smoothing_radius, scale=args.peak_scale,
                         sharpness=args.sharpness, fraction=args.threshold_fraction, separation=args.min_distance)
        report = summary_text(result, source, notes)
        print(report)
        print("DETECTED PEAK TABLE")
        print(" | ".join(HEADINGS))
        for row in table_rows(result):
            print(" | ".join(f"{x:.10g}" if isinstance(x, float) else str(x) for x in row))
        if args.no_show:
            import matplotlib
            matplotlib.use("Agg")
        fig = make_plot(result)
        parent = args.output_parent.expanduser() if args.output_parent else source.parent
        parent.mkdir(parents=True, exist_ok=True)
        out = parent / f"{source.stem}_analysis_{datetime.now():%Y%m%d_%H%M%S_%f}"
        out.mkdir(exist_ok=False)
        (out / "summary.txt").write_text(report, encoding="utf-8")
        settings = {k: str(v) if isinstance(v, Path) else v for k, v in vars(args).items()}
        (out / "summary.json").write_text(json.dumps({"source": str(source), "settings": settings,
            "results": result["summary"], "notes": notes}, indent=2, allow_nan=False), encoding="utf-8")
        with (out / "peaks.csv").open("w", newline="", encoding="utf-8") as f:
            writer = csv.writer(f)
            writer.writerow(HEADINGS)
            writer.writerows(table_rows(result))
        fig.savefig(out / "analysis.png", dpi=180)
        print(f"\nSaved results: {out}")
        if not args.no_show:
            import matplotlib.pyplot as plt
            plt.show()
        return 0
    except (OSError, ValueError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
