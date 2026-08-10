#!/usr/bin/env python3
"""
Usage:
    python average_timings.py file1.csv [file2.csv ...]
    python average_timings.py file1.csv --outdir ./plots
    python average_timings.py file1.csv --title "title"
    python average_timings.py file1.csv --no-plot

    python average_timings.py
"""

import argparse
import csv
import sys
import glob
import os
from statistics import mean, stdev

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.axes_grid1.inset_locator import inset_axes

IGNORE_COLUMNS = {"timestamp", "success", "ok", "valid", "frame", "frame_id", "index", "id"}

COLUMN_DESCRIPTIONS = {
    "extraction_ms": "Local Cloud Extraction",
    "plane_fit_ms": "Plane Fitting",
    "validation_ms": "Validation",
    "post_fit_ms": "Scoring + Fallback",
    "smoothing_ms": "Smoothing",
    "total_ms": "Total",
}


def describe_column(name):
    return COLUMN_DESCRIPTIONS.get(name, name)


def is_float(value: str) -> bool:
    try:
        float(value)
        return True
    except (ValueError, TypeError):
        return False


def load_numeric_columns(path):
    with open(path, newline="") as f:
        reader = csv.DictReader(f)
        if reader.fieldnames is None:
            return {}, 0

        columns = {name: [] for name in reader.fieldnames if name not in IGNORE_COLUMNS}
        row_count = 0

        for row in reader:
            row_count += 1
            for name in columns:
                raw = row.get(name, "")
                if is_float(raw):
                    columns[name].append(float(raw))

    columns = {name: vals for name, vals in columns.items() if vals}
    return columns, row_count


def load_timestamps(path):
    timestamps = []
    with open(path, newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            raw = row.get("timestamp", "")
            if is_float(raw):
                timestamps.append(float(raw))
    return timestamps


def summarize_file(path):
    columns, row_count = load_numeric_columns(path)

    print(f"\n=== {os.path.basename(path)} ===")
    print(f"Rows: {row_count}")

    if not columns:
        print("No timing columns found")
        return columns

    ordered_names = []
    if "total_ms" in columns:
        ordered_names.append("total_ms")
    ordered_names += sorted(n for n in columns if n != "total_ms")

    col_width = max(len(n) for n in ordered_names) + 2

    for name in ordered_names:
        vals = columns[name]
        avg = mean(vals)
        sd = stdev(vals) if len(vals) > 1 else 0.0
        mn, mx = min(vals), max(vals)
        marker = " <-- main metric" if name == "total_ms" else ""
        print(
            f"  {name:<{col_width}} avg={avg:8.4f} ms  "
            f"std={sd:7.4f}  min={mn:7.4f}  max={mx:7.4f}  n={len(vals)}{marker}"
        )

    if "total_ms" in columns:
        print(f"\n  >> Average total_ms: {mean(columns['total_ms']):.4f} ms")

    return columns


def plot_file(path, columns, outdir, title):
    if not columns:
        return

    timestamps = load_timestamps(path)
    if timestamps:
        t0 = timestamps[0]
        times = [t - t0 for t in timestamps]
    else:
        n = len(next(iter(columns.values())))
        times = list(range(n))

    sub_names = sorted(n for n in columns if n != "total_ms")
    has_subs = len(sub_names) > 0

    n_rows = 2 if has_subs else 1
    fig, axes = plt.subplots(n_rows, 1, figsize=(12, 4 * n_rows + 1), sharex=True)
    axes = [axes] if n_rows == 1 else list(axes)

    ax = axes[0]
    if "total_ms" in columns:
        vals = columns["total_ms"]
        avg, mx = mean(vals), max(vals)
        ax.plot(times[:len(vals)], vals, color="tab:blue", linewidth=1)
        ax.axhline(avg, color="tab:blue", linestyle="--", linewidth=1.2, label=f"avg={avg:.3f} ms")
        ax.axhline(mx, color="tab:green", linestyle="-.", linewidth=1.2, label=f"max={mx:.3f} ms")
        ax.axhline(50, color="red", linestyle="-", linewidth=1.2, label="real-time limit (50 ms)")
        ax.set_ylabel("Processing Time (ms)", fontsize=11)
        ax.set_title("Total Computation Time", fontsize=11)
        ax.legend(loc="upper right", fontsize=11)
        ax.set_ylim(0,60)
    else:
        ax.set_visible(False)
    ax.grid(alpha=0.3)

    if has_subs:
        ax = axes[1]
        colors = plt.cm.tab10.colors
        for i, name in enumerate(sub_names):
            vals = columns[name]
            ax.plot(times[:len(vals)], vals, color=colors[i % len(colors)],
                    linewidth=0.8, label=f"{describe_column(name)} (avg={mean(vals):.3f})")
        ax.set_ylabel("Processing Time (ms)", fontsize=11)
        ax.set_title("Subprocess Computation Time", fontsize=11)
        ax.legend(loc="upper right", fontsize=11)
        ax.grid(alpha=0.3)
        ax.set_ylim(0, 60)

        # --- zoomed-in ---
        col_maxes = [max(columns[name]) for name in sub_names]
        zoom_ceiling = min(col_maxes) * 1.2 if col_maxes else 0
        if zoom_ceiling > 0:
            axins = inset_axes(ax, width="38%", height="38%", loc="upper left", borderpad=4.5)
            for i, name in enumerate(sub_names):
                vals = columns[name]
                axins.plot(times[:len(vals)], vals, color=colors[i % len(colors)], linewidth=0.6)
            axins.set_xlim(times[0], times[min(len(times), max(len(v) for v in columns.values())) - 1])
            axins.set_ylim(0, 0.1)
            axins.set_title("Zoom Near Zero", fontsize=11)
            axins.tick_params(labelsize=10)
            axins.grid(alpha=0.3)

    axes[-1].set_xlabel("Time (s)" if timestamps else "Sample index")

    figure_title = title if title else os.path.basename(path)
    fig.suptitle(figure_title, fontsize=16, fontweight="bold")
    fig.tight_layout(rect=[0, 0, 1, 0.97])

    os.makedirs(outdir, exist_ok=True)
    base = os.path.splitext(os.path.basename(path))[0]
    out_svg = os.path.join(outdir, f"{base}_timings.svg")
    fig.savefig(out_svg, format="svg")
    plt.close(fig)
    print(f"  Saved plot: {out_svg}")


def main():
    parser = argparse.ArgumentParser(description="Average and plo timing CSVs")
    parser.add_argument("files", nargs="*", help="Timing CSV file")
    parser.add_argument("--outdir", default="./plots", help="plot dir")
    parser.add_argument("--title", default=None,
                         help="Figure title")
    parser.add_argument("--no-plot", action="store_true", help="Skip plotting, only print stats")
    args = parser.parse_args()

    files = args.files
    if not files:
        files = sorted(glob.glob("*.csv"))
        if not files:
            print("No CSV files given and none found in current directory")
            print("Usage: python average_timings.py file1.csv [file2.csv ...]")
            sys.exit(1)

    all_total_ms_averages = []

    for path in files:
        if not os.path.isfile(path):
            print(f"Skipping (not found): {path}")
            continue
        columns = summarize_file(path)
        if not args.no_plot:
            plot_file(path, columns, args.outdir, args.title)
        if "total_ms" in columns:
            all_total_ms_averages.append((os.path.basename(path), mean(columns["total_ms"])))

    if len(all_total_ms_averages) > 1:
        print("\n=== Summary across all files (total_ms) ===")
        for name, avg in all_total_ms_averages:
            print(f"  {name}: {avg:.4f} ms")
        overall = mean(a for _, a in all_total_ms_averages)
        print(f"  Overall average across files: {overall:.4f} ms")


if __name__ == "__main__":
    main()