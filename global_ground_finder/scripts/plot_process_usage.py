#!/usr/bin/env python3

import argparse
import csv
import glob
import os
import sys
from collections import defaultdict
from statistics import mean

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


def load_rows(path):
    rows = []
    with open(path, newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            rows.append({
                "timestamp": float(row["timestamp"]),
                "pid": row["pid"],
                "cpu_percent": float(row["cpu_percent"]),
                "ram_rss_mb": float(row["ram_rss_mb"]),
                "ram_percent": float(row["ram_percent"]),
            })
    return rows


def find_dominant_pid(rows):
    """Pick the PID with the highest average CPU usage."""
    by_pid = defaultdict(list)
    for r in rows:
        by_pid[r["pid"]].append(r["cpu_percent"])

    avg_by_pid = {pid: mean(vals) for pid, vals in by_pid.items()}
    dominant_pid = max(avg_by_pid, key=avg_by_pid.get)
    return dominant_pid, avg_by_pid


def summarize(rows, total_ram_gb=None):
    cpu = [r["cpu_percent"] for r in rows]
    rss = [r["ram_rss_mb"] for r in rows]
    ram_pct = [r["ram_percent"] for r in rows]

    summary = {
        "n_samples": len(rows),
        "cpu_avg": mean(cpu),
        "cpu_max": max(cpu),
        "ram_rss_avg_mb": mean(rss),
        "ram_rss_max_mb": max(rss),
        "ram_percent_avg": mean(ram_pct),
        "ram_percent_max": max(ram_pct),
    }

    if total_ram_gb:
        total_mb = total_ram_gb * 1024
        summary["ram_rss_avg_pct_of_total"] = 100 * summary["ram_rss_avg_mb"] / total_mb
        summary["ram_rss_max_pct_of_total"] = 100 * summary["ram_rss_max_mb"] / total_mb

    return summary


def plot_usage(rows, pid, out_svg, summary, figure_title, total_ram_gb=None):
    t0 = rows[0]["timestamp"]
    times = [r["timestamp"] - t0 for r in rows]
    cpu = [r["cpu_percent"] for r in rows]
    rss = [r["ram_rss_mb"] for r in rows]
    ram_pct = [r["ram_percent"] for r in rows]

    fig, axes = plt.subplots(3, 1, figsize=(12, 9), sharex=True)

    ax = axes[0]
    ax.plot(times, cpu, color="tab:blue", linewidth=1)
    ax.axhline(summary["cpu_avg"], color="tab:blue", linestyle="--", linewidth=1,
               label=f"avg={summary['cpu_avg']:.1f}%")
    ax.axhline(summary["cpu_max"], color="tab:red", linestyle="-.", linewidth=1,
               label=f"max={summary['cpu_max']:.1f}%")
    ax.set_ylabel("CPU (%)", fontsize=11)
    ax.set_title(f"CPU usage")
    ax.legend(loc="upper right", fontsize=11)
    ax.grid(alpha=0.3)
    ax.set_ylim(0, 750)

    ax = axes[1]
    ax.plot(times, rss, color="tab:green", linewidth=1)
    ax.axhline(summary["ram_rss_avg_mb"], color="tab:green", linestyle="--", linewidth=1.2,
               label=f"avg={summary['ram_rss_avg_mb']:.1f} MB")
    ax.axhline(summary["ram_rss_max_mb"], color="tab:red", linestyle="-.", linewidth=1.2,
               label=f"max={summary['ram_rss_max_mb']:.1f} MB")
    ax.set_ylabel("RAM (MB)")
    ax.set_title("Memory Usage (MB)", fontsize = 11)
    ax.legend(loc="upper right", fontsize=11)
    ax.grid(alpha=0.3)
    ax.set_ylim(0, 800)

    ax = axes[2]
    ax.plot(times, ram_pct, color="tab:purple", linewidth=1)
    ax.axhline(summary["ram_percent_avg"], color="tab:purple", linestyle="--", linewidth=1.2,
               label=f"avg={summary['ram_percent_avg']:.2f}%")
    ax.axhline(summary["ram_percent_max"], color="tab:red", linestyle="-.", linewidth=1.2,
               label=f"max={summary['ram_percent_max']:.2f}%")
    title = "Memory Usage (% of total 32GB RAM)"
    ax.set_ylabel("RAM (%)", fontsize=11)
    ax.set_title(title)
    ax.legend(loc="upper right", fontsize=11)
    ax.grid(alpha=0.3)
    ax.set_ylim(0, 2.5)

    ax.set_xlim(0, 48)
    axes[-1].set_xlabel("Time (s)")

    fig.suptitle(figure_title, fontsize=11, fontweight="bold")
    fig.tight_layout(rect=[0, 0, 1, 0.97])
    fig.savefig(out_svg, format="svg")
    plt.close(fig)


def process_file(path, outdir, total_ram_gb, title):
    rows = load_rows(path)
    if not rows:
        print(f"Skipping {path}: no rows found.")
        return

    dominant_pid, avg_by_pid = find_dominant_pid(rows)
    dominant_rows = [r for r in rows if r["pid"] == dominant_pid]

    base = os.path.splitext(os.path.basename(path))[0]
    print(f"\n=== {os.path.basename(path)} ===")
    print("PIDs found (avg CPU%):")
    for pid, avg_cpu in sorted(avg_by_pid.items(), key=lambda kv: -kv[1]):
        marker = "  <-- kept (dominant)" if pid == dominant_pid else "  (filtered out)"
        print(f"  pid {pid}: avg_cpu={avg_cpu:.2f}%{marker}")

    summary = summarize(dominant_rows, total_ram_gb=total_ram_gb)

    print(f"\nStats for dominant PID {dominant_pid} (n={summary['n_samples']} samples):")
    print(f"  CPU:      avg={summary['cpu_avg']:.2f}%   max={summary['cpu_max']:.2f}%")
    print(f"  RAM RSS:  avg={summary['ram_rss_avg_mb']:.2f} MB   max={summary['ram_rss_max_mb']:.2f} MB")
    print(f"  RAM %:    avg={summary['ram_percent_avg']:.3f}%   max={summary['ram_percent_max']:.3f}%")
    if total_ram_gb:
        print(f"  (of {total_ram_gb} GB total: "
              f"avg={summary['ram_rss_avg_pct_of_total']:.3f}%, "
              f"max={summary['ram_rss_max_pct_of_total']:.3f}%)")

    os.makedirs(outdir, exist_ok=True)
    out_svg = os.path.join(outdir, f"{base}_pid{dominant_pid}_usage.svg")
    figure_title = title if title else os.path.basename(path)
    plot_usage(dominant_rows, dominant_pid, out_svg, summary,
               figure_title=figure_title, total_ram_gb=total_ram_gb)
    print(f"  Saved plot: {out_svg}")

    return summary


def main():
    parser = argparse.ArgumentParser(description="Plot & summarize dominant-process CPU/RAM usage.")
    parser.add_argument("files", nargs="*", help="Process usage CSV file(s).")
    parser.add_argument("--outdir", default="./plots", help="Directory to save plots to.")
    parser.add_argument("--total-ram-gb", type=float, default=32,
                         help="Total system RAM in GB, used to annotate RAM%% (default: 32).")
    parser.add_argument("--title", default=None,
                         help="Figure title. Defaults to the source CSV filename if omitted.")
    args = parser.parse_args()

    files = args.files or sorted(glob.glob("*.csv"))
    if not files:
        print("No CSV files given and none found in current directory.")
        sys.exit(1)

    for path in files:
        if not os.path.isfile(path):
            print(f"Skipping (not found): {path}")
            continue
        process_file(path, args.outdir, args.total_ram_gb, args.title)


if __name__ == "__main__":
    main()