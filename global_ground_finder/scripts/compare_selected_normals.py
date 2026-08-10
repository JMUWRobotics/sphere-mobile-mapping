#!/usr/bin/env python3

import argparse
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

from compare_normals import (
    compute_errors,
    load_computed_normals,
    load_ground_truth,
    match_timestamps,
)


def summarize_errors(errors):
    return {
        "median": float(np.median(errors)),
        "rmse": float(np.sqrt(np.mean(errors ** 2))),
        "p95": float(np.quantile(errors, 0.95)),
        "max": float(np.max(errors)),
    }


def format_type_label(type_label):
    mapping = {
        "smoothed": "Smoothed",
        "raw": "Raw",
        "smoothed_raw": "Smoothed",
        "smoothed_scored": "Smoothed-Scored",
    }
    return mapping.get(type_label, type_label)


def detect_environment(paths):
    joined = " ".join(str(path).lower() for path in paths)
    if "tunnel" in joined:
        return "tunnel", "Tunnel Environment"
    if "rampe_hoch_runter" in joined or "rampe_runter_hoch" in joined:
        return "rampe_runter_hoch", "Ramp Environment"
    if "huegel" in joined:
        return "huegel", "Hill Environment"
    return "unknown", "Environment"


def build_shared_scales(matched_sets):
    all_errors = np.concatenate([df["angular_error_deg"].to_numpy() for _, _, df in matched_sets])
    all_times = np.concatenate(
        [((df["timestamp"] - df["timestamp"].min()).to_numpy()) for _, _, df in matched_sets]
    )

    error_max = float(np.max(all_errors)) if len(all_errors) else 1.0
    time_max = float(np.max(all_times)) if len(all_times) else 1.0

    bins = np.linspace(0.0, error_max if error_max > 0 else 1.0, 51)
    hist_counts = [np.histogram(df["angular_error_deg"].to_numpy(), bins=bins)[0] for _, _, df in matched_sets]
    hist_max = int(max((counts.max() for counts in hist_counts), default=1))

    return {
        "error_max": error_max,
        "time_max": time_max,
        "bins": bins,
        "hist_max": hist_max,
    }


def plot_dataset(matched_df, dataset_label, type_label, scales, output_dir, env_key, env_title):
    display_type_label = format_type_label(type_label)
    summary = summarize_errors(matched_df["angular_error_deg"].to_numpy())
    t_start = matched_df["timestamp"].min()
    time_rel = (matched_df["timestamp"] - t_start).to_numpy()
    errors = matched_df["angular_error_deg"].to_numpy()

    fig, axes = plt.subplots(2, 1, figsize=(12, 8))
    fig.suptitle(env_title, fontsize=16, fontweight="bold")
    fig.canvas.manager.set_window_title(f"{dataset_label} - {display_type_label}")

    ax1 = axes[0]
    ax1.plot(time_rel, errors, linewidth=0.75, alpha=0.7)
    ax1.scatter(time_rel, errors, s=1, alpha=0.5)
    ax1.axhline(
        y=summary["median"],
        color="r",
        linestyle="--",
        label=f'Median: {summary["median"]:.3f}°',
        linewidth=1.5,
    )
    ax1.axhline(
        y=summary["rmse"],
        color="g",
        linestyle="-.",
        label=f'RMSE: {summary["rmse"]:.3f}°',
        linewidth=1.5,
    )
    ax1.axhline(
        y=summary["p95"],
        color="b",
        linestyle=":",
        label=f'95th percentile: {summary["p95"]:.3f}°',
        linewidth=1.8,
    )
    ax1.set_xlabel("Time (s)", fontsize=11)
    ax1.set_ylabel("Angular Error (°)", fontsize=11)
    ax1.set_title(f"{dataset_label} Angular Error against Ground Truth", fontsize=12, fontweight="bold")
    ax1.set_xlim(0.0, scales["time_max"])
    ax1.set_ylim(0.0, max(scales["error_max"], summary["p95"]) * 1.05)
    ax1.grid(True, alpha=0.3)
    ax1.legend(loc="upper right")

    ax2 = axes[1]
    ax2.hist(errors, bins=scales["bins"], alpha=0.7, edgecolor="black", linewidth=0.5)
    ax2.axvline(
        x=summary["median"],
        color="r",
        linestyle="--",
        label=f'Median: {summary["median"]:.3f}°',
        linewidth=2,
    )
    ax2.axvline(
        x=summary["rmse"],
        color="g",
        linestyle="-.",
        label=f'RMSE: {summary["rmse"]:.3f}°',
        linewidth=2,
    )
    ax2.axvline(
        x=summary["p95"],
        color="b",
        linestyle=":",
        label=f'95th percentile: {summary["p95"]:.3f}°',
        linewidth=2,
    )
    ax2.set_xlabel("Angular Error (°)", fontsize=11)
    ax2.set_ylabel("Count", fontsize=11)
    ax2.set_title(f"{dataset_label} Angular Error Distribution", fontsize=12, fontweight="bold")
    ax2.set_xlim(0.0, scales["bins"][-1])
    ax2.set_ylim(0, max(scales["hist_max"], 1) * 1.05)
    ax2.grid(True, alpha=0.3, axis="y")
    ax2.legend(loc="upper right")

    plt.tight_layout(rect=[0, 0, 1, 0.95])

    stem = output_dir / f"{env_key}_plot_{dataset_label}_{type_label}"
    fig.savefig(stem.with_suffix(".svg"), bbox_inches="tight")
    print(f"Plot saved to {stem.with_suffix('.svg')}")


def parse_args():
    parser = argparse.ArgumentParser(
        description="Compare up to three selected computed-normal CSVs against one ground-truth CSV"
    )
    parser.add_argument("ground_truth_csv", type=str, help="Path to the ground-truth normals CSV")
    parser.add_argument(
        "computed_csv",
        nargs=3,
        type=str,
        help="Three paths to CSVs with computed normals (gf, static radius ggf, adapt radius ggf)",
    )
    parser.add_argument(
        "--labels",
        nargs="+",
        metavar="LABEL",
        help="Optional labels for the three computed CSVs; pass either 3 separate values or a single comma-separated string like 'a, b, c'",
    )
    parser.add_argument(
        "--max-time-diff",
        type=float,
        default=0.05,
        help="Max time difference in sec for timestamp matching",
    )
    parser.add_argument(
        "--output-dir",
        type=str,
        default=None,
        help="Directory for plots and statistics; defaults to ./comparison_selected_results",
    )
    parser.add_argument(
        "--no-show",
        action="store_true",
        help="Do not open plot windows after saving",
    )
    return parser.parse_args()


def main():
    args = parse_args()

    gt_df = load_ground_truth(args.ground_truth_csv)

    computed_paths = [Path(p) for p in args.computed_csv]
    if args.labels:
        if len(args.labels) == 1 and "," in args.labels[0]:
            labels = [item.strip() for item in args.labels[0].split(",") if item.strip()]
        else:
            labels = args.labels
    else:
        labels = [p.stem for p in computed_paths]

    if len(labels) != len(computed_paths):
        raise SystemExit(f"Expected {len(computed_paths)} labels but got {len(labels)}")

    output_dir = Path(args.output_dir).expanduser().resolve() if args.output_dir else Path(
        "comparison_selected_results"
    ).resolve()
    output_dir.mkdir(parents=True, exist_ok=True)

    env_key, env_title = detect_environment([args.ground_truth_csv, *computed_paths])

    matched_sets = []
    for label, computed_path in zip(labels, computed_paths):
        print(f"\nProcessing {label}: {computed_path}")
        computed_df = load_computed_normals(computed_path)
        available_types = [
            t
            for t in ["raw", "scored", "smoothed", "smoothed_raw", "smoothed_scored"]
            if t in computed_df["normal_type"].unique()
        ]

        if not available_types:
            raise SystemExit(f"No known normal_type values found in {computed_path}")

        for type_name in available_types:
            type_df = computed_df[computed_df["normal_type"] == type_name]
            matched_df = match_timestamps(type_df, gt_df, args.max_time_diff)

            if len(matched_df) == 0:
                print(f"WARNING: No timestamp matches found for {computed_path} [{type_name}]. Skipping.")
                continue

            matched_df = compute_errors(matched_df)
            matched_sets.append((label, type_name, matched_df))

    if not matched_sets:
        raise SystemExit("No matched results found for any selected dataset/type.")

    scales = build_shared_scales(matched_sets)

    print("\nShared plot scales:")
    print(f"  Max time range:   {scales['time_max']:.6f}s")
    print(f"  Max error range:  {scales['error_max']:.6f}°")
    print(f"  Histogram bins:    {len(scales['bins']) - 1}")

    for label, type_name, matched_df in matched_sets:
        plot_dataset(matched_df, label, type_name, scales, output_dir, env_key, env_title)

    if not args.no_show:
        plt.show()


if __name__ == "__main__":
    main()