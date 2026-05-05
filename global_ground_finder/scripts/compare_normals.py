#!/usr/bin/env python3
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import argparse
from pathlib import Path


def load_computed_normals(csv_path):
    df = pd.read_csv(csv_path, dtype={'timestamp': np.float64})
    
    required_cols = ['timestamp', 'nx', 'ny', 'nz']
    missing = [col for col in required_cols if col not in df.columns]
    if missing:
        raise ValueError(f"Computed normals CSV missing columns: {missing}")
    
    print(f"Loaded {len(df)} computed normal vectors from {csv_path}")
    print(f"  Timestamp range: {df['timestamp'].min():.9f} to {df['timestamp'].max():.9f}")
    return df


def load_ground_truth(csv_path):
    df = pd.read_csv(csv_path, dtype={'timestamp': np.float64})
    
    required_cols = ['timestamp', 'GT_nx', 'GT_ny', 'GT_nz']
    missing = [col for col in required_cols if col not in df.columns]
    if missing:
        raise ValueError(f"Ground truth CSV missing columns: {missing}")
    
    print(f"Loaded {len(df)} ground truth normal vectors from {csv_path}")
    print(f"  Timestamp range: {df['timestamp'].min():.9f} to {df['timestamp'].max():.9f}")
    return df


def match_timestamps(computed_df, gt_df, max_time_diff=0.05):
    """
    Match computed normals to ground truth by timestamp.
    Uses nearest neighbor matching within max_time_diff tolerance.
    
    Args:
        computed_df: DataFrame with computed normals
        gt_df: DataFrame with ground truth normals
        max_time_diff: Maximum time difference (seconds) for matching
    
    Returns:
        DataFrame with matched pairs
    """
    matched_data = []
    
    for idx, comp_row in computed_df.iterrows():
        comp_time = comp_row['timestamp']
        
        # Find nearest GT timestamp
        time_diffs = np.abs(gt_df['timestamp'] - comp_time)
        min_diff_idx = time_diffs.idxmin()
        min_diff = time_diffs[min_diff_idx]
        
        if min_diff <= max_time_diff:
            gt_row = gt_df.loc[min_diff_idx]
            
            matched_data.append({
                'timestamp': comp_time,
                'nx': comp_row['nx'],
                'ny': comp_row['ny'],
                'nz': comp_row['nz'],
                'GT_nx': gt_row['GT_nx'],
                'GT_ny': gt_row['GT_ny'],
                'GT_nz': gt_row['GT_nz'],
                'time_diff': min_diff
            })
    
    matched_df = pd.DataFrame(matched_data)
    print(f"Matched {len(matched_df)} pairs (out of {len(computed_df)} computed normals)")
    
    if len(matched_df) == 0:
        print("WARNING: No matches found! Check that timestamps are in the same format/range.")
    else:
        print(f"  Time difference range: {matched_df['time_diff'].min():.9f}s to {matched_df['time_diff'].max():.9f}s")
        print(f"  Mean time difference: {matched_df['time_diff'].mean():.9f}s")
    
    return matched_df


def calculate_angular_error(nx, ny, nz, gt_nx, gt_ny, gt_nz):
    """
    Calculate angular error in degrees between two normal vectors.
    
    Uses dot product: cos(theta) = n1 · n2 / (|n1| * |n2|)
    Since normals should be unit vectors, this simplifies to: cos(theta) = n1 · n2
    """
    dot_product = nx * gt_nx + ny * gt_ny + nz * gt_nz
    
    dot_product = np.clip(dot_product, -1.0, 1.0)
    
    # angular error in degrees
    angle_rad = np.arccos(np.abs(dot_product))
    angle_deg = np.degrees(angle_rad)
    
    return angle_deg


def compute_errors(matched_df):
    """Compute angular errors for all matched pairs."""
    matched_df['angular_error_deg'] = calculate_angular_error(
        matched_df['nx'].values,
        matched_df['ny'].values,
        matched_df['nz'].values,
        matched_df['GT_nx'].values,
        matched_df['GT_ny'].values,
        matched_df['GT_nz'].values
    )
    
    return matched_df


def print_statistics(matched_df):
    errors = matched_df['angular_error_deg']
    
    print("\n" + "="*60)
    print("NORMAL VECTOR ERROR STATISTICS")
    print("="*60)
    print(f"Number of samples:     {len(errors)}")
    print(f"Mean error:            {errors.mean():.3f}°")
    print(f"Median error:          {errors.median():.3f}°")
    print(f"Std deviation:         {errors.std():.3f}°")
    print(f"Min error:             {errors.min():.3f}°")
    print(f"Max error:             {errors.max():.3f}°")
    print(f"95th percentile:       {errors.quantile(0.95):.3f}°")
    print(f"99th percentile:       {errors.quantile(0.99):.3f}°")
    print("="*60 + "\n")


def plot_results(matched_df, output_path=None):
    fig, axes = plt.subplots(2, 1, figsize=(12, 8))
    
    # Offset timestamps
    t_start = matched_df['timestamp'].min()
    time_rel = matched_df['timestamp'] - t_start
    errors = matched_df['angular_error_deg']
    
    # Error over time
    ax1 = axes[0]
    ax1.plot(time_rel, errors, linewidth=0.75, alpha=0.7)
    ax1.scatter(time_rel, errors, s=1, alpha=0.5)
    ax1.axhline(y=errors.mean(), color='r', linestyle='--', 
                label=f'Mean: {errors.mean():.2f}°', linewidth=1.5)
    ax1.axhline(y=errors.median(), color='g', linestyle='--', 
                label=f'Median: {errors.median():.2f}°', linewidth=1.5)
    ax1.set_xlabel('Time (s)', fontsize=11)
    ax1.set_ylabel('Angular Error (°)', fontsize=11)
    ax1.set_title('Normal Vector Angular Error vs Ground Truth', fontsize=12, fontweight='bold')
    ax1.grid(True, alpha=0.3)
    ax1.legend(loc='upper right')
    
    # Error histogram
    ax2 = axes[1]
    ax2.hist(errors, bins=50, alpha=0.7, edgecolor='black', linewidth=0.5)
    ax2.axvline(x=errors.mean(), color='r', linestyle='--', 
                label=f'Mean: {errors.mean():.2f}°', linewidth=2)
    ax2.axvline(x=errors.median(), color='g', linestyle='--', 
                label=f'Median: {errors.median():.2f}°', linewidth=2)
    ax2.set_xlabel('Angular Error (°)', fontsize=11)
    ax2.set_ylabel('Frequency', fontsize=11)
    ax2.set_title('Angular Error Distribution', fontsize=12, fontweight='bold')
    ax2.grid(True, alpha=0.3, axis='y')
    ax2.legend(loc='upper right')
    
    plt.tight_layout()
    
    if output_path:
        plt.savefig(output_path, dpi=150, bbox_inches='tight')
        print(f"Plot saved to {output_path}")
    
    plt.show()


def main():
    parser = argparse.ArgumentParser(
        description='Compare computed normal vectors against ground truth',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python3 compare_normals_to_gt.py computed.csv ground_truth.csv
  
  python3 compare_normals_to_gt.py computed.csv gt.csv --max-time-diff 0.1
  
  python3 compare_normals_to_gt.py computed.csv gt.csv --output results.csv --plot errors.png
        """
    )
    
    parser.add_argument('computed_csv', type=str,
                        help='Path to CSV with computed normals (must have: timestamp, nx, ny, nz)')
    parser.add_argument('ground_truth_csv', type=str,
                        help='Path to CSV with ground truth normals (must have: timestamp, GT_nx, GT_ny, GT_nz)')
    parser.add_argument('--max-time-diff', type=float, default=0.05,
                        help='Maximum time difference (seconds) for timestamp matching (default: 0.05)')
    parser.add_argument('--output', '-o', type=str, default=None,
                        help='Save matched pairs with errors to CSV')
    parser.add_argument('--plot', '-p', type=str, default=None,
                        help='Save plot')
    parser.add_argument('--no-plot', action='store_true',
                        help='Skip plot')
    
    args = parser.parse_args()
    
    print("Loading data...")
    computed_df = load_computed_normals(args.computed_csv)
    gt_df = load_ground_truth(args.ground_truth_csv)
    
    print(f"\nMatching timestamps (max time diff: {args.max_time_diff}s)...")
    matched_df = match_timestamps(computed_df, gt_df, args.max_time_diff)
    
    if len(matched_df) == 0:
        print("ERROR: No matching timestamps found. Exiting.")
        return
    
    print("Computing angular errors...")
    matched_df = compute_errors(matched_df)
    
    print_statistics(matched_df)
    
    if args.output:
        matched_df.to_csv(args.output, index=False)
        print(f"Results saved to {args.output}")
    
    if not args.no_plot:
        plot_results(matched_df, args.plot)


if __name__ == '__main__':
    main()
