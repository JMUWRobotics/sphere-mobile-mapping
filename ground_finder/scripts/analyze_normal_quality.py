#!/usr/bin/env python3
"""
Analyze the quality and stability of ground normal vector estimates.
Compares performance with and without fallback mechanism.

Usage:
    python3 analyze_normal_quality.py /path/to/scored_normals.csv [-s START] [-e END] [--minimal]
"""

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import argparse

def load_data(csv_path):
    """Load scored normals CSV data."""
    return pd.read_csv(csv_path)

def compute_statistics(df):
    """Compute key quality metrics."""
    stats = {}
    
    stats['total_samples'] = len(df)
    stats['fallback_count'] = df['using_fallback'].sum()
    stats['fallback_rate'] = stats['fallback_count'] / stats['total_samples'] * 100
    
    if 'fallback_unavailable' in df.columns:
        stats['fallback_unavailable_count'] = df['fallback_unavailable'].sum()
        stats['fallback_unavailable_rate'] = stats['fallback_unavailable_count'] / stats['total_samples'] * 100
    else:
        stats['fallback_unavailable_count'] = 0
        stats['fallback_unavailable_rate'] = 0.0
    
    # Normal operation: neither fallback used nor unavailable
    normal_operation = df[(df['using_fallback'] == 0) & (df.get('fallback_unavailable', 0) == 0)]
    stats['normal_operation_count'] = len(normal_operation)
    stats['normal_operation_rate'] = stats['normal_operation_count'] / stats['total_samples'] * 100
    
    stats['mean_combined_score'] = df['pub_combined_score'].mean()
    stats['mean_visibility_score'] = df['pub_vis_score'].mean()
    stats['mean_inlier_score'] = df['pub_inlier_score'].mean()
    
    # When fallback is used vs not used
    fallback_data = df[df['using_fallback'] == 1]
    no_fallback_data = df[df['using_fallback'] == 0]
    
    if 'fallback_unavailable' in df.columns:
        fallback_unavailable_data = df[df['fallback_unavailable'] == 1]
    else:
        fallback_unavailable_data = pd.DataFrame()
    
    if len(fallback_data) > 0:
        stats['fallback_mean_curr_score'] = fallback_data['curr_combined_score'].mean()
    else:
        stats['fallback_mean_curr_score'] = 0.0
    
    if len(no_fallback_data) > 0:
        stats['no_fallback_mean_curr_score'] = no_fallback_data['curr_combined_score'].mean()
    else:
        stats['no_fallback_mean_curr_score'] = 0.0
    
    if len(fallback_unavailable_data) > 0:
        stats['fallback_unavailable_mean_curr_score'] = fallback_unavailable_data['curr_combined_score'].mean()
    else:
        stats['fallback_unavailable_mean_curr_score'] = 0.0
    
    return stats

def plot_analysis(df, output_prefix='analysis', minimal=False):
    """Generate analysis plots."""
    has_fallback_unavail = 'fallback_unavailable' in df.columns
    
    # Normalize timestamps to start from 0
    time_rel = (df['timestamp'] - df['timestamp'].iloc[0]).values
    
    if minimal:
        fig, ax = plt.subplots(1, 1, figsize=(14, 5))
        axes = [ax]
    else:
        fig, axes = plt.subplots(3, 1, figsize=(14, 10))
    
    # Plot 1: Score comparison over time
    ax = axes[0]
    ax.plot(time_rel, df['curr_combined_score'].values, 'g-', linewidth = 0.75, alpha=0.6, label='Current Score')
    ax.plot(time_rel, df['pub_combined_score'].values, 'b-', linewidth = 0.75, alpha=0.6, label='Published Score (with fallback)')
    fallback_times = df[df['using_fallback'] == 1]
    fallback_time_rel = (fallback_times['timestamp'] - df['timestamp'].iloc[0]).values
    ax.scatter(fallback_time_rel, fallback_times['pub_combined_score'].values, 
               c='red', s=1, label='Fallback Used', zorder=5)
    if has_fallback_unavail:
        unavail_times = df[df['fallback_unavailable'] == 1]
        unavail_time_rel = (unavail_times['timestamp'] - df['timestamp'].iloc[0]).values
        ax.scatter(unavail_time_rel, unavail_times['curr_combined_score'].values,
                   c='orange', s=1, label='Fallback Unavailable', zorder=5)
    ax.axhline(y=df['pub_combined_score'].mean(), color='b', linestyle='--', alpha=0.5, label='Mean Published')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Combined Score')
    ax.set_title('Score Comparison: Current vs Published')
    ax.legend()
    ax.grid(True, alpha=0.3)
    
    if minimal:
        plt.tight_layout()
        plt.savefig(f'{output_prefix}_plot.png', dpi=150)
        print(f"Saved plot to {output_prefix}_plot.png")
        plt.show()
        return
    
    # Plot 2: Inlier ratio and visibility score
    ax = axes[1]
    ax2 = ax.twinx()
    ax.plot(time_rel, df['inlier_ratio'].values, 'purple', linewidth = 0.75, alpha=0.6, label='Inlier Ratio')
    ax2.plot(time_rel, df['pub_vis_score'].values, 'orange', linewidth = 0.75, alpha=0.6, label='Visibility Score')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Inlier Ratio', color='purple')
    ax2.set_ylabel('Visibility Score', color='orange')
    ax.set_title('Inlier Ratio vs Visibility Score')
    ax.tick_params(axis='y', labelcolor='purple')
    ax2.tick_params(axis='y', labelcolor='orange')
    ax.grid(True, alpha=0.3)
    
    # Plot 3: Score breakdown
    ax = axes[2]
    ax.plot(time_rel, df['curr_vis_score'].values, linewidth = 0.75, alpha=0.6, label='Visibility Score')
    ax.plot(time_rel, df['curr_inlier_score'].values, linewidth = 0.75, alpha=0.6, label='Inlier Score')
    ax.plot(time_rel, df['curr_combined_score'].values, 'k-', linewidth = 0.75, alpha=0.8, label='Combined Score')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Score')
    ax.set_title('Score Components Over Time')
    ax.legend()
    ax.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig(f'{output_prefix}_plots.png', dpi=150)
    print(f"Saved plots to {output_prefix}_plots.png")
    plt.show()

def print_report(stats):
    """Print formatted analysis report."""
    print("\n" + "="*60)
    print("GROUND NORMAL QUALITY ANALYSIS REPORT")
    print("="*60)
    
    print("\nSTATISTICS:")
    print(f"  Total Samples:        {stats['total_samples']}")
    print(f"  Fallback Used:        {stats['fallback_count']} times ({stats['fallback_rate']:.1f}%)")
    if stats['fallback_unavailable_count'] > 0:
        print(f"  Fallback Unavailable: {stats['fallback_unavailable_count']} times ({stats['fallback_unavailable_rate']:.1f}%) (buffer was empty/low-quality)")
    print(f"  Normal Operation:     {stats['normal_operation_count']} times ({stats['normal_operation_rate']:.1f}%)")
    
    print("\nSCORE STATISTICS:")
    print(f"  Mean Combined Score:  {stats['mean_combined_score']:.3f}")
    print(f"  Mean Visibility:      {stats['mean_visibility_score']:.3f}")
    print(f"  Mean Inlier Score:    {stats['mean_inlier_score']:.3f}")
    
    print("\nFALLBACK IMPACT:")
    if stats['fallback_count'] > 0:
        print(f"  With Fallback (low current score):")
        print(f"    Mean Current Score:      {stats['fallback_mean_curr_score']:.3f}  [bad normals, replaced using sliding window]")
        print(f"  Without Fallback (score above threshold):")
        print(f"    Mean Current Score:      {stats['no_fallback_mean_curr_score']:.3f}  [good normals, used directly]")
        
        if stats['fallback_unavailable_count'] > 0:
            print(f"  Fallback Unavailable (needed but not available):")
            print(f"    Mean Current Score:      {stats['fallback_unavailable_mean_curr_score']:.3f}  [bad normals, no replacement available]")
    else:
        print("  No fallback events detected")
    
    print("\n" + "="*60)

def main():
    parser = argparse.ArgumentParser(
        description='Analyze ground normal vector quality with scoring and fallback mechanism',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        allow_abbrev=False
    )
    parser.add_argument('csv_path', help='Path to scored_normals.csv file')
    parser.add_argument('-s', '--start', type=float, default=None,
                        help='Start time in seconds')
    parser.add_argument('-e', '--end', type=float, default=None,
                        help='End time in seconds')
    parser.add_argument('--minimal', action='store_true',
                        help='Only plot first plot (no score breakdown)')
    
    args = parser.parse_args()
    
    print(f"Loading data from: {args.csv_path}")
    
    df = load_data(args.csv_path)
    print(f"Loaded {len(df)} samples")
    
    # Filter by time range if specified
    if args.start is not None or args.end is not None:
        time_rel = (df['timestamp'] - df['timestamp'].iloc[0]).values
        mask = np.ones(len(df), dtype=bool)
        
        if args.start is not None:
            mask &= (time_rel >= args.start)
        if args.end is not None:
            mask &= (time_rel <= args.end)
        
        df = df[mask].reset_index(drop=True)
        
        time_range_str = f"{args.start if args.start is not None else 0:.1f}s to {args.end if args.end is not None else time_rel[-1]:.1f}s"
        print(f"Filtered to time range: {time_range_str}")
        print(f"Samples after filtering: {len(df)}")
    
    if len(df) == 0:
        print("Error: No samples in specified time range")
        return
    
    stats = compute_statistics(df)
    
    print_report(stats)
    
    output_prefix = args.csv_path.replace('.csv', '')
    if args.start is not None or args.end is not None:
        output_prefix += f'_t{args.start if args.start else 0:.0f}-{args.end if args.end else "end"}'
    
    plot_analysis(df, output_prefix, minimal=args.minimal)
    
    print("\nAnalysis complete!")

if __name__ == "__main__":
    main()
