#!/usr/bin/env python3
"""
Analyze inlier score distributions across multiple datasets to recommend optimal threshold.

Usage:
    python3 analyze_inlier_threshold.py csv_file1.csv [csv_file2.csv ...] [--plot]
"""

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import argparse
import sys

def load_and_merge_data(csv_paths):
    """Load and merge multiple CSV files."""
    dfs = []
    for path in csv_paths:
        print(f"Loading: {path}")
        df = pd.read_csv(path)
        dfs.append(df)
    
    combined = pd.concat(dfs, ignore_index=True)
    print(f"Merged {len(csv_paths)} files: total {len(combined)} samples\n")
    return combined

def analyze_inlier_distribution(df):
    """Analyze inlier score and inlier ratio distributions."""
    stats = {}
    
    # Inlier score stats (this is the computed score, capped at 1.0)
    stats['pub_inlier_score_min'] = df['pub_inlier_score'].min()
    stats['pub_inlier_score_max'] = df['pub_inlier_score'].max()
    stats['pub_inlier_score_mean'] = df['pub_inlier_score'].mean()
    stats['pub_inlier_score_std'] = df['pub_inlier_score'].std()
    stats['pub_inlier_score_median'] = df['pub_inlier_score'].median()
    stats['pub_inlier_score_q25'] = df['pub_inlier_score'].quantile(0.25)
    stats['pub_inlier_score_q75'] = df['pub_inlier_score'].quantile(0.75)
    
    # Inlier ratio stats (raw ratio before capping)
    stats['inlier_ratio_min'] = df['inlier_ratio'].min()
    stats['inlier_ratio_max'] = df['inlier_ratio'].max()
    stats['inlier_ratio_mean'] = df['inlier_ratio'].mean()
    stats['inlier_ratio_std'] = df['inlier_ratio'].std()
    stats['inlier_ratio_median'] = df['inlier_ratio'].median()
    
    # Current inlier score (before visibility weighting)
    stats['curr_inlier_score_min'] = df['curr_inlier_score'].min()
    stats['curr_inlier_score_max'] = df['curr_inlier_score'].max()
    stats['curr_inlier_score_mean'] = df['curr_inlier_score'].mean()
    stats['curr_inlier_score_std'] = df['curr_inlier_score'].std()
    
    # Combined score stats
    stats['pub_combined_score_min'] = df['pub_combined_score'].min()
    stats['pub_combined_score_max'] = df['pub_combined_score'].max()
    stats['pub_combined_score_mean'] = df['pub_combined_score'].mean()
    stats['pub_combined_score_std'] = df['pub_combined_score'].std()
    stats['pub_combined_score_median'] = df['pub_combined_score'].median()
    
    # Count samples at different quality thresholds
    stats['samples_below_0.2'] = len(df[df['pub_inlier_score'] < 0.2])
    stats['samples_below_0.3'] = len(df[df['pub_inlier_score'] < 0.3])
    stats['samples_below_0.4'] = len(df[df['pub_inlier_score'] < 0.4])
    stats['samples_below_0.5'] = len(df[df['pub_inlier_score'] < 0.5])
    
    # Check correlation between inlier_ratio and combined_score
    stats['correlation_ratio_to_combined'] = df['inlier_ratio'].corr(df['pub_combined_score'])
    
    return stats

def recommend_threshold(df):
    """Recommend optimal inlier score threshold."""
    recommendations = {}
    
    # Strategy 1: Use mean - std as conservative threshold (keeps ~68%)
    mean_score = df['pub_inlier_score'].mean()
    std_score = df['pub_inlier_score'].std()
    recommendations['conservative'] = max(mean_score - std_score, 0.1)
    
    # Strategy 2: Use median + some margin (keeps upper half of quality)
    median_score = df['pub_inlier_score'].median()
    recommendations['median_based'] = median_score
    
    # Strategy 3: Use 25th percentile as threshold (rejects worst 25%)
    q25 = df['pub_inlier_score'].quantile(0.25)
    recommendations['reject_worst_25'] = q25
    
    # Strategy 4: Use combined score distribution to identify natural break points
    combined_mean = df['pub_combined_score'].mean()
    combined_std = df['pub_combined_score'].std()
    recommendations['combined_score_based'] = combined_mean - 0.5 * combined_std  # reject poor combined scores
    
    return recommendations

def print_detailed_report(df, stats, recommendations):
    """Print comprehensive analysis report."""
    print("="*70)
    print("INLIER SCORE DISTRIBUTION ANALYSIS")
    print("="*70)
    
    print(f"\nDATASET SUMMARY:")
    print(f"  Total Samples: {len(df)}")
    print(f"  Average Inlier Count: {df['inlier_count'].mean():.1f} ± {df['inlier_count'].std():.1f}")
    print(f"  Average Subcloud Size: {df['subcloud_size'].mean():.1f} ± {df['subcloud_size'].std():.1f}")
    
    print(f"\nPUBLISHED INLIER SCORE DISTRIBUTION:")
    print(f"  Min:    {stats['pub_inlier_score_min']:.3f}")
    print(f"  Q25:    {stats['pub_inlier_score_q25']:.3f}  [25th percentile]")
    print(f"  Median: {stats['pub_inlier_score_median']:.3f}")
    print(f"  Mean:   {stats['pub_inlier_score_mean']:.3f}")
    print(f"  Q75:    {stats['pub_inlier_score_q75']:.3f}  [75th percentile]")
    print(f"  Max:    {stats['pub_inlier_score_max']:.3f}")
    print(f"  StdDev: {stats['pub_inlier_score_std']:.3f}")
    
    print(f"\nRAW INLIER RATIO DISTRIBUTION:")
    print(f"  Min:    {stats['inlier_ratio_min']:.3f}")
    print(f"  Median: {stats['inlier_ratio_median']:.3f}")
    print(f"  Mean:   {stats['inlier_ratio_mean']:.3f}")
    print(f"  Max:    {stats['inlier_ratio_max']:.3f}")
    print(f"  StdDev: {stats['inlier_ratio_std']:.3f}")
    
    print(f"\nCOMBINED SCORE DISTRIBUTION:")
    print(f"  Min:    {stats['pub_combined_score_min']:.3f}")
    print(f"  Mean:   {stats['pub_combined_score_mean']:.3f}")
    print(f"  Median: {stats['pub_combined_score_median']:.3f}")
    print(f"  Max:    {stats['pub_combined_score_max']:.3f}")
    print(f"  StdDev: {stats['pub_combined_score_std']:.3f}")
    
    print(f"\nQUALITY REJECTION RATES (if threshold applied):")
    total = len(df)
    print(f"  Score < 0.2: {stats['samples_below_0.2']:4d} samples ({stats['samples_below_0.2']/total*100:5.1f}%) — Very poor inlier ratio")
    print(f"  Score < 0.3: {stats['samples_below_0.3']:4d} samples ({stats['samples_below_0.3']/total*100:5.1f}%) — Poor inlier ratio")
    print(f"  Score < 0.4: {stats['samples_below_0.4']:4d} samples ({stats['samples_below_0.4']/total*100:5.1f}%) — Below-average inlier ratio")
    print(f"  Score < 0.5: {stats['samples_below_0.5']:4d} samples ({stats['samples_below_0.5']/total*100:5.1f}%) — Below median inlier ratio")
    
    print(f"\nCORRELATION ANALYSIS:")
    print(f"  Inlier Ratio ↔ Combined Score: {stats['correlation_ratio_to_combined']:.3f}")
    print(f"  (Higher correlation = inlier ratio is a strong predictor of overall quality)")
    
    print(f"\n{'='*70}")
    print("RECOMMENDED THRESHOLDS FOR INLIER SCORE:")
    print(f"{'='*70}")
    
    print(f"\n1. CONSERVATIVE (accept ~68% of samples):")
    print(f"   Threshold: {recommendations['conservative']:.3f}")
    print(f"   Rationale: Mean - StdDev; rejects outliers while keeping bulk of data")
    
    print(f"\n2. MEDIAN-BASED (accept better half):")
    print(f"   Threshold: {recommendations['median_based']:.3f}")
    print(f"   Rationale: Splits dataset in half by quality; simple 50/50 split")
    
    print(f"\n3. REJECT WORST 25% (keep upper 75%):")
    print(f"   Threshold: {recommendations['reject_worst_25']:.3f}")
    print(f"   Rationale: Simple quartile-based rejection of poorest samples")
    
    print(f"\n4. COMBINED SCORE BASED:")
    print(f"   Threshold: {recommendations['combined_score_based']:.3f}")
    print(f"   Rationale: Accounts for visibility; threshold based on overall quality score")
    
    print(f"\n{'='*70}")
    print("RECOMMENDATION:")
    print(f"{'='*70}")
    
    # Choose a recommended threshold
    recommended = recommendations['median_based']
    
    print(f"\nBased on analysis, recommend starting with: {recommended:.3f}")
    print(f"\nThis threshold:")
    print(f"  • Rejects ~{(len(df[df['pub_inlier_score'] < recommended])/len(df)*100):.1f}% of samples (lower quality)")
    print(f"  • Keeps  ~{(len(df[df['pub_inlier_score'] >= recommended])/len(df)*100):.1f}% of samples (higher quality)")
    print(f"  • Is well-motivated by data distribution (median)")
    print(f"\nAlternatively, if you want more aggressive filtering:")
    print(f"  • Use {recommendations['reject_worst_25']:.3f} to reject worst 25%")
    print(f"\nOr if you want more conservative filtering:")
    print(f"  • Use {recommendations['conservative']:.3f} to keep most samples")
    
    print(f"\n" + "="*70 + "\n")

def plot_distributions(df, output_prefix='inlier_analysis'):
    """Plot inlier score distributions."""
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    
    # Plot 1: Histogram of inlier scores
    ax = axes[0, 0]
    ax.hist(df['pub_inlier_score'], bins=50, color='steelblue', alpha=0.7, edgecolor='black')
    ax.axvline(df['pub_inlier_score'].mean(), color='r', linestyle='--', linewidth=2, label=f"Mean: {df['pub_inlier_score'].mean():.3f}")
    ax.axvline(df['pub_inlier_score'].median(), color='g', linestyle='--', linewidth=2, label=f"Median: {df['pub_inlier_score'].median():.3f}")
    ax.set_xlabel('Published Inlier Score')
    ax.set_ylabel('Frequency')
    ax.set_title('Distribution of Inlier Scores')
    ax.legend()
    ax.grid(True, alpha=0.3)
    
    # Plot 2: Histogram of inlier ratios
    ax = axes[0, 1]
    ax.hist(df['inlier_ratio'], bins=50, color='coral', alpha=0.7, edgecolor='black')
    ax.axvline(df['inlier_ratio'].mean(), color='r', linestyle='--', linewidth=2, label=f"Mean: {df['inlier_ratio'].mean():.3f}")
    ax.set_xlabel('Inlier Ratio (raw)')
    ax.set_ylabel('Frequency')
    ax.set_title('Distribution of Inlier Ratios')
    ax.legend()
    ax.grid(True, alpha=0.3)
    
    # Plot 3: Scatter: inlier ratio vs combined score
    ax = axes[1, 0]
    scatter = ax.scatter(df['inlier_ratio'], df['pub_combined_score'], alpha=0.5, s=10)
    ax.set_xlabel('Inlier Ratio')
    ax.set_ylabel('Combined Score')
    ax.set_title('Inlier Ratio vs Combined Score')
    ax.grid(True, alpha=0.3)
    
    # Plot 4: Box plot comparison
    ax = axes[1, 1]
    data_to_plot = [
        df['pub_inlier_score'].values,
        df['pub_combined_score'].values,
        df['pub_vis_score'].values,
    ]
    bp = ax.boxplot(data_to_plot, labels=['Inlier Score', 'Combined Score', 'Visibility Score'])
    ax.set_ylabel('Score Value')
    ax.set_title('Score Component Comparison')
    ax.grid(True, alpha=0.3, axis='y')
    
    plt.tight_layout()
    plt.savefig(f'{output_prefix}_distributions.png', dpi=150)
    print(f"Saved distribution plot to {output_prefix}_distributions.png\n")
    plt.close()

def main():
    parser = argparse.ArgumentParser(
        description='Analyze inlier score distributions to recommend optimal threshold',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        allow_abbrev=False
    )
    parser.add_argument('csv_files', nargs='+', help='CSV log files to analyze')
    parser.add_argument('--plot', action='store_true', help='Generate distribution plots')
    
    args = parser.parse_args()
    
    if not args.csv_files:
        print("Error: No CSV files specified")
        sys.exit(1)
    
    df = load_and_merge_data(args.csv_files)
    stats = analyze_inlier_distribution(df)
    recommendations = recommend_threshold(df)
    
    print_detailed_report(df, stats, recommendations)
    
    if args.plot:
        output_prefix = 'inlier_analysis'
        plot_distributions(df, output_prefix)
    
    print("Analysis complete!")

if __name__ == "__main__":
    main()
