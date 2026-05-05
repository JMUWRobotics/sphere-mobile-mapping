#!/usr/bin/env python3
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

# Read the kernel data
df = pd.read_csv('/tmp/gaussian_kernel.csv')

# Create the plot
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8))

# Plot 1: Weight vs Time (convert to numpy)
ax1.plot(df['time_s'].values, df['weight'].values, 'b-', linewidth=2, marker='o', markersize=4)
ax1.set_xlabel('Time (seconds)', fontsize=12)
ax1.set_ylabel('Weight', fontsize=12)
ax1.set_title('Gaussian Kernel Weights vs Time', fontsize=14, fontweight='bold')
ax1.grid(True, alpha=0.3)
ax1.axhline(y=0, color='k', linestyle='-', linewidth=0.5)
ax1.axvline(x=0, color='r', linestyle='--', linewidth=1, alpha=0.5, label='Current sample (t=0)')
ax1.legend()

# Plot 2: Weight vs Index (convert to numpy)
ax2.plot(df['index'].values, df['weight'].values, 'g-', linewidth=2, marker='s', markersize=4)
ax2.set_xlabel('Buffer Index', fontsize=12)
ax2.set_ylabel('Weight', fontsize=12)
ax2.set_title('Gaussian Kernel Weights vs Buffer Index', fontsize=14, fontweight='bold')
ax2.grid(True, alpha=0.3)
ax2.axvline(x=0, color='r', linestyle='--', linewidth=1, alpha=0.5, label='Newest sample')
ax2.legend()

plt.tight_layout()

# Print statistics
print("=" * 60)
print("Gaussian Kernel Statistics:")
print("=" * 60)
print(f"Window size: {len(df)}")
print(f"Time span: {df['time_s'].min():.4f} to {df['time_s'].max():.4f} seconds")
print(f"Max weight: {df['weight'].max():.6f} at t={df.loc[df['weight'].idxmax(), 'time_s']:.4f}s")
print(f"Sum of weights: {df['weight'].sum():.6f} (should be ≈1.0 for normalized kernel)")
print(f"Weight at newest sample (t=0): {df.loc[0, 'weight']:.6f}")
print(f"Weight at oldest sample: {df.loc[len(df)-1, 'weight']:.6f}")
print("=" * 60)

# Check if it looks like a proper Gaussian
if df.loc[0, 'weight'] == df['weight'].max():
    print("✓ Peak at t=0 (newest sample) - CORRECT for causal filter")
else:
    print("✗ Peak NOT at t=0 - potential issue!")

if abs(df['weight'].sum() - 1.0) < 0.01:
    print("✓ Weights sum to ≈1.0 - properly normalized")
else:
    print(f"✗ Weights sum to {df['weight'].sum():.6f} - normalization issue!")

plt.savefig('/tmp/gaussian_kernel_plot.png', dpi=150, bbox_inches='tight')
print("\nPlot saved to: /tmp/gaussian_kernel_plot.png")
plt.show()
