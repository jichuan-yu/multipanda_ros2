#!/usr/bin/env python3
"""
Plot teleop_err_norm and safe_err_norm from relative_error_monitor CSV data.
"""

import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import argparse

def plot_relative_errors(csv_file, output_image=None, start_time=0):
    """Plot teleop_err_norm and safe_err_norm from CSV file.

    Args:
        csv_file: Path to the CSV file
        output_image: Output image path (optional)
        start_time: Start plotting from this time (seconds), default 0
    """

    # Read CSV file
    df = pd.read_csv(csv_file)

    # Convert timestamp to relative time (seconds from start)
    df['time'] = df['timestamp'] - df['timestamp'].iloc[0]

    # Filter data from start_time onwards
    if start_time > 0:
        df = df[df['time'] >= start_time].copy()

    # Extract the two error norm columns
    teleop_err = df['teleop_err_norm']
    safe_err = df['safe_err_norm']

    # Create figure
    fig, ax = plt.subplots(figsize=(12, 6))

    # Plot teleop error (skip NaN values)
    valid_teleop = ~teleop_err.isna()
    if valid_teleop.any():
        ax.plot(df['time'][valid_teleop], teleop_err[valid_teleop] * 1000,
                label='Teleop Error Norm', linewidth=1.5, alpha=0.9)

    # Plot safe error (skip NaN values)
    valid_safe = ~safe_err.isna()
    if valid_safe.any():
        ax.plot(df['time'][valid_safe], safe_err[valid_safe] * 1000,
                label='Safe Error Norm', linewidth=1.5, alpha=0.9)

    # Customize plot
    ax.set_xlabel('Time (s)', fontsize=12)
    ax.set_ylabel('Error Norm (mm)', fontsize=12)
    ax.set_title('Relative Position Error: Teleop vs Safe (from Initial Baseline)',
                 fontsize=14, fontweight='bold')
    ax.legend(fontsize=11)
    ax.grid(True, alpha=0.3, linestyle='--')

    # Set y-axis to start from 0
    ax.set_ylim(bottom=0)

    # Tight layout
    plt.tight_layout()

    # Save or show
    if output_image:
        plt.savefig(output_image, dpi=150, bbox_inches='tight')
        print(f"Plot saved to: {output_image}")
    else:
        plt.show()

def main():
    parser = argparse.ArgumentParser(
        description='Plot teleop_err_norm and safe_err_norm from relative_error CSV'
    )
    parser.add_argument('csv_file', help='Path to the CSV file')
    parser.add_argument('-o', '--output', help='Output image path (e.g., plot.png)')
    parser.add_argument('-s', '--show', action='store_true',
                        help='Show plot interactively')
    parser.add_argument('-t', '--start-time', type=float, default=0,
                        help='Start plotting from this time (seconds), default 0')

    args = parser.parse_args()

    plot_relative_errors(args.csv_file, args.output if not args.show else None, args.start_time)

if __name__ == '__main__':
    main()
