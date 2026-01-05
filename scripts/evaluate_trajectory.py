#!/usr/bin/env python3
"""
SLAM Trajectory Evaluation Script

Runs evo toolkit to compute ATE/RPE metrics and generate visualization.
Provides automated evaluation pipeline for OV2SLAM trajectories.

Requirements:
    pip install evo

Usage:
    python evaluate_trajectory.py --traj ov2slam.txt --gt gt.txt --output-dir eval_results
"""

import argparse
import subprocess
import os
import sys


def run_command(cmd, description):
    """
    Run shell command and print output.

    Args:
        cmd: Command string to execute
        description: Human-readable description of command

    Returns:
        True if command succeeded, False otherwise
    """
    print(f"\n{'='*70}")
    print(f"Running: {description}")
    print(f"Command: {cmd}")
    print(f"{'='*70}\n")

    result = subprocess.run(cmd, shell=True, capture_output=True, text=True)

    print(result.stdout)
    if result.stderr:
        print("STDERR:", result.stderr)

    if result.returncode != 0:
        print(f"Error: Command failed with return code {result.returncode}")
        return False

    return True


def check_evo_installed():
    """Check if evo is installed"""
    try:
        result = subprocess.run(['evo_ape', '--help'],
                              capture_output=True, text=True)
        return result.returncode == 0
    except FileNotFoundError:
        return False


def main():
    parser = argparse.ArgumentParser(
        description='Evaluate SLAM trajectory accuracy using evo toolkit',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Example:
    python evaluate_trajectory.py \\
        --traj ov2slam_trajectory.txt \\
        --gt gt_trajectory.txt \\
        --output-dir eval_results \\
        --align

Metrics computed:
    - APE (Absolute Pose Error): Global trajectory accuracy
    -RPE (Relative Pose Error): Local drift consistency

Output:
    eval_results/
    ├── ape_xyz.png          # APE visualization (3D plot)
    ├── ape_results.zip      # APE statistics (JSON)
    ├── rpe_1m.png           # RPE at 1m delta
    ├── rpe_5m.png           # RPE at 5m delta
    └── rpe_10m.png          # RPE at 10m delta
        """
    )

    parser.add_argument('--traj', required=True,
                       help='OV2SLAM trajectory (TUM format)')
    parser.add_argument('--gt', required=True,
                       help='Ground truth trajectory (TUM format)')
    parser.add_argument('--output-dir', default='eval_results',
                       help='Output directory for results (default: eval_results)')
    parser.add_argument('--no-align', action='store_true',
                       help='Skip alignment (use if already aligned)')
    parser.add_argument('--rpe-deltas', type=float, nargs='+', default=[1.0, 5.0, 10.0],
                       help='RPE delta units in meters (default: 1.0 5.0 10.0)')

    args = parser.parse_args()

    # Check evo installation
    print("Checking evo installation...")
    if not check_evo_installed():
        print("Error: evo is not installed!")
        print("\nPlease install evo:")
        print("  pip install evo --upgrade")
        sys.exit(1)
    print("  evo is installed ✓")

    # Create output directory
    os.makedirs(args.output_dir, exist_ok=True)
    print(f"Output directory: {args.output_dir}")

    # Base command components
    align_flag = '' if args.no_align else '--align --correct_scale'
    pose_relation = '--pose_relation trans_part'

    # Track success
    all_success = True

    # ========================================
    # 1. APE Evaluation (Primary Metric)
    # ========================================
    print("\n" + "="*70)
    print("APE (Absolute Pose Error) Evaluation")
    print("="*70)

    # APE with XYZ plot
    cmd = (f"evo_ape tum {args.traj} {args.gt} {align_flag} {pose_relation} "
           f"--plot --plot_mode xyz --save_plot {args.output_dir}/ape_xyz.png")
    all_success &= run_command(cmd, "APE evaluation (XYZ plot)")

    # Save APE results
    cmd = (f"evo_ape tum {args.traj} {args.gt} {align_flag} {pose_relation} "
           f"--save_results {args.output_dir}/ape_results.zip")
    all_success &= run_command(cmd, "Saving APE results")

    # ========================================
    # 2. RPE Evaluation (Local Drift)
    # ========================================
    print("\n" + "="*70)
    print("RPE (Relative Pose Error) Evaluation")
    print("="*70)

    for delta in args.rpe_deltas:
        cmd = (f"evo_rpe tum {args.traj} {args.gt} {align_flag} "
               f"--delta_unit m --delta {delta} {pose_relation} "
               f"--plot --plot_mode xyz --save_plot {args.output_dir}/rpe_{delta:.0f}m.png")
        all_success &= run_command(cmd, f"RPE evaluation (delta={delta}m)")

    # ========================================
    # 3. Trajectory Visualization
    # ========================================
    print("\n" + "="*70)
    print("Trajectory Visualization")
    print("="*70)

    cmd = (f"evo_traj tum {args.traj} {args.gt} "
           f"-p --plot_mode xyz --save_plot {args.output_dir}/trajectories.png")
    all_success &= run_command(cmd, "Trajectory comparison plot")

    # ========================================
    # 4. Summary
    # ========================================
    print(f"\n{'='*70}")
    print("EVALUATION SUMMARY")
    print(f"{'='*70}")
    print(f"Trajectory: {args.traj}")
    print(f"Ground Truth: {args.gt}")
    print(f"Alignment: {'Disabled' if args.no_align else 'Enabled (SE3 + scale)'}")
    print(f"\nResults saved to: {args.output_dir}/")
    print(f"  - ape_xyz.png          # APE plot (3D view)")
    print(f"  - ape_results.zip      # APE statistics (max, mean, median, RMSE, std)")
    print(f"  - rpe_*.png            # RPE plots at different deltas")
    print(f"  - trajectories.png    # Trajectory comparison")

    if all_success:
        print("\n✓ Evaluation completed successfully!")
        print("\nTo view detailed statistics:")
        print(f"  unzip -l {args.output_dir}/ape_results.zip")
    else:
        print("\n✗ Warning: Some evaluation steps failed.")
        print("  Check output above for details.")

    # Return exit code
    sys.exit(0 if all_success else 1)


if __name__ == '__main__':
    main()
