#!/usr/bin/env python3
"""
OV2SLAM Trajectory Validator and Cleaner

Validates OV2SLAM trajectory in TUM format, removes outliers,
and ensures data quality.

Usage:
    python ov2slam_to_tum.py --input ov2slam_trajectory.txt --output ov2slam_clean.txt
"""

import numpy as np
import argparse


def load_tum_trajectory(file_path):
    """
    Load trajectory from TUM format file.

    Format: timestamp tx ty tz qx qy qz qw

    Args:
        file_path: Path to TUM format file

    Returns:
        List of poses with timestamp, position, orientation
    """
    data = []

    with open(file_path) as f:
        for line in f:
            if line.startswith('#') or not line.strip():
                continue

            parts = line.split()
            if len(parts) < 8:
                continue

            timestamp = float(parts[0])
            tx, ty, tz = float(parts[1]), float(parts[2]), float(parts[3])
            qx, qy, qz, qw = float(parts[4]), float(parts[5]), float(parts[6]), float(parts[7])

            # Validate quaternion (should be normalized)
            q = np.array([qx, qy, qz, qw])
            q_norm = np.linalg.norm(q)

            if abs(q_norm - 1.0) > 0.1:
                print(f"Warning: Quaternion not normalized at t={timestamp}, norm={q_norm:.3f}")
                continue

            # Check for NaN or Inf
            if not np.isfinite([tx, ty, tz, qx, qy, qz, qw]).all():
                print(f"Warning: Non-finite values at t={timestamp}, skipping")
                continue

            data.append({
                'timestamp': timestamp,
                'p': np.array([tx, ty, tz]),
                'q': q
            })

    return data


def clean_trajectory(data, max_jump=10.0, max_velocity=50.0):
    """
    Remove outlier poses with large position jumps or unrealistic velocity.

    Args:
        data: List of poses
        max_jump: Maximum position jump between consecutive poses (meters)
        max_velocity: Maximum velocity (m/s) considering timestamp delta

    Returns:
        Cleaned list of poses
    """
    if len(data) == 0:
        return data

    cleaned = [data[0]]

    for i in range(1, len(data)):
        p_prev = cleaned[-1]['p']
        t_prev = cleaned[-1]['timestamp']

        p_curr = data[i]['p']
        t_curr = data[i]['timestamp']

        # Check position jump
        jump = np.linalg.norm(p_curr - p_prev)

        # Check velocity
        dt = t_curr - t_prev
        velocity = jump / dt if dt > 0 else float('inf')

        if jump > max_jump:
            print(f"Warning: Large jump {jump:.2f}m at t={data[i]['timestamp']:.3f}, skipping")
            continue

        if velocity > max_velocity:
            print(f"Warning: High velocity {velocity:.2f}m/s at t={data[i]['timestamp']:.3f}, skipping")
            continue

        cleaned.append(data[i])

    return cleaned


def compute_trajectory_statistics(data):
    """Compute basic trajectory statistics"""
    if len(data) == 0:
        return {}

    timestamps = [p['timestamp'] for p in data]
    positions = np.array([p['p'] for p in data])

    duration = timestamps[-1] - timestamps[0]
    total_distance = np.sum(np.linalg.norm(np.diff(positions, axis=0), axis=0))

    stats = {
        'num_poses': len(data),
        'duration_s': duration,
        'total_distance_m': total_distance,
        'avg_velocity_m_s': total_distance / duration if duration > 0 else 0,
        'start_time': timestamps[0],
        'end_time': timestamps[-1]
    }

    return stats


def main():
    parser = argparse.ArgumentParser(
        description='Validate and clean OV2SLAM trajectory in TUM format',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Example:
    python ov2slam_to_tum.py \\
        --input ov2slam_trajectory.txt \\
        --output ov2slam_trajectory_clean.txt \\
        --max-jump 10.0
        """
    )

    parser.add_argument('--input', required=True, help='Input OV2SLAM trajectory (TUM format)')
    parser.add_argument('--output', required=True, help='Output cleaned TUM file')
    parser.add_argument('--max-jump', type=float, default=10.0,
                       help='Maximum position jump between consecutive poses (meters)')
    parser.add_argument('--max-velocity', type=float, default=50.0,
                       help='Maximum velocity (m/s) for validation')

    args = parser.parse_args()

    # Load trajectory
    print("Loading OV2SLAM trajectory...")
    data = load_tum_trajectory(args.input)
    print(f"  Loaded {len(data)} poses")

    if len(data) == 0:
        print("Error: No valid poses loaded")
        return

    # Compute statistics
    print("Computing trajectory statistics...")
    stats_before = compute_trajectory_statistics(data)
    print(f"  Duration: {stats_before['duration_s']:.1f} s")
    print(f"  Distance: {stats_before['total_distance_m']:.1f} m")
    print(f"  Avg velocity: {stats_before['avg_velocity_m_s']:.2f} m/s")
    print(f"  Time range: {stats_before['start_time']:.3f} - {stats_before['end_time']:.3f}")

    # Clean trajectory
    print(f"\nCleaning trajectory (max_jump={args.max_jump}m, max_velocity={args.max_velocity}m/s)...")
    cleaned = clean_trajectory(data, max_jump=args.max_jump, max_velocity=args.max_velocity)
    print(f"  Cleaned to {len(cleaned)} poses ({len(cleaned) - len(data)} removed)")

    # Write cleaned trajectory
    print(f"\nWriting TUM file to {args.output}...")
    with open(args.output, 'w') as f:
        f.write("# timestamp tx ty tz qx qy qz qw\n")
        for pose in cleaned:
            t = pose['timestamp']
            p = pose['p']
            q = pose['q']
            f.write(f"{t:.9f} {p[0]:.6f} {p[1]:.6f} {p[2]:.6f} "
                   f"{q[0]:.6f} {q[1]:.6f} {q[2]:.6f} {q[3]:.6f}\n")

    # Compute cleaned statistics
    stats_after = compute_trajectory_statistics(cleaned)
    print(f"  Output: {len(cleaned)} poses written")
    print(f"\nFinal statistics:")
    print(f"  Duration: {stats_after['duration_s']:.1f} s")
    print(f"  Distance: {stats_after['total_distance_m']:.1f} m")
    print(f"  Avg velocity: {stats_after['avg_velocity_m_s']:.2f} m/s")

    print("\nDone!")


if __name__ == '__main__':
    main()
