#!/usr/bin/env python3
"""
GPS/AHRS to TUM Format Converter

Converts GPS (WGS84) + AHRS (body frame orientation) to TUM format trajectory.
Applies body-to-camera extrinsics transformation for direct comparison with OV2SLAM.

Usage:
    python gps_to_tum.py --gps gps.txt --ahrs ahrs.txt --config config.yaml --output gt.txt
"""

import numpy as np
from scipy.spatial.transform import Rotation
import yaml
import argparse


def load_extrinsics(yaml_path):
    """Load T_body_cam from YAML config file"""
    with open(yaml_path) as f:
        config = yaml.safe_load(f)

    # Read body_T_cam0 matrix (4x4 homogeneous transformation)
    data = config['Camera']['body_T_cam0']['data']
    T_body_cam = np.array(data).reshape(4, 4)

    return T_body_cam


def gps_to_enu(lat, lon, alt, lat0, lon0, alt0):
    """
    Convert GPS (WGS84) to local ENU (East-North-Up) coordinates.

    Uses first GPS point as origin. Simple tangent plane approximation
    sufficient for local trajectory evaluation (< 10km scale).

    Args:
        lat, lon, alt: Current GPS position (degrees, meters)
        lat0, lon0, alt0: Origin GPS position

    Returns:
        np.array([x, y, z]) in ENU frame (meters)
    """
    R = 6378137.0  # Earth radius in meters

    lat_rad = lat * np.pi / 180.0
    dlat = (lat - lat0) * np.pi / 180.0
    dlon = (lon - lon0) * np.pi / 180.0
    dalt = alt - alt0

    # ENU coordinates
    x = dlon * R * np.cos(lat_rad)  # East
    y = dlat * R                    # North
    z = dalt                        # Up

    return np.array([x, y, z])


def slerp(q1, q2, t):
    """
    Spherical linear interpolation between quaternions.

    Args:
        q1, q2: Quaternions in scalar-first format (w, x, y, z)
        t: Interpolation parameter [0, 1]

    Returns:
        Interpolated quaternion (w, x, y, z)
    """
    # Normalize quaternions
    q1 = q1 / np.linalg.norm(q1)
    q2 = q2 / np.linalg.norm(q2)

    # Compute cosine of angle between quaternions
    dot = np.dot(q1, q2)

    # If quaternions are close, use linear interpolation
    if abs(dot) > 0.9995:
        result = q1 + t * (q2 - q1)
        return result / np.linalg.norm(result)

    # Ensure shortest path
    if dot < 0:
        q2 = -q2
        dot = -dot

    theta = np.arccos(np.clip(dot, -1.0, 1.0))
    sin_theta = np.sin(theta)

    if abs(sin_theta) < 1e-6:
        return q1

    w1 = np.sin((1 - t) * theta) / sin_theta
    w2 = np.sin(t * theta) / sin_theta

    return w1 * q1 + w2 * q2


def load_gps_data(gps_file):
    """Load GPS data from file"""
    data = []

    with open(gps_file) as f:
        for line in f:
            if line.startswith('#') or not line.strip():
                continue

            parts = line.split()
            if len(parts) < 11:
                continue

            timestamp = float(parts[0])
            lat = float(parts[2])
            ns = parts[3]
            lon = float(parts[4])
            ew = parts[5]
            heading = float(parts[6])
            quality = int(parts[7])
            n_sat = int(parts[8])
            hdop = float(parts[9])
            alt = float(parts[10])

            # Convert N/S, E/W to signed values
            if ns == 'S':
                lat = -lat
            if ew == 'W':
                lon = -lon

            # Quality filter: skip low-quality GPS
            if quality < 1 or hdop > 2.0:
                continue

            data.append({
                'timestamp': timestamp,
                'lat': lat,
                'lon': lon,
                'alt': alt,
                'heading': heading
            })

    return data


def load_ahrs_data(ahrs_file):
    """Load AHRS orientation data from file"""
    data = []

    with open(ahrs_file) as f:
        for line in f:
            if line.startswith('#') or not line.strip():
                continue

            parts = line.split()
            if len(parts) < 5:
                continue

            timestamp = float(parts[0])
            qx = float(parts[1])
            qy = float(parts[2])
            qz = float(parts[3])
            qw = float(parts[4])

            # File format: scalar-last (qx, qy, qz, qw)
            # Convert to scalar-first (w, x, y, z) for internal processing
            q = np.array([qw, qx, qy, qz])

            # Validate quaternion norm
            q_norm = np.linalg.norm(q)
            if abs(q_norm - 1.0) > 0.1:
                continue

            data.append({
                'timestamp': timestamp,
                'q': q  # scalar-first format (w, x, y, z)
            })

    return data


def interpolate_orientation(ahrs_data, target_timestamp):
    """Interpolate AHRS orientation to target timestamp using SLERP"""
    # Find surrounding samples
    before = None
    after = None

    for sample in ahrs_data:
        if sample['timestamp'] <= target_timestamp:
            before = sample
        elif sample['timestamp'] > target_timestamp:
            after = sample
            break

    # Handle edge cases
    if before is None and after is None:
        return None
    elif before is None:
        return after['q']
    elif after is None:
        return before['q']
    elif before['timestamp'] == after['timestamp']:
        return before['q']

    # Check gap size
    dt = after['timestamp'] - before['timestamp']
    if dt > 0.5:  # Gap too large (> 500ms)
        print(f"Warning: Large AHRS gap {dt:.3f}s at t={target_timestamp:.3f}")
        return before['q']

    # Interpolate
    alpha = (target_timestamp - before['timestamp']) / dt
    return slerp(before['q'], after['q'], alpha)


def transform_body_to_camera(p_body, q_body, T_body_cam):
    """
    Transform pose from body frame to camera frame.

    Args:
        p_body: Position in body frame (ENU coordinates)
        q_body: Orientation in body frame (w, x, y, z)
        T_body_cam: 4x4 homogeneous transformation (body to camera)

    Returns:
        p_cam: Position in camera frame
        q_cam: Orientation in camera frame (w, x, y, z)
    """
    # Compute inverse transformation
    T_cam_body = np.linalg.inv(T_body_cam)

    # Transform position: p_cam = R_cam_body @ p_body + t_cam_body
    p_cam = T_cam_body[:3, :3] @ p_body + T_cam_body[:3, 3]

    # Transform orientation: q_cam = q_body * q_body_cam
    R_body_cam = T_body_cam[:3, :3]
    q_body_cam = Rotation.from_matrix(R_body_cam).as_quat()  # (x, y, z, w)

    # Convert q_body from (w,x,y,z) to (x,y,z,w) for scipy
    q_body_xyzw = np.array([q_body[1], q_body[2], q_body[3], q_body[0]])

    # Compose rotations
    R_cam = Rotation.from_quat(q_body_cam) * Rotation.from_quat(q_body_xyzw)
    q_cam_xyzw = R_cam.as_quat()  # (x, y, z, w)

    # Convert back to (w, x, y, z)
    q_cam = np.array([q_cam_xyzw[3], q_cam_xyzw[0], q_cam_xyzw[1], q_cam_xyzw[2]])

    return p_cam, q_cam


def main():
    parser = argparse.ArgumentParser(
        description='Convert GPS/AHRS to TUM format for SLAM evaluation',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Example:
    python gps_to_tum.py \\
        --gps ~/datasets/pohang00/navigation/gps.txt \\
        --ahrs ~/datasets/pohang00/navigation/ahrs.txt \\
        --config parameters_files/pohang00.yaml \\
        --output gt_trajectory.txt
        """
    )

    parser.add_argument('--gps', required=True, help='GPS file path')
    parser.add_argument('--ahrs', required=True, help='AHRS file path')
    parser.add_argument('--config', required=True, help='YAML config file (for extrinsics)')
    parser.add_argument('--output', required=True, help='Output TUM file')
    parser.add_argument('--no-transform', action='store_true',
                       help='Skip body-to-camera transformation (keep in body frame)')

    args = parser.parse_args()

    # Load data
    print("Loading GPS data...")
    gps_data = load_gps_data(args.gps)
    print(f"  Loaded {len(gps_data)} GPS points")

    if len(gps_data) == 0:
        print("Error: No GPS data loaded")
        return

    print("Loading AHRS data...")
    ahrs_data = load_ahrs_data(args.ahrs)
    print(f"  Loaded {len(ahrs_data)} AHRS orientations")

    # Load extrinsics
    if not args.no_transform:
        print("Loading extrinsics from config...")
        T_body_cam = load_extrinsics(args.config)
        print("  Extrinsics loaded")
    else:
        T_body_cam = None
        print("Skipping body-to-camera transformation")

    # Get origin from first GPS point
    lat0 = gps_data[0]['lat']
    lon0 = gps_data[0]['lon']
    alt0 = gps_data[0]['alt']

    print(f"Origin: lat={lat0:.6f}, lon={lon0:.6f}, alt={alt0:.2f}")

    # Process each GPS point
    print("Merging GPS + AHRS data...")
    merged_data = []

    for gps_point in gps_data:
        timestamp = gps_point['timestamp']

        # Convert GPS to ENU (body frame position)
        p_body_enu = gps_to_enu(
            gps_point['lat'], gps_point['lon'], gps_point['alt'],
            lat0, lon0, alt0
        )

        # Get AHRS orientation at this timestamp
        q_body = interpolate_orientation(ahrs_data, timestamp)
        if q_body is None:
            print(f"Warning: No AHRS data for t={timestamp:.3f}, skipping")
            continue

        # Transform to camera frame
        if args.no_transform:
            p_cam = p_body_enu
            q_cam = q_body
        else:
            p_cam, q_cam = transform_body_to_camera(p_body_enu, q_body, T_body_cam)

        merged_data.append({
            'timestamp': timestamp,
            'p': p_cam,
            'q': q_cam
        })

    print(f"  Merged {len(merged_data)} poses")

    # Write TUM format
    print(f"Writing TUM file to {args.output}...")
    with open(args.output, 'w') as f:
        f.write("# timestamp tx ty tz qx qy qz qw\n")
        for pose in merged_data:
            t = pose['timestamp']
            p = pose['p']
            q = pose['q']
            # TUM format: x y z qx qy qz qw (scalar-last)
            f.write(f"{t:.9f} {p[0]:.6f} {p[1]:.6f} {p[2]:.6f} "
                   f"{q[1]:.6f} {q[2]:.6f} {q[3]:.6f} {q[0]:.6f}\n")

    print("Done!")
    print(f"  Output: {len(merged_data)} poses written to {args.output}")


if __name__ == '__main__':
    main()
