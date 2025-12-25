#pragma once

#include <Eigen/Dense>
#include <string>
#include <vector>
#include <unordered_map>

/**
 * @brief Ground truth loader for GPS and AHRS data from Pohang Canal dataset
 *
 * AHRS FILE FORMAT:
 * ------------------
 * timestamp qx qy qz qw wx wy wz ax ay az
 *
 * Where:
 * - timestamp: double, seconds since epoch
 * - qx,qy,qz,qw: Quaternion components (orientation in BODY frame)
 *   - File convention: scalar-last (qx, qy, qz, qw)
 *   - Eigen convention: scalar-first (qw, qx, qy, qz)
 * - wx,wy,wz: Angular velocity in rad/s
 * - ax,ay,az: Linear acceleration in m/s²
 *
 * IMPORTANT NOTES:
 * ----------------
 * 1. qx,qy,qz are quaternion components, NOT Euler angles!
 *    Do NOT read them as roll,pitch,yaw - this will cause large orientation errors.
 *
 * 2. AHRS provides orientation in BODY frame (vehicle reference)
 *    SLAM works in CAMERA frame (camera reference)
 *    Transform required: T_cam_world = T_body_world * T_cam_body
 *
 * 3. Quaternion validation:
 *    - Norm should be ~1.0 (normalized)
 *    - If norm >> 1.0, you're reading wrong columns (probably wx,wy,wz)
 *
 * GPS FILE FORMAT:
 * ----------------
 * timestamp lat N/S lon E/W heading quality n_sat hdop altitude
 */
class GTLoader {
public:
    GTLoader();

    // Load ground truth from GPS/AHRS files
    bool loadFromGPS(const std::string& gps_file);
    bool loadFromAHRS(const std::string& ahrs_file);

    // Get pose at specific timestamp
    bool getPoseAt(double timestamp, Eigen::Vector3d& position, Eigen::Quaterniond& orientation);

    // Get orientation only (for AHRS-only mode)
    bool getOrientationOnlyAt(double timestamp, Eigen::Quaterniond& orientation);

    // Get all GT positions (for trajectory visualization)
    const std::vector<Eigen::Vector3d>& getTrajectory() const { return positions_; }

    // Get origin (first point) - useful for coordinate conversion
    const Eigen::Vector3d& getOrigin() const { return origin_; }

private:
    struct GTPose {
        double timestamp;
        double latitude;
        double longitude;
        double altitude;
        double heading;
        Eigen::Quaterniond orientation;  // From AHRS
    };

    struct AHRSPose {
        double timestamp;
        Eigen::Quaterniond orientation;
    };

    std::vector<GTPose> gt_poses_;
    std::unordered_map<double, size_t> timestamp_map_;
    std::vector<AHRSPose> ahrs_poses_;
    std::unordered_map<double, size_t> ahrs_timestamp_map_;

    std::vector<Eigen::Vector3d> positions_;  // Converted to local XYZ
    Eigen::Vector3d origin_;  // Origin in local XYZ

    // Origin in geodetic coordinates (for geodeticToLocal conversion)
    double origin_lat_ = 0.0;
    double origin_lon_ = 0.0;
    double origin_alt_ = 0.0;

    // Simple geodetic to local conversion (first point = origin)
    void convertToLocal();
    Eigen::Vector3d geodeticToLocal(double lat, double lon, double alt);
};
