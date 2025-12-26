#include "gt_loader.hpp"
#include <fstream>
#include <sstream>
#include <iostream>
#include <cmath>
#include <algorithm>

// Earth radius in meters
constexpr double EARTH_RADIUS = 6378137.0;

GTLoader::GTLoader() {
    origin_ = Eigen::Vector3d::Zero();
}

bool GTLoader::loadFromGPS(const std::string& gps_file) {
    std::ifstream file(gps_file);
    if (!file.is_open()) {
        std::cerr << "[GTLoader] Failed to open GPS file: " << gps_file << std::endl;
        return false;
    }

    std::string line;
    int line_num = 0;
    double first_lat = 0.0, first_lon = 0.0, first_alt = 0.0;

    while (std::getline(file, line)) {
        line_num++;
        if (line.empty() || line[0] == '#') continue;

        std::istringstream iss(line);
        GTPose pose;
        char ns, ew;

        // Format: timestamp gps_time lat N/S lon E/W heading quality n_sat hdop altitude
        // (11 columns total)
        double quality, num_sats, hdop;
        iss >> pose.timestamp >> pose.gps_time >> pose.latitude >> ns >> pose.longitude >> ew
            >> pose.heading >> quality >> num_sats >> hdop >> pose.altitude;

        // Convert N/S and E/W to signed values
        if (ns == 'S') pose.latitude = -pose.latitude;
        if (ew == 'W') pose.longitude = -pose.longitude;

        if (line_num == 1) {
            first_lat = pose.latitude;
            first_lon = pose.longitude;
            first_alt = pose.altitude;
        }

        gt_poses_.push_back(pose);
        timestamp_map_[pose.timestamp] = gt_poses_.size() - 1;
    }

    std::cout << "[GTLoader] Loaded " << gt_poses_.size() << " GPS poses from " << gps_file << std::endl;

    // Debug: Show first GPS point to verify parsing
    if( gt_poses_.size() > 0 ) {
        std::cout << "[GTLoader] First GPS point:" << std::endl;
        std::cout << "  timestamp: " << gt_poses_[0].timestamp << std::endl;
        std::cout << "  latitude: " << gt_poses_[0].latitude << std::endl;
        std::cout << "  longitude: " << gt_poses_[0].longitude << std::endl;
        std::cout << "  altitude: " << gt_poses_[0].altitude << std::endl;
        std::cout << "  heading: " << gt_poses_[0].heading << std::endl;
    }

    // Set origin from first point
    origin_lat_ = first_lat;
    origin_lon_ = first_lon;
    origin_alt_ = first_alt;
    origin_ = geodeticToLocal(first_lat, first_lon, first_alt);

    // Convert all poses to local coordinates
    convertToLocal();

    // Debug: Show trajectory bounds
    if( positions_.size() > 1 ) {
        Eigen::Vector3d min_pos = positions_[0];
        Eigen::Vector3d max_pos = positions_[0];
        for(const auto& pos : positions_) {
            min_pos = min_pos.cwiseMin(pos);
            max_pos = max_pos.cwiseMax(pos);
        }
        std::cout << "[GTLoader] Trajectory bounds (local XYZ):" << std::endl;
        std::cout << "  Min: x=" << min_pos.x() << ", y=" << min_pos.y() << ", z=" << min_pos.z() << std::endl;
        std::cout << "  Max: x=" << max_pos.x() << ", y=" << max_pos.y() << ", z=" << max_pos.z() << std::endl;
        std::cout << "  Range: dx=" << (max_pos.x() - min_pos.x()) << ", dy=" << (max_pos.y() - min_pos.y()) << ", dz=" << (max_pos.z() - min_pos.z()) << std::endl;
    }

    return true;
}

bool GTLoader::loadFromAHRS(const std::string& ahrs_file) {
    std::ifstream file(ahrs_file);
    if (!file.is_open()) {
        std::cerr << "[GTLoader] Failed to open AHRS file: " << ahrs_file << std::endl;
        return false;
    }

    std::string line;
    int line_num = 0;

    while (std::getline(file, line)) {
        line_num++;
        if (line.empty() || line[0] == '#') continue;

        std::istringstream iss(line);
        double timestamp, qx, qy, qz, qw;
        double wx, wy, wz, ax, ay, az;

        // Format: timestamp qx qy qz qw wx wy wz ax ay az
        // where qx,qy,qz,qw is orientation quaternion (scalar last: qw is scalar)
        // wx,wy,wz is angular rate in rad/s
        // ax,ay,az is linear acceleration in m/s^2
        iss >> timestamp >> qx >> qy >> qz >> qw >> wx >> wy >> wz >> ax >> ay >> az;

        // Create quaternion from file (Eigen uses scalar-first: qw, qx, qy, qz)
        Eigen::Quaterniond q(qw, qx, qy, qz);

        // SANITY CHECK: Validate quaternion on first line
        if( line_num == 1 ) {
            double norm = q.norm();
            if( std::abs(norm - 1.0) > 0.2 ) {
                std::cerr << "[GTLoader] ERROR: Quaternion not normalized! norm=" << norm << std::endl;
                std::cerr << "[GTLoader] Are you reading correct columns?" << std::endl;
                std::cerr << "[GTLoader] Expected format: timestamp qx qy qz qw wx wy wz ax ay az" << std::endl;
                std::cerr << "[GTLoader] Quaternion components should have norm ~1.0" << std::endl;
                std::cerr << "[GTLoader] If norm >> 1.0, you might be reading angular rates instead!" << std::endl;
                return false;
            }

            // SANITY CHECK: Acceleration Z should be ~ -9.8 (gravity)
            if( std::abs(az + 9.8) > 5.0 ) {
                std::cerr << "[GTLoader] WARNING: Acceleration Z looks wrong! az=" << az << " m/s²" << std::endl;
                std::cerr << "[GTLoader] Expected ~ -9.8 m/s² (gravity pointing down)" << std::endl;
                std::cerr << "[GTLoader] Check if column mapping is correct" << std::endl;
            }

            std::cout << "[GTLoader] AHRS format validation passed (qnorm=" << norm << ", az=" << az << ")" << std::endl;
        }

        // Normalize quaternion (AHRS data should already be normalized, but ensure it)
        q.normalize();

        // Store AHRS pose independently for AHRS-only mode
        AHRSPose ahrs_pose;
        ahrs_pose.timestamp = timestamp;
        ahrs_pose.orientation = q;
        ahrs_poses_.push_back(ahrs_pose);
        ahrs_timestamp_map_[timestamp] = ahrs_poses_.size() - 1;

        // Find corresponding GPS pose and add orientation
        // Use simple linear search since AHRS has higher frequency than GPS
        double min_diff = 1.0;
        size_t closest_idx = 0;
        for(size_t i = 0; i < gt_poses_.size(); ++i) {
            double diff = std::abs(gt_poses_[i].timestamp - timestamp);
            if(diff < min_diff) {
                min_diff = diff;
                closest_idx = i;
            }
        }

        if(min_diff < 0.1) {  // Within 100ms
            gt_poses_[closest_idx].orientation = q;
            gt_poses_[closest_idx].timestamp = timestamp;  // Use AHRS timestamp for precision
        }
    }

    std::cout << "[GTLoader] Merged AHRS orientation data" << std::endl;
    return true;
}

bool GTLoader::getPoseAt(double timestamp, Eigen::Vector3d& position, Eigen::Quaterniond& orientation) {
    // Find closest timestamp (simple linear search)
    double min_diff = 1.0;
    size_t closest_idx = 0;
    for(size_t i = 0; i < gt_poses_.size(); ++i) {
        double diff = std::abs(gt_poses_[i].timestamp - timestamp);
        if(diff < min_diff) {
            min_diff = diff;
            closest_idx = i;
        }
    }

    if(min_diff < 0.5 && closest_idx < positions_.size()) {  // Within 500ms
        position = positions_[closest_idx];
        orientation = gt_poses_[closest_idx].orientation;
        return true;
    }
    return false;
}

void GTLoader::convertToLocal() {
    positions_.clear();
    positions_.reserve(gt_poses_.size());

    for (const auto& pose : gt_poses_) {
        Eigen::Vector3d local = geodeticToLocal(pose.latitude, pose.longitude, pose.altitude);
        positions_.push_back(local);
    }
}

Eigen::Vector3d GTLoader::geodeticToLocal(double lat, double lon, double alt) {
    // Simple conversion for small areas: use first point as origin
    // This is accurate enough for visualization purposes

    // Convert to radians
    double lat_rad = lat * M_PI / 180.0;

    // Approximate XYZ offsets from origin (using member variables)
    double lat_offset = (lat - origin_lat_) * M_PI / 180.0;
    double lon_offset = (lon - origin_lon_) * M_PI / 180.0;
    double alt_offset = alt - origin_alt_;

    // Convert to meters
    // NOTE: SLAM uses X=North, Y=East coordinate system
    // GPS lat_offset (North) → X, lon_offset (East) → Y
    double x = lat_offset * EARTH_RADIUS;                          // X = North
    double y = lon_offset * EARTH_RADIUS * std::cos(lat_rad);      // Y = East
    double z = alt_offset;

    return Eigen::Vector3d(x, y, z);
}

bool GTLoader::getOrientationOnlyAt(double timestamp, Eigen::Quaterniond& orientation) {
    double min_diff = 1.0;
    size_t closest_idx = 0;
    for(size_t i = 0; i < ahrs_poses_.size(); ++i) {
        double diff = std::abs(ahrs_poses_[i].timestamp - timestamp);
        if(diff < min_diff) {
            min_diff = diff;
            closest_idx = i;
        }
    }
    if(min_diff < 0.5) {
        orientation = ahrs_poses_[closest_idx].orientation;
        return true;
    }
    return false;
}
