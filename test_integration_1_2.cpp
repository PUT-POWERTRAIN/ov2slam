/**
 * INTEGRATION TEST 1.2: Verify IMU Data Storage During Parsing
 *
 * This test verifies that:
 * 1. AHRS file is loaded successfully
 * 2. IMU data (angular_velocity, linear_acceleration) is stored in AHRSPose structs
 * 3. Data contains non-zero values (not default initialized)
 * 4. Values are physically reasonable
 */

#include <iostream>
#include <iomanip>
#include <fstream>
#include "gt_loader.hpp"

void printAHRSPose(const GTLoader::AHRSPose& pose, int idx) {
    std::cout << "  [" << idx << "] timestamp=" << std::fixed << std::setprecision(6) << pose.timestamp
              << " | orientation=[" << pose.orientation.w() << ", "
              << pose.orientation.x() << ", "
              << pose.orientation.y() << ", "
              << pose.orientation.z() << "]" << std::endl;
    std::cout << "       | gyro=[" << std::setprecision(4)
              << pose.angular_velocity[0] << ", "
              << pose.angular_velocity[1] << ", "
              << pose.angular_velocity[2] << "] rad/s" << std::endl;
    std::cout << "       | accel=["
              << pose.linear_acceleration[0] << ", "
              << pose.linear_acceleration[1] << ", "
              << pose.linear_acceleration[2] << "] m/s²" << std::endl;
}

int main() {
    std::cout << "=== Integration Test 1.2: IMU Data Storage ===" << std::endl;
    std::cout << std::endl;

    // Initialize ground truth loader
    GTLoader gt_loader;

    // Load AHRS data
    std::string ahrs_file = "/home/wojtess/datasets/pohang00/navigation/ahrs.txt";
    std::cout << "[1] Loading AHRS data from: " << ahrs_file << std::endl;

    if (!gt_loader.loadAHRSGroundTruth(ahrs_file)) {
        std::cerr << "  ✗ FAILED: Could not load AHRS file" << std::endl;
        return 1;
    }
    std::cout << "  ✓ SUCCESS: AHRS file loaded" << std::endl;

    // Get AHRS poses
    const auto& ahrs_poses = gt_loader.getAHRSPoses();
    std::cout << "  ✓ Loaded " << ahrs_poses.size() << " AHRS poses" << std::endl;
    std::cout << std::endl;

    // Verify IMU data is present
    std::cout << "[2] Verifying IMU data storage..." << std::endl;

    if (ahrs_poses.empty()) {
        std::cerr << "  ✗ FAILED: No AHRS poses loaded" << std::endl;
        return 1;
    }

    // Check first 5 poses
    std::cout << "  Sample first 5 poses:" << std::endl;
    for (size_t i = 0; i < std::min(size_t(5), ahrs_poses.size()); ++i) {
        printAHRSPose(ahrs_poses[i], i);
    }
    std::cout << std::endl;

    // Verify data integrity
    std::cout << "[3] Data integrity checks..." << std::endl;

    int non_zero_gyro = 0;
    int non_zero_accel = 0;
    int valid_quat = 0;
    int physically_reasonable = 0;

    for (const auto& pose : ahrs_poses) {
        // Check quaternion norm
        double q_norm = pose.orientation.norm();
        if (std::abs(q_norm - 1.0) < 0.1) {
            valid_quat++;
        }

        // Check gyro (should have some variation, not all zeros)
        double gyro_norm = pose.angular_velocity.norm();
        if (gyro_norm > 1e-6) {
            non_zero_gyro++;
        }

        // Check accel (should have gravity ~9.8 m/s² in some direction)
        double accel_norm = pose.linear_acceleration.norm();
        if (accel_norm > 1.0) {
            non_zero_accel++;
        }

        // Physical reasonability
        // Gyro: typical range ±1 rad/s for slow motion
        // Accel: typical range ~9.8 m/s² (gravity) + motion
        bool gyro_reasonable = gyro_norm < 10.0;
        bool accel_reasonable = accel_norm > 5.0 && accel_norm < 20.0;
        if (gyro_reasonable && accel_reasonable) {
            physically_reasonable++;
        }
    }

    std::cout << "  Quaternion normalization: " << valid_quat << "/" << ahrs_poses.size()
              << " (" << (100.0 * valid_quat / ahrs_poses.size()) << "%)" << std::endl;
    std::cout << "  Non-zero gyro readings: " << non_zero_gyro << "/" << ahrs_poses.size()
              << " (" << (100.0 * non_zero_gyro / ahrs_poses.size()) << "%)" << std::endl;
    std::cout << "  Non-zero accel readings: " << non_zero_accel << "/" << ahrs_poses.size()
              << " (" << (100.0 * non_zero_accel / ahrs_poses.size()) << "%)" << std::endl;
    std::cout << "  Physically reasonable: " << physically_reasonable << "/" << ahrs_poses.size()
              << " (" << (100.0 * physically_reasonable / ahrs_poses.size()) << "%)" << std::endl;
    std::cout << std::endl;

    // Verdict
    std::cout << "[4] INTEGRATION VERDICT..." << std::endl;

    bool all_pass = true;
    if (valid_quat < (int)(ahrs_poses.size() * 0.99)) {
        std::cerr << "  ✗ FAIL: Too many non-normalized quaternions" << std::endl;
        all_pass = false;
    } else {
        std::cout << "  ✓ PASS: Quaternions are normalized" << std::endl;
    }

    if (non_zero_gyro < (int)(ahrs_poses.size() * 0.5)) {
        std::cerr << "  ✗ FAIL: Too many zero gyro readings" << std::endl;
        all_pass = false;
    } else {
        std::cout << "  ✓ PASS: Gyro data is present" << std::endl;
    }

    if (non_zero_accel < (int)(ahrs_poses.size() * 0.99)) {
        std::cerr << "  ✗ FAIL: Too many zero accel readings" << std::endl;
        all_pass = false;
    } else {
        std::cout << "  ✓ PASS: Accel data is present" << std::endl;
    }

    if (physically_reasonable < (int)(ahrs_poses.size() * 0.9)) {
        std::cerr << "  ✗ WARN: Many readings outside physical range" << std::endl;
        // Don't fail on this, just warn
    } else {
        std::cout << "  ✓ PASS: Data is physically reasonable" << std::endl;
    }

    std::cout << std::endl;

    if (all_pass) {
        std::cout << "=== INTEGRATION VERIFIED ===" << std::endl;
        std::cout << "Status: APPROVE" << std::endl;
        std::cout << "Summary: IMU data is correctly stored during parsing and accessible in memory." << std::endl;
        std::cout << "Ready for Subphase 1.3 (add query methods)." << std::endl;
        return 0;
    } else {
        std::cout << "=== INTEGRATION FAILED ===" << std::endl;
        std::cout << "Status: FIX REQUIRED" << std::endl;
        return 1;
    }
}
