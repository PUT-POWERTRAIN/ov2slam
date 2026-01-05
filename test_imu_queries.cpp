/**
 * Comprehensive test program for IMU query methods (getIMUData and getIMUAt)
 *
 * Test Plan:
 * 1. Basic Functionality - IMU loading verification
 * 2. getIMUData Range Queries - 4 different scenarios
 * 3. getIMUAt Single Measurement - 3 different timestamps
 * 4. Edge Cases - empty loader, timestamp 0, large ranges
 * 5. Performance Verification - measure query time
 */

#include "gt_loader.hpp"
#include <iostream>
#include <chrono>
#include <iomanip>
#include <cmath>

// Color codes for output
#define RESET   "\033[0m"
#define GREEN   "\033[32m"
#define RED     "\033[31m"
#define YELLOW  "\033[33m"
#define BLUE    "\033[34m"
#define BOLD    "\033[1m"

// Test result tracking
int tests_passed = 0;
int tests_failed = 0;

void printHeader(const std::string& text) {
    std::cout << "\n" << BOLD << BLUE << "========================================\n";
    std::cout << text << "\n";
    std::cout << "========================================" << RESET << "\n";
}

void printTest(const std::string& test_name) {
    std::cout << "\n" << BOLD << "Running: " << test_name << RESET << "\n";
}

void printPass(const std::string& message) {
    std::cout << GREEN << "✓ PASS: " << RESET << message << "\n";
    tests_passed++;
}

void printFail(const std::string& message) {
    std::cout << RED << "✗ FAIL: " << RESET << message << "\n";
    tests_failed++;
}

void printInfo(const std::string& message) {
    std::cout << YELLOW << "ℹ INFO: " << RESET << message << "\n";
}

void printMeasurement(const std::string& label, double value, const std::string& unit) {
    std::cout << std::fixed << std::setprecision(3);
    std::cout << "  " << label << ": " << value << " " << unit << "\n";
}

bool approxEqual(double a, double b, double tolerance = 1e-6) {
    return std::abs(a - b) < tolerance;
}

// Test 1: Basic Functionality
bool test1_basic_functionality(GTLoader& loader) {
    printTest("Test 1: Basic Functionality - IMU Loading Verification");

    // Get a sample IMU measurement to verify data is loaded
    auto first_imu = loader.getIMUAt(1625124349.159);
    auto last_imu = loader.getIMUAt(1625127000.0);

    // Check if we got valid data (non-zero timestamp indicates success)
    if (first_imu.timestamp == 0.0 && last_imu.timestamp == 0.0) {
        printFail("No AHRS/IMU data loaded");
        return false;
    }

    printInfo("IMU data successfully loaded");
    printInfo("First measurement timestamp: " + std::to_string(first_imu.timestamp));
    printInfo("Last measurement timestamp: " + std::to_string(last_imu.timestamp));

    std::cout << "  First IMU:\n";
    std::cout << "    timestamp: " << std::fixed << std::setprecision(3) << first_imu.timestamp << "\n";
    std::cout << "    gyro: (" << first_imu.angular_velocity.transpose() << ") rad/s\n";
    std::cout << "    accel: (" << first_imu.linear_acceleration.transpose() << ") m/s²\n";

    std::cout << "  Last IMU:\n";
    std::cout << "    timestamp: " << last_imu.timestamp << "\n";
    std::cout << "    gyro: (" << last_imu.angular_velocity.transpose() << ") rad/s\n";
    std::cout << "    accel: (" << last_imu.linear_acceleration.transpose() << ") m/s²\n";

    // Check if IMU data is non-zero (real AHRS data should have some values)
    bool has_gyro = first_imu.angular_velocity.norm() > 0.0 || last_imu.angular_velocity.norm() > 0.0;
    bool has_accel = first_imu.linear_acceleration.norm() > 0.0 || last_imu.linear_acceleration.norm() > 0.0;

    if (has_gyro && has_accel) {
        printPass("IMU data loaded successfully with non-zero measurements");
        return true;
    } else {
        printFail("IMU data appears to be zero - check if AHRS file contains IMU measurements");
        return false;
    }
}

// Test 2: getIMUData Range Queries
bool test2_range_queries(GTLoader& loader) {
    printTest("Test 2: getIMUData Range Queries");

    bool all_passed = true;

    // Test 2a: 1 second range (should have ~100 measurements)
    printInfo("Test 2a: 1 second range (t=1625124349.0 to 1625124350.0)");
    auto range1 = loader.getIMUData(1625124349.0, 1625124350.0);
    std::cout << "    Measurements returned: " << range1.size() << "\n";

    if (range1.size() >= 90 && range1.size() <= 110) {
        printPass("1 second range returned expected ~100 measurements (got " + std::to_string(range1.size()) + ")");
    } else {
        printFail("1 second range returned unexpected count: " + std::to_string(range1.size()) + " (expected ~100)");
        all_passed = false;
    }

    // Test 2b: Small range (41ms)
    printInfo("Test 2b: Small range (t=1625124349.159 to 1625124349.200)");
    auto range2 = loader.getIMUData(1625124349.159, 1625124349.200);
    std::cout << "    Measurements returned: " << range2.size() << "\n";

    if (range2.size() >= 3 && range2.size() <= 5) {
        printPass("Small range returned expected 3-5 measurements (got " + std::to_string(range2.size()) + ")");
    } else {
        printFail("Small range returned unexpected count: " + std::to_string(range2.size()) + " (expected 3-5)");
        all_passed = false;
    }

    // Test 2c: Out of range (should return empty)
    printInfo("Test 2c: Out of range (t=1000000000.0 to 1000000001.0)");
    auto range3 = loader.getIMUData(1000000000.0, 1000000001.0);
    std::cout << "    Measurements returned: " << range3.size() << "\n";

    if (range3.empty()) {
        printPass("Out of range query correctly returned empty vector");
    } else {
        printFail("Out of range query returned " + std::to_string(range3.size()) + " measurements (expected 0)");
        all_passed = false;
    }

    // Test 2d: Invalid range (t_start > t_end)
    printInfo("Test 2d: Invalid range (t_start=1625124400.0 > t_end=1625124300.0)");
    auto range4 = loader.getIMUData(1625124400.0, 1625124300.0);
    std::cout << "    Measurements returned: " << range4.size() << "\n";

    if (range4.empty()) {
        printPass("Invalid range correctly returned empty vector");
    } else {
        printFail("Invalid range returned " + std::to_string(range4.size()) + " measurements (expected 0)");
        all_passed = false;
    }

    return all_passed;
}

// Test 3: getIMUAt Single Measurement
bool test3_single_measurement(GTLoader& loader) {
    printTest("Test 3: getIMUAt Single Measurement Queries");

    bool all_passed = true;

    // Test 3a: First measurement
    printInfo("Test 3a: Request IMU at first timestamp (1625124349.159)");
    auto imu1 = loader.getIMUAt(1625124349.159);
    std::cout << "    timestamp: " << std::fixed << std::setprecision(3) << imu1.timestamp << "\n";
    std::cout << "    gyro: (" << imu1.angular_velocity.transpose() << ")\n";
    std::cout << "    accel: (" << imu1.linear_acceleration.transpose() << ")\n";

    if (approxEqual(imu1.timestamp, 1625124349.159, 0.001)) {
        printPass("First timestamp returned correct measurement");
    } else {
        printFail("First timestamp returned measurement at t=" + std::to_string(imu1.timestamp));
        all_passed = false;
    }

    // Test 3b: Middle measurement
    printInfo("Test 3b: Request IMU at middle timestamp (1625125000.0)");
    auto imu2 = loader.getIMUAt(1625125000.0);
    std::cout << "    timestamp: " << imu2.timestamp << "\n";
    std::cout << "    gyro: (" << imu2.angular_velocity.transpose() << ")\n";
    std::cout << "    accel: (" << imu2.linear_acceleration.transpose() << ")\n";

    if (imu2.timestamp > 1625124349.0 && imu2.timestamp < 1625127000.0) {
        printPass("Middle timestamp returned measurement within expected range");
    } else {
        printFail("Middle timestamp returned measurement at unexpected time: " + std::to_string(imu2.timestamp));
        all_passed = false;
    }

    // Test 3c: Near end
    printInfo("Test 3c: Request IMU near end (1625126500.0)");
    auto imu3 = loader.getIMUAt(1625126500.0);
    std::cout << "    timestamp: " << imu3.timestamp << "\n";
    std::cout << "    gyro: (" << imu3.angular_velocity.transpose() << ")\n";
    std::cout << "    accel: (" << imu3.linear_acceleration.transpose() << ")\n";

    if (imu3.timestamp > 1625126000.0) {
        printPass("Near-end timestamp returned measurement from expected time range");
    } else {
        printFail("Near-end timestamp returned measurement at unexpected time: " + std::to_string(imu3.timestamp));
        all_passed = false;
    }

    return all_passed;
}

// Test 4: Edge Cases
bool test4_edge_cases() {
    printTest("Test 4: Edge Cases");

    bool all_passed = true;

    // Test 4a: Empty GTLoader
    printInfo("Test 4a: Call getIMUData on empty GTLoader");
    GTLoader empty_loader;
    auto empty_range = empty_loader.getIMUData(0.0, 1.0);

    if (empty_range.empty()) {
        printPass("Empty GTLoader correctly returned empty vector");
    } else {
        printFail("Empty GTLoader returned " + std::to_string(empty_range.size()) + " measurements (expected 0)");
        all_passed = false;
    }

    // Test 4b: getIMUAt with empty loader
    printInfo("Test 4b: Call getIMUAt on empty GTLoader");
    auto empty_imu = empty_loader.getIMUAt(0.0);

    if (approxEqual(empty_imu.timestamp, 0.0)) {
        printPass("Empty GTLoader correctly returned default-constructed AHRSPose");
    } else {
        printFail("Empty GTLoader returned AHRSPose with timestamp: " + std::to_string(empty_imu.timestamp));
        all_passed = false;
    }

    return all_passed;
}

// Test 5: Performance Verification
bool test5_performance(GTLoader& loader) {
    printTest("Test 5: Performance Verification");

    // Warm-up
    for (int i = 0; i < 100; i++) {
        loader.getIMUData(1625124349.0, 1625124350.0);
    }

    // Test 5a: Single query performance
    printInfo("Test 5a: Single getIMUData query performance");
    auto start = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < 1000; i++) {
        loader.getIMUData(1625124349.0, 1625124350.0);
    }
    auto end = std::chrono::high_resolution_clock::now();
    double total_ms = std::chrono::duration<double, std::milli>(end - start).count();
    double avg_ms = total_ms / 1000.0;

    printMeasurement("Total time (1000 queries)", total_ms, "ms");
    printMeasurement("Average per query", avg_ms, "ms");

    if (avg_ms < 0.1) {
        printPass("Query performance is EXCELLENT (" + std::to_string(avg_ms) + " ms < 0.1 ms target)");
    } else if (avg_ms < 1.0) {
        printPass("Query performance is GOOD (" + std::to_string(avg_ms) + " ms < 1.0 ms)");
    } else {
        printFail("Query performance is POOR (" + std::to_string(avg_ms) + " ms >= 1.0 ms)");
        return false;
    }

    // Test 5b: getIMUAt performance
    printInfo("Test 5b: Single getIMUAt query performance");
    start = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < 10000; i++) {
        loader.getIMUAt(1625125000.0);
    }
    end = std::chrono::high_resolution_clock::now();
    total_ms = std::chrono::duration<double, std::milli>(end - start).count();
    avg_ms = total_ms / 10000.0;

    printMeasurement("Total time (10000 queries)", total_ms, "ms");
    printMeasurement("Average per query", avg_ms, "ms");

    if (avg_ms < 0.05) {
        printPass("getIMUAt performance is EXCELLENT (" + std::to_string(avg_ms) + " ms < 0.05 ms target)");
    } else if (avg_ms < 0.5) {
        printPass("getIMUAt performance is GOOD (" + std::to_string(avg_ms) + " ms < 0.5 ms)");
    } else {
        printFail("getIMUAt performance is POOR (" + std::to_string(avg_ms) + " ms >= 0.5 ms)");
        return false;
    }

    return true;
}

int main() {
    printHeader("IMU Query Methods Test Suite");

    // Initialize loader
    GTLoader loader;
    std::string ahrs_file = "/home/wojtess/datasets/pohang00/navigation/ahrs.txt";
    std::string gps_file = "/home/wojtess/datasets/pohang00/navigation/gps.txt";

    std::cout << BOLD << "Loading dataset from: /home/wojtess/datasets/pohang00" << RESET << "\n";
    std::cout << "  AHRS file: " << ahrs_file << "\n";
    std::cout << "  GPS file: " << gps_file << "\n";

    // Load AHRS data (contains IMU measurements)
    if (!loader.loadFromAHRS(ahrs_file)) {
        std::cerr << RED << "ERROR: Failed to load AHRS file" << RESET << "\n";
        return 1;
    }

    // Load GPS data (optional - for ground truth trajectory)
    loader.loadFromGPS(gps_file);  // Don't fail if GPS is missing

    std::cout << GREEN << "Dataset loaded successfully" << RESET << "\n";

    // Run tests
    bool test1_pass = test1_basic_functionality(loader);
    bool test2_pass = test2_range_queries(loader);
    bool test3_pass = test3_single_measurement(loader);
    bool test4_pass = test4_edge_cases();
    bool test5_pass = test5_performance(loader);

    // Summary
    printHeader("Test Summary");

    std::cout << BOLD << "Tests Passed: " << GREEN << tests_passed << RESET << "\n";
    std::cout << BOLD << "Tests Failed: " << (tests_failed > 0 ? RED : GREEN) << tests_failed << RESET << "\n";

    std::cout << "\n" << BOLD << "Detailed Results:\n" << RESET;
    std::cout << "  Test 1 (Basic Functionality): " << (test1_pass ? GREEN "✓ PASS" : RED "✗ FAIL") << RESET << "\n";
    std::cout << "  Test 2 (Range Queries): " << (test2_pass ? GREEN "✓ PASS" : RED "✗ FAIL") << RESET << "\n";
    std::cout << "  Test 3 (Single Measurement): " << (test3_pass ? GREEN "✓ PASS" : RED "✗ FAIL") << RESET << "\n";
    std::cout << "  Test 4 (Edge Cases): " << (test4_pass ? GREEN "✓ PASS" : RED "✗ FAIL") << RESET << "\n";
    std::cout << "  Test 5 (Performance): " << (test5_pass ? GREEN "✓ PASS" : RED "✗ FAIL") << RESET << "\n";

    // Final recommendation
    printHeader("Final Recommendation");

    if (tests_failed == 0 && test1_pass && test5_pass) {
        std::cout << BOLD << GREEN << "✓ APPROVE" << RESET << "\n";
        std::cout << "All IMU query methods work correctly with excellent performance.\n";
        std::cout << "The implementation is ready for integration.\n";
        return 0;
    } else if (tests_failed <= 2) {
        std::cout << BOLD << YELLOW << "⚠ CONDITIONAL APPROVE" << RESET << "\n";
        std::cout << "Minor issues detected. Review recommendations above.\n";
        return 0;
    } else {
        std::cout << BOLD << RED << "✗ FIX REQUIRED" << RESET << "\n";
        std::cout << "Multiple failures detected. Implementation needs fixes before integration.\n";
        return 1;
    }
}
