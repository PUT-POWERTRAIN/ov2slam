/**
 * IMU Data Loading Test
 * Validates that AHRS data is correctly loaded and stored in AHRSPose structs
 */

#include <iostream>
#include <fstream>
#include <vector>
#include <iomanip>
#include <Eigen/Dense>

struct AHRSPose {
    double timestamp;
    double qx, qy, qz, qw;  // Quaternion (scalar-last convention)
    double wx, wy, wz;      // Angular velocity (rad/s)
    double ax, ay, az;      // Linear acceleration (m/s²)

    // Physical validation
    bool isValid() const {
        // Check quaternion normalization
        Eigen::Quaterniond q(qw, qx, qy, qz);
        double norm = q.norm();

        // Check reasonable values
        bool accelValid = std::abs(az + 9.81) < 2.0; // gravity pointing down
        bool gyroValid = std::sqrt(wx*wx + wy*wy + wz*wz) < 1.0; // not spinning too fast

        return std::abs(norm - 1.0) < 0.1 && accelValid && gyroValid;
    }

    void print(std::ostream& os, int precision = 6) const {
        os << std::fixed << std::setprecision(precision)
           << "Timestamp: " << timestamp << "\n"
           << "  Quaternion: [" << qx << ", " << qy << ", " << qz << ", " << qw << "]\n"
           << "  Gyro (rad/s): [" << wx << ", " << wy << ", " << wz << "]\n"
           << "  Accel (m/s²): [" << ax << ", " << ay << ", " << az << "]\n";

        // Compute derived values
        Eigen::Quaterniond q(qw, qx, qy, qz);
        double gyroNorm = std::sqrt(wx*wx + wy*wy + wz*wz);
        double accelNorm = std::sqrt(ax*ax + ay*ay + az*az);

        os << "  Quaternion norm: " << std::setprecision(8) << q.norm() << "\n"
           << std::setprecision(6)
           << "  Gyro magnitude: " << gyroNorm << " rad/s\n"
           << "  Accel magnitude: " << accelNorm << " m/s²\n";
    }
};

std::vector<AHRSPose> loadAhrsFile(const std::string& filepath) {
    std::vector<AHRSPose> poses;
    std::ifstream file(filepath);

    if (!file.is_open()) {
        std::cerr << "ERROR: Cannot open AHRS file: " << filepath << std::endl;
        return poses;
    }

    std::string line;
    int lineNum = 0;
    while (std::getline(file, line)) {
        lineNum++;
        if (line.empty() || line[0] == '#') continue; // Skip empty lines and comments

        AHRSPose pose;
        std::istringstream iss(line);

        // Parse: timestamp qx qy qz qw wx wy wz ax ay az
        if (!(iss >> pose.timestamp >> pose.qx >> pose.qy >> pose.qz >> pose.qw
                   >> pose.wx >> pose.wy >> pose.wz
                   >> pose.ax >> pose.ay >> pose.az)) {
            std::cerr << "WARNING: Failed to parse line " << lineNum << ": " << line << std::endl;
            continue;
        }

        poses.push_back(pose);
    }

    std::cout << "Loaded " << poses.size() << " AHRS poses from " << filepath << std::endl;
    return poses;
}

void runDataQualityChecks(const std::vector<AHRSPose>& poses) {
    if (poses.empty()) {
        std::cerr << "ERROR: No poses to validate!" << std::endl;
        return;
    }

    std::cout << "\n=== DATA QUALITY CHECKS ===" << std::endl;

    // Check 1: Quaternion normalization
    int badQuaternions = 0;
    for (const auto& pose : poses) {
        Eigen::Quaterniond q(pose.qw, pose.qx, pose.qy, pose.qz);
        if (std::abs(q.norm() - 1.0) > 0.1) {
            badQuaternions++;
        }
    }
    std::cout << "Quaternion normalization: " << (poses.size() - badQuaternions) << "/" << poses.size()
              << " valid (norm ≈ 1.0)" << std::endl;

    // Check 2: Acceleration Z axis (should be ~ -9.81 m/s² for gravity)
    int badAccelZ = 0;
    double accelZSum = 0.0;
    for (const auto& pose : poses) {
        accelZSum += pose.az;
        if (std::abs(pose.az + 9.81) > 2.0) {
            badAccelZ++;
        }
    }
    double avgAccelZ = accelZSum / poses.size();
    std::cout << "Acceleration Z axis: " << (poses.size() - badAccelZ) << "/" << poses.size()
              << " valid (avg: " << avgAccelZ << " m/s², expected ~ -9.81)" << std::endl;

    // Check 3: Gyro magnitude (should be small for static/near-static motion)
    int badGyro = 0;
    double gyroMax = 0.0;
    for (const auto& pose : poses) {
        double gyroNorm = std::sqrt(pose.wx*pose.wx + pose.wy*pose.wy + pose.wz*pose.wz);
        gyroMax = std::max(gyroMax, gyroNorm);
        if (gyroNorm > 1.0) { // 1 rad/s ≈ 57°/s
            badGyro++;
        }
    }
    std::cout << "Gyroscope magnitude: " << (poses.size() - badGyro) << "/" << poses.size()
              << " valid (< 1.0 rad/s, max: " << gyroMax << " rad/s)" << std::endl;

    // Check 4: Timestamp monotonicity
    int badTimestamps = 0;
    for (size_t i = 1; i < poses.size(); i++) {
        if (poses[i].timestamp <= poses[i-1].timestamp) {
            badTimestamps++;
        }
    }
    std::cout << "Timestamp monotonicity: " << (poses.size() - badTimestamps - 1) << "/" << (poses.size() - 1)
              << " valid (strictly increasing)" << std::endl;

    // Check 5: Valid AHRSPose::isValid() method
    int invalidPoses = 0;
    for (const auto& pose : poses) {
        if (!pose.isValid()) {
            invalidPoses++;
        }
    }
    std::cout << "Overall validity: " << (poses.size() - invalidPoses) << "/" << poses.size()
              << " passed" << std::endl;
}

int main(int argc, char** argv) {
    std::cout << "=== IMU DATA LOADING TEST ===" << std::endl;

    std::string ahrsFile = "/home/wojtess/datasets/pohang00/navigation/ahrs.txt";

    // Allow override from command line
    if (argc > 1) {
        ahrsFile = argv[1];
    }

    // Load data
    auto poses = loadAhrsFile(ahrsFile);
    if (poses.empty()) {
        std::cerr << "\nFAILED: No data loaded!" << std::endl;
        return 1;
    }

    std::cout << "\n=== SAMPLE DATA (First 5 poses) ===" << std::endl;
    int sampleCount = std::min(5, (int)poses.size());
    for (int i = 0; i < sampleCount; i++) {
        std::cout << "\nPose " << (i+1) << ":\n";
        poses[i].print(std::cout);
    }

    // Run quality checks
    runDataQualityChecks(poses);

    std::cout << "\n=== TEST RESULT ===" << std::endl;

    // Overall verdict
    bool allValid = true;
    for (const auto& pose : poses) {
        if (!pose.isValid()) {
            allValid = false;
            break;
        }
    }

    if (allValid) {
        std::cout << "APPROVE: All IMU data loaded correctly with physically reasonable values" << std::endl;
        return 0;
    } else {
        std::cout << "FAIL: Some IMU data failed validation checks" << std::endl;
        return 1;
    }
}
