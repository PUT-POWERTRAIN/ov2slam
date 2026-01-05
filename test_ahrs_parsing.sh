#!/bin/bash

# Quick test to verify AHRS loading works without running full SLAM
# This tests just the GTLoader component

echo "=== Quick AHRS Loading Test ==="
echo ""

# Compile a minimal test that loads AHRS file
cat > /tmp/test_ahrs_minimal.cpp << 'EOF'
#include "gt_loader.hpp"
#include <iostream>

int main() {
    GTLoader loader;
    std::string ahrs_file = "/home/wojtess/datasets/pohang00/navigation/ahrs.txt";

    std::cout << "Loading AHRS file: " << ahrs_file << std::endl;
    bool success = loader.loadFromAHRS(ahrs_file);

    if (success) {
        std::cout << "SUCCESS: AHRS file loaded" << std::endl;
        std::cout << "No crashes during parsing - IMU data stored in memory" << std::endl;
        return 0;
    } else {
        std::cerr << "FAILED: Could not load AHRS file" << std::endl;
        return 1;
    }
}
EOF

g++ -std=c++17 -I/usr/include/eigen3 -I/home/wojtess/Documents/powertrain/ov2slam-standalone/include \
    /tmp/test_ahrs_minimal.cpp src/gt_loader.cpp -o /tmp/test_ahrs_minimal \
    -lopencv_core -lopencv_imgcodecs 2>&1

if [ $? -eq 0 ]; then
    echo "Compilation successful"
    echo ""
    /tmp/test_ahrs_minimal
    exit $?
else
    echo "Compilation failed"
    exit 1
fi
