#!/bin/bash
# Build OV2SLAM in standalone mode (no ROS required)

set -e

echo "========================================"
echo "  OV2SLAM Standalone Build Script"
echo "========================================"

# Copy stub version
echo "Installing standalone visualizer stub..."
cp include/stub_ros_visualizer.hpp include/ros_visualizer.hpp

# Use standalone CMakeLists
echo "Using standalone CMakeLists.txt..."
cp CMakeLists_standalone.txt CMakeLists.txt

# Create build directory
BUILD_DIR="build_standalone"
rm -rf $BUILD_DIR
mkdir -p $BUILD_DIR
cd $BUILD_DIR

# Configure with CMake (without ROS)
echo "Configuring CMake..."
cmake ..

# Build
echo "Building OV2SLAM standalone..."
make -j$(nproc)

echo ""
echo "========================================"
echo "  Build Complete!"
echo "========================================"
echo ""
echo "To run OV2SLAM on your dataset:"
echo "  cd .."
echo "  ./$BUILD_DIR/ov2slam_standalone parameters_files/pohang00.yaml ~/datasets/pohang00"
