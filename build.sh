#!/bin/bash
# Build OV2SLAM

set -e

echo "========================================"
echo "  OV2SLAM Build Script"
echo "========================================"

# Create build directory
BUILD_DIR="build"
rm -rf $BUILD_DIR
mkdir -p $BUILD_DIR
cd $BUILD_DIR

# Configure with CMake
echo "Configuring CMake..."

CMAKE_ARGS="-DENABLE_PROFILING=ON"
if [ ! -z "$ENABLE_RERUN" ]; then
  CMAKE_ARGS="$CMAKE_ARGS -DENABLE_RERUN=$ENABLE_RERUN"
fi
if [ ! -z "$ENABLE_TESTS" ]; then
  CMAKE_ARGS="$CMAKE_ARGS -DENABLE_TESTS=$ENABLE_TESTS"
fi

cmake $CMAKE_ARGS ..

# Build
echo "Building OV2SLAM..."
make -j$(nproc)

# Run tests if enabled
if [ ! -z "$ENABLE_TESTS" ]; then
  echo ""
  echo "========================================"
  echo "  Running Tests"
  echo "========================================"
  ctest --output-on-failure
fi

echo ""
echo "========================================"
echo "  Build Complete!"
echo "========================================"
echo ""
echo "To run OV2SLAM on your dataset:"
echo "  cd .."
echo "  ./$BUILD_DIR/ov2slam parameters_files/pohang00.yaml ~/datasets/pohang00"
echo ""
if [ ! -z "$ENABLE_TESTS" ]; then
  echo "To run tests manually:"
  echo "  cd $BUILD_DIR"
  echo "  ./gps_tests"
fi
