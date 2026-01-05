#!/bin/bash
# Performance comparison script for 2-thread, 4-thread, and 6-thread parallel decode
# Usage: ./test_io_performance.sh <dataset_path> [start_frame] [end_frame]

set -e

if [ $# -lt 1 ]; then
    echo "Usage: $0 <dataset_path> [start_frame] [end_frame]"
    echo "Example: $0 ~/datasets/pohang00 12200 12300"
    exit 1
fi

DATASET_PATH="$1"
START_FRAME="${2:-12200}"
END_FRAME="${3:-12300}"
PARAM_FILE="parameters_files/pohang00.yaml"

echo "=========================================="
echo "I/O Performance Comparison Test"
echo "=========================================="
echo "Dataset: $DATASET_PATH"
echo "Frame range: $START_FRAME - $END_FRAME"
echo ""

# Function to test a specific configuration
test_config() {
    local THREADS=$1
    local HEADER=$2
    local INCLUDE_FILE=$3
    local LOADER_TYPE=$4
    local DESCRIPTION="$5"

    echo "=========================================="
    echo "Testing: $DESCRIPTION"
    echo "=========================================="

    # Backup current main.cpp
    cp src/main.cpp src/main.cpp.bak

    # Modify main.cpp to use the specified loader
    sed -i "s|#include \"../async_image_loader.*\"|#include \"$INCLUDE_FILE\"|g" src/main.cpp
    sed -i "s|AsyncImageLoader.* loader(|$LOADER_TYPE loader(|g" src/main.cpp
    sed -i "s|AsyncImageLoader.*::ImagePair img_pair|${LOADER_TYPE}::ImagePair img_pair|g" src/main.cpp

    # Rebuild
    echo "Building..."
    ./build.sh > /dev/null 2>&1

    # Run test
    echo "Running test..."
    OUTPUT=$(timeout 300 ./build/ov2slam "$PARAM_FILE" "$DATASET_PATH" "$START_FRAME" "$END_FRAME" 2>&1)

    # Restore main.cpp
    mv src/main.cpp.bak src/main.cpp

    # Extract timing info
    echo "$OUTPUT" | grep -E "(Frame ${END_FRAME}|===|Bottleneck|Avg)" | tail -10
    echo ""
}

# Test 1: 2-thread parallel decode (baseline)
echo "=========================================="
echo "Build 1: 2-Thread Parallel Decode"
echo "=========================================="
cp src/main.cpp src/main.cpp.bak
sed -i 's|#include "../async_image_loader.*"|#include "../async_image_loader_parallel.hpp"|g' src/main.cpp
sed -i 's|AsyncImageLoader.* loader(|AsyncImageLoaderParallel loader(|g' src/main.cpp
sed -i 's|AsyncImageLoader.*::ImagePair img_pair|AsyncImageLoaderParallel::ImagePair img_pair|g' src/main.cpp
sed -i 's|4-thread PNG decode:.*|2-thread PNG decode)|g' src/main.cpp
./build.sh > /dev/null 2>&1
echo "Running 2-thread test..."
timeout 300 ./build/ov2slam "$PARAM_FILE" "$DATASET_PATH" "$START_FRAME" "$END_FRAME" 2>&1 | grep -E "(Frame ${END_FRAME}|===|Bottleneck)" | tail -10
mv src/main.cpp.bak src/main.cpp

echo ""

# Test 2: 4-thread parallel decode
echo "=========================================="
echo "Build 2: 4-Thread Parallel Decode"
echo "=========================================="
cp src/main.cpp src/main.cpp.bak
sed -i 's|#include "../async_image_loader.*"|#include "../async_image_loader_4thread.hpp"|g' src/main.cpp
sed -i 's|AsyncImageLoader.* loader(|AsyncImageLoader4Thread loader(|g' src/main.cpp
sed -i 's|AsyncImageLoader.*::ImagePair img_pair|AsyncImageLoader4Thread::ImagePair img_pair|g' src/main.cpp
sed -i 's|2-thread PNG decode)|4-thread PNG decode: 2 left + 2 right)|g' src/main.cpp
./build.sh > /dev/null 2>&1
echo "Running 4-thread test..."
timeout 300 ./build/ov2slam "$PARAM_FILE" "$DATASET_PATH" "$START_FRAME" "$END_FRAME" 2>&1 | grep -E "(Frame ${END_FRAME}|===|Bottleneck)" | tail -10
mv src/main.cpp.bak src/main.cpp

echo ""

# Test 3: 6-thread parallel decode
echo "=========================================="
echo "Build 3: 6-Thread Parallel Decode"
echo "=========================================="
cp src/main.cpp src/main.cpp.bak
sed -i 's|#include "../async_image_loader.*"|#include "../async_image_loader_6thread.hpp"|g' src/main.cpp
sed -i 's|AsyncImageLoader.* loader(|AsyncImageLoader6Thread loader(|g' src/main.cpp
sed -i 's|AsyncImageLoader.*::ImagePair img_pair|AsyncImageLoader6Thread::ImagePair img_pair|g' src/main.cpp
sed -i 's|4-thread PNG decode:.*|6-thread PNG decode: 3 left + 3 right)|g' src/main.cpp
./build.sh > /dev/null 2>&1
echo "Running 6-thread test..."
timeout 300 ./build/ov2slam "$PARAM_FILE" "$DATASET_PATH" "$START_FRAME" "$END_FRAME" 2>&1 | grep -E "(Frame ${END_FRAME}|===|Bottleneck)" | tail -10
mv src/main.cpp.bak src/main.cpp

echo ""
echo "=========================================="
echo "Test Complete!"
echo "=========================================="
