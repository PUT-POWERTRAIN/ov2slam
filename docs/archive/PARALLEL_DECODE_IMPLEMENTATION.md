# ⚠️ ARCHIVED DOCUMENT - SUPERSEDED

**Status**: This document describes implementation approaches that have been superseded.
**Date Archived**: 2026-01-03
**Reason**: Implementation complete and documented in main codebase. See METHODOLOGY.md for current implementation details.

---

# Parallel PNG Decode Implementation - 2, 4, and 6 Thread Variants

## Overview

This document describes the implementation of three parallel PNG decoding architectures for OV2SLAM's async image loader. The goal was to optimize I/O performance by parallelizing the bottleneck operation (PNG decode with `cv::imread`).

## Baseline Performance

Before optimization:
- **I/O (PNG decode)**: 70-80 ms/frame (~87% of total time)
- **SLAM processing**: ~10 ms/frame (asynchronous)
- **Total**: ~90 ms/frame synchronous

## Implementations

### 1. 2-Thread Parallel Decode (`async_image_loader_parallel.hpp`)

**Architecture**: 1 thread for left images + 1 thread for right images

**Thread Layout**:
```
Left Thread: left_queue_ → cv::imread(left) → frames_[idx].left
Right Thread: right_queue_ → cv::imread(right) → frames_[idx].right
Main Thread: getNext() → wait for both ready → return ImagePair
```

**Key Features**:
- Each thread has its own dedicated work queue
- Coordination via shared `frames_` map with atomic flags
- Condition variables for efficient waiting
- 16-frame prefetch buffer

**Performance**: ~42 ms/frame I/O (2x speedup from 80ms)

**File**: `async_image_loader_parallel.hpp`

---

### 2. 4-Thread Parallel Decode (`async_image_loader_4thread.hpp`)

**Architecture**: 2 threads for left images + 2 threads for right images

**Thread Layout**:
```
Left1 Thread: left_queue1_ → cv::imread(left)
Left2 Thread: left_queue2_ → cv::imread(left)
Right1 Thread: right_queue1_ → cv::imread(right)
Right2 Thread: right_queue2_ → cv::imread(right)
Main Thread: getNext() → wait for both ready → return ImagePair
```

**Key Features**:
- Separate queues per thread for load balancing
- Same coordination mechanism as 2-thread version
- 16-frame prefetch buffer
- Expected: ~2-3x speedup over baseline (depending on CPU cores)

**File**: `async_image_loader_4thread.hpp`

---

### 3. 6-Thread Parallel Decode (`async_image_loader_6thread.hpp`)

**Architecture**: 3 threads for left images + 3 threads for right images

**Thread Layout**:
```
Left1 Thread: left_queue1_ → cv::imread(left)
Left2 Thread: left_queue2_ → cv::imread(left)
Left3 Thread: left_queue3_ → cv::imread(left)
Right1 Thread: right_queue1_ → cv::imread(right)
Right2 Thread: right_queue2_ → cv::imread(right)
Right3 Thread: right_queue3_ → cv::imread(right)
Main Thread: getNext() → wait for both ready → return ImagePair
```

**Key Features**:
- Maximum parallelization for decode-bound workloads
- Same coordination mechanism as other versions
- 16-frame prefetch buffer
- Expected: ~3-4x speedup over baseline (with diminishing returns)

**File**: `async_image_loader_6thread.hpp`

---

## Common Architecture Pattern

All three implementations use the same design pattern:

### Data Structures

```cpp
struct LoadTask {
    double timestamp;
    std::string img_name;
    size_t idx;
};

struct FramePair {
    cv::Mat left;
    cv::Mat right;
    double timestamp;
    size_t idx;
    long left_load_us = 0;
    long right_load_us = 0;
    bool left_ready = false;
    bool right_ready = false;
};

struct ImagePair {
    double timestamp;
    cv::Mat left;
    cv::Mat right;
    size_t frame_idx;
    long load_us;   // Time spent decoding
    long wait_us;   // Time spent waiting for prefetch
};
```

### Coordination Mechanism

1. **Task Distribution**: Each `addFrame()` call adds the task to ALL left queues and ALL right queues
2. **Independent Decoding**: Each worker thread independently processes tasks from its queue
3. **Frame Coordination**: Workers share a `frames_` map to coordinate when both left/right are ready
4. **Output Generation**: When both left and right are ready, an `ImagePair` is pushed to `ready_queue_`

### Synchronization

- **Mutex-protected queues**: Each worker queue has its own mutex
- **Condition variables**: Efficient waiting without busy-spinning
- **Shared frames map**: Protected by `mtx_ready_` for coordination

---

## How to Use

### Switching Between Implementations

**Method 1: Manual Edit**
```bash
# Edit src/main.cpp, line 23:
#include "../async_image_loader_parallel.hpp"  # For 2-thread
#include "../async_image_loader_4thread.hpp"   # For 4-thread
#include "../async_image_loader_6thread.hpp"   # For 6-thread

# Also update line 184:
AsyncImageLoaderParallel loader(...)  # For 2-thread
AsyncImageLoader4Thread loader(...)   # For 4-thread
AsyncImageLoader6Thread loader(...)   # For 6-thread

# Also update line 203:
AsyncImageLoaderParallel::ImagePair img_pair  # For 2-thread
AsyncImageLoader4Thread::ImagePair img_pair   # For 4-thread
AsyncImageLoader6Thread::ImagePair img_pair   # For 6-thread

# Then rebuild:
./build.sh
```

**Method 2: Automated Script**
```bash
# The test_io_performance.sh script automatically switches between all three
# and runs performance tests:
./test_io_performance.sh ~/datasets/pohang00 12200 12300
```

---

## Performance Testing

### Running Individual Tests

```bash
# Build with 2-thread loader (current default)
./build.sh
./build/ov2slam parameters_files/pohang00.yaml ~/datasets/pohang00 12200 12300

# The output includes timing breakdown:
# Frame 100 timings: I/O=X.XXms, SLAM=X.XXms, Wait=X.XXms, Total=X.XXms (X.X fps)
```

### Running Comparison Tests

```bash
# Test all three variants and compare
./test_io_performance.sh ~/datasets/pohang00 12200 12300
```

### Expected Results

| Configuration | I/O Time | Wait Time | Speedup | Notes |
|--------------|----------|-----------|---------|-------|
| Baseline (no async) | 70-80ms | N/A | 1x | Synchronous |
| 2-Thread | ~42ms | ~40ms | 2x | Sweet spot |
| 4-Thread | ~25-30ms | ~0-5ms | 2.5-3x | Diminishing returns |
| 6-Thread | ~20-25ms | ~0ms | 3-4x | CPU bound |

---

## Implementation Details

### Why Separate Queues Per Thread?

**Initial Approach (Shared Queue)**: Race conditions when both threads call `front()`
```cpp
// PROBLEM: Deadlock
task = shared_queue_.front();  // Both threads here
shared_queue_.pop();
```

**Solution**: Each thread has its own queue, tasks are duplicated to all queues
```cpp
// SOLUTION: No race
{
    std::lock_guard<std::mutex> lock(mtx_left1_);
    left_queue1_.push(task);  // Thread 1
}
{
    std::lock_guard<std::mutex> lock(mtx_left2_);
    left_queue2_.push(task);  // Thread 2
}
```

### Load Balancing

Currently, tasks are added to ALL queues. Each thread processes tasks independently, so:
- First thread to grab task processes it
- Other threads skip the task (via `frames_.find()` check)

This provides automatic load balancing without complex work distribution logic.

### Memory Management

- **Frames map**: Holds partially decoded frames (left only or right only)
- **Ready queue**: Holds complete frame pairs ready for SLAM
- **Automatic cleanup**: Frames are erased from map when complete

---

## Current State

**Default Configuration**: 2-thread parallel decode (`async_image_loader_parallel.hpp`)

**Main.cpp**: Currently configured for 2-thread decode

**Build Status**: All three variants compile successfully

---

## Future Optimizations

### Potential Improvements

1. **Work Stealing**: Instead of duplicating tasks to all queues, use work stealing
2. **Adaptive Threading**: Dynamically adjust thread count based on CPU load
3. **JPEG Conversion**: Convert PNG→JPEG for faster decode (dataset modification)
4. **Memory Mapping**: Use mmap for zero-copy I/O (requires OS support)

### Benchmarks Needed

- Test on full dataset (not just 100 frames)
- Measure CPU utilization per variant
- Test on different storage media (SSD vs HDD vs NVMe)
- Measure scaling with prefetch buffer size

---

## Files Modified

1. **async_image_loader_parallel.hpp** - 2-thread decode (existing)
2. **async_image_loader_4thread.hpp** - 4-thread decode (new)
3. **async_image_loader_6thread.hpp** - 6-thread decode (new)
4. **src/main.cpp** - Updated to support switching between loaders
5. **test_io_performance.sh** - Automated comparison script

---

## Troubleshooting

### Build Errors

If you get errors about `pair` not being declared:
- Ensure all references to the ImagePair type match the loader class
- Search for: `AsyncImageLoader*::ImagePair`

### Runtime Errors

If program crashes:
- Check that dataset path is correct
- Verify stereo/left_images/ and stereo/right_images/ exist
- Check that timestamp.txt format is correct

### Performance Issues

If wait time is high:
- Increase prefetch buffer (change `16` to larger value)
- Check that batch queueing is working (should add 8 frames at a time)
- Verify disk I/O is not bottleneck (use `iostat`)

---

## Summary

All three implementations (2, 4, and 6-thread) are complete and ready for testing. The 2-thread version provides 2x speedup with minimal complexity. The 4-thread and 6-thread versions offer additional performance gains but may exhibit diminishing returns depending on CPU core count and disk I/O speed.

To test on your dataset with actual data, run:
```bash
./test_io_performance.sh <path_to_dataset> <start_frame> <end_frame>
```
