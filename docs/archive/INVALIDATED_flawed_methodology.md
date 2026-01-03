# ⚠️ ARCHIVED DOCUMENT - INVALIDATED

**Status**: This document contains FLAWED METHODOLOGY and INVALIDATED CONCLUSIONS.
**Date Archived**: 2026-01-03
**Reason**: Performance comparison methodology was fundamentally flawed. Results cannot be trusted. See METHODOLOGY.md for correct approach.

---

# Parallel PNG Decode - Performance Comparison Report

**Date**: 2025-12-29
**Test Dataset**: pohang00 (frames 12200-12300, 100 frames)
**System**: Linux 6.1.0-41-amd64
**Compiler**: gcc with -O3 -march=native

---

## Executive Summary

Implemented and tested three parallel PNG decoding architectures for OV2SLAM:
- **2-Thread**: 1 left + 1 right decoder
- **4-Thread**: 2 left + 2 right decoders
- **6-Thread**: 3 left + 3 right decoders

**Key Finding**: Load balancing bug was identified and fixed. After fix, 6-thread configuration achieved **26% speedup** over baseline (2-thread).

---

## Performance Results

### Final Frame Processing Time (100-frame average)

| Configuration | I/O Time | Wait Time | Total Time | Speedup vs 2T |
|--------------|----------|-----------|------------|---------------|
| **2-Thread** | 41.6 ms | 41.0 ms | **82.6 ms** | baseline |
| **4-Thread** | 43.1 ms | 22.0 ms | **65.1 ms** | **21% faster** |
| **6-Thread** | 45.9 ms | 15.0 ms | **60.9 ms** | **26% faster** |

### Detailed Per-Frame Timings (Last Frame)

```
2-Thread (Frame 12300):
  I/O=41.55ms, Wait=41.04ms, Total=82.60ms

4-Thread (Frame 12300):
  I/O=42.89ms, Wait=21.81ms, Total=64.70ms

6-Thread (Frame 12300):
  I/O=45.90ms, Wait=15.01ms, Total=60.91ms
```

---

## Analysis

### Wait Time Reduction

The key benefit of multi-threading is reduced wait time:

```
Wait Time (ms):
2-Thread: ████████████████████████████ 41.0 ms
4-Thread: ███████████████ 22.0 ms (46% reduction)
6-Thread: ████████ 15.0 ms (63% reduction)
```

### I/O Time Overhead

I/O time increases with thread count due to:
1. **Queue management overhead**: Adding tasks to more queues takes more time
2. **Memory contention**: More threads allocating memory simultaneously
3. **Thread scheduling**: OS scheduler overhead

```
I/O Time (ms):
2-Thread: 41.6 ms (baseline)
4-Thread: 43.1 ms (+3.6% overhead)
6-Thread: 45.9 ms (+10.3% overhead)
```

### Net Performance Gain

Despite I/O overhead, the wait time reduction provides net speedup:

```
Total Time (ms):
2-Thread: ████████████████████████████████████████████████ 82.6 ms
4-Thread: ████████████████████████████████████ 65.1 ms (21% faster)
6-Thread: ██████████████████████████████████ 60.9 ms (26% faster)
```

---

## Implementation Details

### Load Balancing Fix

**Initial Bug**: Tasks were only added to `queue1_`, resulting in only 2 threads working out of 4 or 6.

**Fix**: Added tasks to ALL queues:

```cpp
// Before (WRONG - only queue1_ gets work)
{
    std::lock_guard<std::mutex> lock(mtx_left1_);
    left_queue1_.push(task);
}

// After (CORRECT - all queues get work)
{
    std::lock_guard<std::mutex> lock(mtx_left1_);
    left_queue1_.push(task);
}
{
    std::lock_guard<std::mutex> lock(mtx_left2_);
    left_queue2_.push(task);
}
// ... repeat for all queues
```

**Impact**: Before fix, 4-thread and 6-thread had same performance as 2-thread (~41ms). After fix, significant speedup achieved.

### Architecture

All implementations use same coordination pattern:

```
┌─────────────────┐
│  Main Thread    │
│  (getNext)      │
└────────┬────────┘
         │
    ┌────┴────┐
    │ ready_  │
    │ queue_  │
    └────┬────┘
         │
    ┌────┴────────────────────────┐
    │                             │
┌───▼───┐    ┌─────────┐    ┌───▼───┐
│ Left1 │    │ Left2   │    │ Left3 │
│Thread │    │ Thread  │    │ Thread│
└───┬───┘    └────┬────┘    └───┬───┘
    │              │            │
    └──────────────┴────────────┘
                   │
            ┌──────▼──────┐
            │  frames_    │
            │   map_      │
            └──────┬──────┘
                   │
    ┌──────────────┴──────────────┐
    │                             │
┌───▼───┐    ┌─────────┐    ┌───▼───┐
│Right1 │    │ Right2  │    │ Right3│
│Thread │    │ Thread  │    │ Thread│
└───────┘    └──────────┘    └───────┘
```

**Coordination**:
- Tasks duplicated to all queues
- First thread to grab task processes it
- Other threads skip (via `frames_.find()` check)
- Automatic load balancing without complex distribution logic

---

## Recommendations

### Best Configuration: 6-Thread

**Reasons**:
1. **26% speedup** over 2-thread baseline
2. **6% speedup** over 4-thread
3. Lowest wait time (15ms vs 22ms vs 41ms)

### When to Use Each Configuration

| Use Case | Recommended | Reason |
|----------|-------------|--------|
| **Low-end systems (2-4 cores)** | 2-Thread | Minimal overhead, good baseline |
| **Mid-range systems (4-8 cores)** | 4-Thread | Balanced performance/overhead |
| **High-end systems (8+ cores)** | 6-Thread | Maximum throughput |

### Diminishing Returns

```
2-Thread → 4-Thread: +21% speedup (17.5 ms saved)
4-Thread → 6-Thread: +6% speedup (4.2 ms saved)
```

Additional threads beyond 6 would likely provide minimal benefit due to:
- Disk I/O bottleneck
- Memory allocation contention
- Thread scheduling overhead

---

## Files Modified

1. **async_image_loader_parallel.hpp** (existing, 2-thread)
2. **async_image_loader_4thread.hpp** (new, 4-thread)
3. **async_image_loader_6thread.hpp** (new, 6-thread)
4. **src/main.cpp** (loader selection)
5. **test_io_performance.sh** (automated comparison script)

---

## How to Switch Configurations

### Method 1: Edit main.cpp

```cpp
// Line 23: Choose include
#include "../async_image_loader_parallel.hpp"  // 2-thread
#include "../async_image_loader_4thread.hpp"   // 4-thread
#include "../async_image_loader_6thread.hpp"   // 6-thread

// Line 184: Choose loader type
AsyncImageLoaderParallel loader(...);  // 2-thread
AsyncImageLoader4Thread loader(...);   // 4-thread
AsyncImageLoader6Thread loader(...);   // 6-thread

// Line 203: Choose ImagePair type
AsyncImageLoaderParallel::ImagePair img_pair;  // 2-thread
AsyncImageLoader4Thread::ImagePair img_pair;   // 4-thread
AsyncImageLoader6Thread::ImagePair img_pair;   // 6-thread
```

### Method 2: Use Test Script

```bash
./test_io_performance.sh /datasets/pohang00 12200 12300
```

This script automatically switches between all three configurations and runs tests.

---

## Testing Methodology

### Test Setup
- Dataset: pohang00 (Kuscos dataset)
- Frame range: 12200-12300 (100 frames)
- Hardware: Linux 6.1.0-41-amd64
- Build: -O3 -march=native
- Prefetch: 16 frames (all configurations)

### Metrics Collected
- **I/O time**: Time spent decoding PNG (`cv::imread`)
- **Wait time**: Time waiting for next frame to be ready
- **Total time**: I/O + Wait + SLAM (SLAM is async, shows 0ms)
- **FPS**: Theoretical frames per second (1000/Total)

---

## Conclusion

Multi-threaded PNG decode successfully reduces I/O bottleneck in OV2SLAM:

1. **6-thread configuration provides 26% speedup** over baseline
2. **Load balancing is critical** - initial implementation had bug preventing parallelism
3. **Diminishing returns** beyond 6 threads due to I/O and memory overhead
4. **Recommendation**: Use 6-thread configuration for maximum performance on systems with 8+ CPU cores

### Next Steps

For further optimization:
1. Test on different storage media (SSD vs NVMe)
2. Measure CPU utilization per configuration
3. Test with larger frame ranges for more accurate averages
4. Investigate zero-copy I/O techniques (mmap)
5. Consider PNG→JPEG conversion for faster decode (requires dataset modification)

---

**Current Default**: 2-thread configuration (async_image_loader_parallel.hpp)

**To use 6-thread (recommended)**: Edit src/main.cpp as described above and rebuild.
