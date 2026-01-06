# Logger Thread Safety Test Requirements

**Phase:** 1.3 - Agent 3 (Testing/Validation Review)
**Date:** 2026-01-06
**Component:** `Logger` class (`include/logger.hpp`)
**Review Focus:** Thread safety and correctness of mutex protection implementation

---

## Executive Summary

**Testing Status:** NEEDS ADDITIONAL TESTS

The Logger class has been modified with `ProfiledMutex` protection for all public methods. However, comprehensive testing is required to verify thread safety under concurrent access scenarios.

**Implementation Status:**
- ✅ Mutex added to all public methods
- ✅ Uses RAII lock guards (exception-safe)
- ✅ Static mutex (shared across all Logger instances)
- ⚠️ No dedicated thread safety tests exist
- ⚠️ No performance regression tests exist

---

## Threading Context Analysis

### Current Usage Pattern

**Single Producer Model:**
1. **Main Thread** → Adds poses during tracking (every frame)
   - `Logger::addSE3Pose(time, Twc, is_kf)` at ~30 Hz

2. **Same Thread (at shutdown)** → Writes trajectory files
   - `Logger::writeTrajectory()`
   - `Logger::writeKfsTrajectory()`
   - `Logger::addKfSE3Pose()` (during iteration)

### Critical Sections

**Method 1:** `addSE3Pose()` - Called every frame
```cpp
// Line 109-136 in logger.hpp
// Protected: std::lock_guard<ProfiledMutex> lock(logger_mutex_);
// Operations:
//   - push_back() to vse3pose_ (vector)
//   - push_back() to vfullse3pose_ (vector)
//   - push_back() to vkittipose_ (vector)
//   - push_back() to vframepose_ (vector)
//   - Read from vframepose_.back() (previous pose)
//   - SE3 computations
```

**Method 2:** `addKfSE3Pose()` - Called during keyframe iteration
```cpp
// Line 138-141 in logger.hpp
// Protected: std::lock_guard<ProfiledMutex> lock(logger_mutex_);
// Operations:
//   - emplace() to vse3kfpose_ (map)
```

**Method 3:** `writeTrajectory()` - Called at shutdown
```cpp
// Line 143-169 in logger.hpp
// Protected: std::lock_guard<ProfiledMutex> lock(logger_mutex_);
// Operations:
//   - Open file
//   - Iterate vse3pose_
//   - Write + flush to file
//   - Close file
```

**Method 4:** `writeKfsTrajectory()` - Called at shutdown
```cpp
// Line 227-252 in logger.hpp
// Protected: std::lock_guard<ProfiledMutex> lock(logger_mutex_);
// Operations:
//   - Open file
//   - Iterate vse3kfpose_
//   - Write + flush to file
//   - Close file
```

### Potential Race Conditions (Pre-Mutex)

**Without mutex protection, these could occur:**
1. **Data Race:** `addSE3Pose()` writes to vectors while `writeTrajectory()` reads them
2. **Iterator Invalidation:** Vector resize during iteration
3. **Torn Writes:** Partial state updates during concurrent `push_back()`
4. **File Corruption:** Interleaved file writes (unlikely but possible)

---

## Test Requirements

### 1. Thread Safety Test Requirements

#### Test 1.1: Concurrent Write + Read
**Priority:** CRITICAL
**Estimated Duration:** 5 minutes
**Description:** Verify that concurrent pose addition and trajectory writing produces correct output

**Test Scenario:**
- Thread A: Continuously add poses at 30 Hz (realistic SLAM rate)
- Thread B: Every 1 second, write trajectory to file
- Duration: 10 seconds (300 poses added)

**Test Parameters:**
```cpp
int num_poses = 300;
int write_interval_ms = 1000;
int add_interval_ms = 33;  // ~30 Hz
```

**Success Criteria:**
- ✅ No crashes or segfaults
- ✅ Final file contains exactly `num_poses` lines
- ✅ Each line has exactly 8 fields (timestamp + 3 position + 4 quaternion)
- ✅ No partial lines (no mid-line interruptions)
- ✅ No interleaved data (lines are complete)
- ✅ Quaternion norms ≈ 1.0 (±0.01)

**Validation Commands:**
```bash
# Count lines
wc -l test_trajectory.txt
# Should equal num_poses

# Check field count
awk '{print NF}' test_trajectory.txt | sort -u
# Should be only: 8

# Check quaternion normalization
awk '{print $5^2 + $6^2 + $7^2 + $8^2}' test_trajectory.txt |
  awk 'BEGIN{min=1; max=1} {if($1<min)min=$1; if($1>max)max=$1} END{print min, max}'
# Should be: 0.99 1.01
```

**Implementation Notes:**
- Use `std::thread` for concurrent execution
- Use `std::atomic<bool>` for shutdown signaling
- Compare output with single-threaded baseline

---

#### Test 1.2: Concurrent Keyframe + Regular Pose Addition
**Priority:** HIGH
**Estimated Duration:** 3 minutes
**Description:** Verify concurrent calls to `addSE3Pose()` and `addKfSE3Pose()`

**Test Scenario:**
- Thread A: Add regular poses at 30 Hz
- Thread B: Add keyframe poses at 3 Hz (10% of frames)
- Duration: 10 seconds (300 regular + 30 keyframe poses)

**Success Criteria:**
- ✅ No crashes
- ✅ All regular poses present in trajectory file
- ✅ All keyframe poses present in keyframe file
- ✅ No duplicates in keyframe file (map ensures uniqueness)
- ✅ Timestamps are monotonically increasing

**Validation:**
```bash
# Check for duplicates
sort test_kfs.txt | uniq -d
# Should be empty

# Check timestamp ordering
awk 'NR>1 {if($1 <= prev) print "Out of order at line", NR; prev=$1}' test_kfs.txt
# Should be empty
```

---

#### Test 1.3: Concurrent Write + Write
**Priority:** MEDIUM
**Estimated Duration:** 2 minutes
**Description:** Verify concurrent calls to different write methods

**Test Scenario:**
- Thread A: Write trajectory every 500ms
- Thread B: Write keyframe trajectory every 500ms (offset by 250ms)
- Duration: 5 seconds (10 writes per thread)

**Success Criteria:**
- ✅ No file corruption
- ✅ All files are valid (not empty, correct format)
- ✅ No interleaved writes between files (separate file handles)

**Validation:**
- Check file locks don't deadlock (test completes in <10 seconds)
- Verify both files have consistent data

---

### 2. Performance Test Requirements

#### Test 2.1: Mutex Overhead Measurement
**Priority:** HIGH
**Estimated Duration:** 10 minutes
**Description:** Measure performance impact of mutex protection

**Test Setup:**
- Baseline: Single-threaded execution (no contention)
- Test: Multi-threaded execution (with contention)
- Metric: Time per operation, throughput

**Test Cases:**
```cpp
// Case 1: Single-threaded baseline
for(int i = 0; i < 10000; i++) {
    Logger::addSE3Pose(time, pose, false);
}
Logger::writeTrajectory("baseline.txt");

// Case 2: Two threads (contention)
Thread A: addSE3Pose() for 5000 iterations
Thread B: addKfSE3Pose() for 5000 iterations
Logger::writeTrajectory("contended.txt");
```

**Acceptable Overhead Threshold:**
- **Per-operation overhead:** < 1% (mutex lock/unlock)
- **Total throughput:** > 99% of baseline
- **Contention ratio:** < 5% (from ProfiledMutex stats)

**Profiling Output:**
```
Expected from SyncProfiler:
Mutex Name          Locks    Wait (ms)    Hold (ms)    Avg Wait    Avg Hold    Contention %
Logger::logger_mutex_  10000      <50         <100        <0.005      <0.010      <5%
```

**Validation:**
- Build with `-DENABLE_PROFILING=ON`
- Check profiling report at program exit
- Compare wait times vs hold times

---

#### Test 2.2: File Write Performance
**Priority:** LOW
**Estimated Duration:** 5 minutes
**Description:** Verify that mutex doesn't serialize I/O unnecessarily

**Test Scenario:**
- Add 10,000 poses
- Write trajectory file
- Measure time

**Success Criteria:**
- Total time < 1 second (10k poses)
- File I/O time dominates over mutex lock time
- Flush frequency (every line) doesn't cause excessive slowdown

**Optimization Check:**
- Consider batching writes (remove intermediate `flush()`)
- Verify mutex scope is minimal (only around vector access, not entire file write)

---

### 3. Correctness Test Requirements

#### Test 3.1: Output Format Validation
**Priority:** CRITICAL
**Estimated Duration:** 2 minutes
**Description:** Verify output file format matches specification

**Test Data:**
- 100 known poses with ground truth values
- Deterministic timestamps (0.0, 1.0, 2.0, ...)

**Validation Checks:**
```bash
# 1. Correct number of fields
awk '{if(NF != 8) print "Line", NR, "has", NF, "fields"}' test_traj.txt

# 2. Timestamp field is numeric (field 1)
awk '{if($1 !~ /^-?[0-9]+\.?[0-9]*$/) print "Invalid timestamp at line", NR}' test_traj.txt

# 3. Position fields are numeric (fields 2-4)
awk '{for(i=2;i<=4;i++) if($i !~ /^-?[0-9]+\.?[0-9]*$/) print "Invalid position at line", NR}' test_traj.txt

# 4. Quaternion fields are numeric (fields 5-8)
awk '{for(i=5;i<=8;i++) if($i !~ /^-?[0-9]+\.?[0-9]*$/) print "Invalid quaternion at line", NR}' test_traj.txt

# 5. Quaternion normalization
awk '{q=$5^2+$6^2+$7^2+$8^2; if(q < 0.99 || q > 1.01) print "Unnormalized quaternion at line", NR, "norm=", q}' test_traj.txt

# 6. No scientific notation in output (should be fixed-point)
awk '{if($0 ~ /e[+-]?[0-9]/) print "Scientific notation at line", NR}' test_traj.txt
```

**Success Criteria:**
- ✅ All checks pass (no output from validation commands)
- ✅ Format matches specification: `timestamp tx ty tz qx qy qz qw`
- ✅ Precision is 9 decimal places (setprecision(9))

---

#### Test 3.2: Data Integrity Verification
**Priority:** HIGH
**Estimated Duration:** 3 minutes
**Description:** Verify output matches input data

**Test Procedure:**
1. Generate 100 known poses with deterministic values
2. Add to Logger
3. Write to file
4. Read back and compare

**Comparison Logic:**
```cpp
for(int i = 0; i < num_poses; i++) {
    SE3Pose original = known_poses[i];
    SE3Pose read_back = read_pose_from_file(i);

    ASSERT_NEAR(original.time_, read_back.time_, 1e-9);
    ASSERT_NEAR(original.twc_[0], read_back.twc_[0], 1e-9);
    ASSERT_NEAR(original.twc_[1], read_back.twc_[1], 1e-9);
    ASSERT_NEAR(original.twc_[2], read_back.twc_[2], 1e-9);
    ASSERT_NEAR(original.qwc_[0], read_back.qwc_[0], 1e-9);
    // ... etc
}
```

**Success Criteria:**
- ✅ All poses match exactly (bit-for-bit)
- ✅ No rounding errors beyond precision limit
- ✅ No missing or extra poses

---

#### Test 3.3: Regression Test vs Pre-Fix Behavior
**Priority:** HIGH
**Estimated Duration:** 15 minutes
**Description:** Ensure mutex protection doesn't change output (except for correctness)

**Test Procedure:**
1. Run OV2SLAM on 1000 frames of pohang00 dataset
2. Compare trajectory with baseline (known-good version)
3. Check for differences

**Validation:**
```bash
# Generate trajectory
./build/ov2slam parameters_files/pohang00.yaml ~/datasets/pohang00 0 1000

# Compare with baseline
diff ov2slam_traj.txt baseline/ov2slam_traj.txt

# Or use evo for trajectory comparison (if installed)
evo_ape euroc baseline.txt ov2slam_traj.txt -r --plot
```

**Success Criteria:**
- ✅ Trajectories are identical (if previous version was correct)
- ✅ Or trajectory is more correct (fixes previous corruption)
- ✅ Position error < 1mm compared to ground truth (if available)

---

### 4. Integration Test Requirements

#### Test 4.1: Full SLAM Pipeline Integration
**Priority:** CRITICAL
**Estimated Duration:** 30 minutes
**Description:** Test Logger within complete OV2SLAM system

**Test Parameters:**
- Dataset: pohang00 (or available dataset)
- Frame range: 0-1000 (or entire dataset)
- Configuration: Default parameters

**Test Procedure:**
```bash
# Build with profiling
./build.sh

# Run SLAM
/usr/bin/time -v ./build/ov2slam parameters_files/pohang00.yaml ~/datasets/pohang00 0 1000

# Check exit code
echo $?
# Should be 0
```

**Success Criteria:**
- ✅ Clean exit (exit code 0)
- ✅ No segfaults during shutdown
- ✅ All output files created:
  - `ov2slam_traj.txt`
  - `ov2slam_traj_kitti.txt`
  - `ov2slam_kfs_traj.txt`
- ✅ Output files are valid (pass format validation)
- ✅ Profiling report shows mutex contention < 10%
- ✅ Total runtime < 2x baseline (no major slowdown)

**Shutdown Verification:**
```
Expected output sequence:
1. "OV2SLAM Finished"
2. "Going to write the computed trajectory into : ov2slam_traj.txt"
3. "Trajectory file written!"
4. "Going to write the computed KFs trajectory into : ov2slam_kfs_traj.txt"
5. "Kfs Trajectory file written!"
6. "OV2SLAM is stopping!"
```

---

#### Test 4.2: Concurrent Rerun + Logger
**Priority:** MEDIUM
**Estimated Duration:** 20 minutes
**Description:** Verify Logger doesn't conflict with Rerun visualization (if enabled)

**Test Setup:**
- Build with `ENABLE_RERUN=ON`
- Run SLAM with visualization
- Both Rerun and Logger writing simultaneously

**Success Criteria:**
- ✅ No crashes
- ✅ Both Logger files and Rerun logs are valid
- ✅ Frame rate remains acceptable (> 10 fps)

---

### 5. Stress Test Requirements

#### Test 5.1: Maximum Frame Rate
**Priority:** HIGH
**Estimated Duration:** 5 minutes
**Description:** Test Logger at maximum pose addition rate

**Test Parameters:**
```cpp
// Simulate high-speed camera (e.g., 120 Hz)
int frame_rate_hz = 120;
int duration_sec = 10;
int num_poses = frame_rate_hz * duration_sec;  // 1200 poses

// Thread A: Add poses as fast as possible
for(int i = 0; i < num_poses; i++) {
    Logger::addSE3Pose(i * 1.0/frame_rate_hz, random_pose(), false);
}

// Thread B: Write trajectory every 1 second
while(running) {
    Logger::writeTrajectory("stress_test.txt");
    std::this_thread::sleep_for(std::chrono::seconds(1));
}
```

**Success Criteria:**
- ✅ No data loss
- ✅ No crashes
- ✅ Mutex contention remains < 20%
- ✅ System remains responsive (no lockups)

---

#### Test 5.2: Keyframe Burst
**Priority:** MEDIUM
**Estimated Duration:** 3 minutes
**Description:** Test scenario where all frames become keyframes

**Test Scenario:**
- Add 1000 poses, mark all as keyframes
- This calls both `addSE3Pose()` and `addKfSE3Pose()` for every frame

**Success Criteria:**
- ✅ No slowdown (double write cost is acceptable)
- ✅ No deadlocks
- ✅ Keyframe trajectory file contains all 1000 entries

---

#### Test 5.3: Rapid Shutdown
**Priority:** MEDIUM
**Estimated Duration:** 2 minutes
**Description:** Test shutdown while poses are being added

**Test Scenario:**
1. Start adding poses at 30 Hz
2. After 1 second, trigger shutdown (call `writeResults()`)
3. Verify shutdown is clean

**Success Criteria:**
- ✅ All poses added before shutdown are written
- ✅ No partial file write
- ✅ Clean exit (exit code 0)

**Edge Case:**
- What if `addSE3Pose()` is called concurrently with `writeTrajectory()`?
- Mutex should prevent race condition

---

### 6. Edge Cases

#### Test 6.1: Empty File Writes
**Priority:** LOW
**Estimated Duration:** 1 minute
**Description:** Verify behavior with no data

**Test Procedure:**
```cpp
// Don't add any poses
Logger::writeTrajectory("empty.txt");
Logger::writeKfsTrajectory("empty_kfs.txt");
```

**Success Criteria:**
- ✅ Files are created (0 bytes or header only)
- ✅ No crashes
- ✅ No error messages

---

#### Test 6.2: Rapid Successive Calls
**Priority:** MEDIUM
**Estimated Duration:** 2 minutes
**Description:** Test same method called repeatedly without delay

**Test Procedure:**
```cpp
for(int i = 0; i < 1000; i++) {
    Logger::addSE3Pose(i, pose, false);
}
// No sleep, back-to-back calls
```

**Success Criteria:**
- ✅ No mutex starvation
- ✅ All 1000 poses added
- ✅ Lock acquisition doesn't fail

---

#### Test 6.3: Exception During Write
**Priority:** LOW
**Estimated Duration:** 5 minutes
**Description:** Verify exception safety

**Test Procedure:**
- Simulate disk full error (inject fault)
- Or use invalid file path

**Success Criteria:**
- ✅ Exception is propagated (not silently swallowed)
- ✅ Mutex is released (RAII ensures this)
- ✅ No memory leaks
- ✅ System remains in consistent state

**Note:** This may require mocking filesystem or using `LD_PRELOAD` to intercept `write()` calls

---

#### Test 6.4: Concurrent Flush + Write
**Priority:** LOW
**Estimated Duration:** 2 minutes
**Description:** Verify explicit flush doesn't interfere with concurrent writes

**Test Scenario:**
- Thread A: Continuously adding poses
- Thread B: Calling `writeTrajectory()` (which does `flush()` inside loop)
- Thread C: Explicitly calling `flush()` on file stream (if exposed)

**Success Criteria:**
- ✅ No race conditions on file handle
- ✅ Flush doesn't block other operations excessively
- ✅ Data is consistently written to disk

---

## Test Priority Matrix

| Test ID | Test Name | Priority | Duration | Dependencies |
|---------|-----------|----------|----------|--------------|
| 1.1 | Concurrent Write + Read | CRITICAL | 5 min | None |
| 1.2 | Concurrent KF + Regular | HIGH | 3 min | 1.1 |
| 1.3 | Concurrent Write + Write | MEDIUM | 2 min | 1.1 |
| 2.1 | Mutex Overhead | HIGH | 10 min | None |
| 2.2 | File Write Performance | LOW | 5 min | 2.1 |
| 3.1 | Format Validation | CRITICAL | 2 min | None |
| 3.2 | Data Integrity | HIGH | 3 min | None |
| 3.3 | Regression Test | HIGH | 15 min | 3.1, 3.2 |
| 4.1 | Full Integration | CRITICAL | 30 min | All above |
| 4.2 | Rerun Integration | MEDIUM | 20 min | 4.1 |
| 5.1 | Max Frame Rate | HIGH | 5 min | 2.1 |
| 5.2 | Keyframe Burst | MEDIUM | 3 min | 1.2 |
| 5.3 | Rapid Shutdown | MEDIUM | 2 min | 1.1 |
| 6.1 | Empty File | LOW | 1 min | None |
| 6.2 | Rapid Calls | MEDIUM | 2 min | None |
| 6.3 | Exception Safety | LOW | 5 min | None |
| 6.4 | Concurrent Flush | LOW | 2 min | None |

**Total Estimated Time:** ~2 hours (including setup and analysis)

---

## Test Implementation Guide

### Suggested Test Structure

```
test/
├── test_logger_thread_safety.cpp    # Tests 1.1, 1.2, 1.3
├── test_logger_performance.cpp      # Tests 2.1, 2.2
├── test_logger_correctness.cpp      # Tests 3.1, 3.2, 3.3
├── test_logger_integration.sh       # Test 4.1 (bash script)
├── test_logger_stress.cpp           # Tests 5.1, 5.2, 5.3
└── test_logger_edge_cases.cpp       # Tests 6.1, 6.2, 6.3, 6.4
```

### Build System Integration

Add to `CMakeLists.txt`:
```cmake
# Logger thread safety tests
add_executable(test_logger_thread_safety test/test_logger_thread_safety.cpp)
target_link_libraries(test_logger_thread_safety ${PROJECT_NAME} pthread)

add_executable(test_logger_performance test/test_logger_performance.cpp)
target_link_libraries(test_logger_performance ${PROJECT_NAME} pthread)

# ... etc
```

### Minimal Test Framework

Since OV2SLAM doesn't use Google Test, use simple assertion macros:
```cpp
#define ASSERT_TRUE(cond) \
    if(!(cond)) { \
        std::cerr << "FAILED: " << #cond << " at " << __FILE__ << ":" << __LINE__ << "\n"; \
        return false; \
    }

#define ASSERT_NEAR(a, b, eps) \
    if(std::abs((a)-(b)) > (eps)) { \
        std::cerr << "FAILED: " << (a) << " != " << (b) << " at " << __FILE__ << ":" << __LINE__ << "\n"; \
        return false; \
    }
```

---

## Approval Checklist

**Testing Approval:** NEEDS ADDITIONAL TESTS

**Required Before Production:**
- [ ] Test 1.1 (Concurrent Write + Read) - CRITICAL
- [ ] Test 3.1 (Format Validation) - CRITICAL
- [ ] Test 4.1 (Full Integration) - CRITICAL
- [ ] Test 2.1 (Mutex Overhead) - HIGH
- [ ] Test 3.3 (Regression Test) - HIGH

**Nice to Have:**
- [ ] Test 1.2 (Concurrent KF + Regular)
- [ ] Test 5.1 (Max Frame Rate)
- [ ] All stress tests

**Can Be Deferred:**
- [ ] Test 2.2 (File Write Performance)
- [ ] Test 4.2 (Rerun Integration)
- [ ] Edge case tests (6.x)

---

## Next Steps

1. **Implement Test 1.1** (highest priority)
   - Creates basic thread safety validation
   - Quick win (5 minutes)

2. **Implement Test 3.1** (automated validation)
   - Creates regression test framework
   - Can be run on all future changes

3. **Run Full Integration Test** (Test 4.1)
   - Validates complete system
   - Most realistic scenario

4. **Measure Performance** (Test 2.1)
   - Quantify mutex overhead
   - Ensure < 1% impact

5. **Document Results**
   - Update this document with test results
   - Mark each test as PASS / FAIL / PARTIAL

---

## Conclusion

The Logger mutex protection implementation is **theoretically sound** but **lacks empirical validation**. The required tests above will provide confidence that:

1. Thread safety is achieved (no races, no corruption)
2. Performance is not degraded (< 1% overhead)
3. Correctness is maintained (output matches specification)
4. Integration with SLAM pipeline is smooth
5. Edge cases are handled gracefully

**Estimated Effort:** 1-2 days for full test suite implementation and validation.

**Risk Assessment:**
- **High Risk:** Tests 1.1, 3.1, 4.1 (critical for correctness)
- **Medium Risk:** Tests 2.1, 3.3, 5.1 (performance and regression)
- **Low Risk:** Edge cases and stress tests

**Recommendation:** Implement CRITICAL and HIGH priority tests before deploying to production. DEFER LOW priority tests to future iterations.
