# Phase 3.1: Keyframe Threshold Sensitivity Analysis - Test Plan

**Date:** 2025-01-05
**Status:** TEST DESIGN COMPLETE
**Current Keyframe Rate:** 23-26%
**Target Rate:** 10-15%

---

## Executive Summary

Two critical findings from agent analysis:

1. **Code Confirmed:** Condition 5 (High 3D Rejection) uses **backwards logic** - rejects keyframes when 3D points are abundant (>50% of max), which contradicts SLAM principles
2. **Root Cause:** This backwards condition creates keyframe starvation during good tracking, making watchdog (1.0s) the primary keyframe source

**Test Strategy:** Systematic isolation of each condition to measure individual impact, then selective optimization.

---

## Test Matrix

### Baseline Test (Current Configuration)

**Configuration:**
- Watchdog timeout: 1.0s
- Low 3D threshold: 20
- High 3D rejection: ENABLED (50% threshold)
- Stereo time: 5.0s
- Parallax: 30.0°
- Frame count: 10

**Expected:** 23-26% keyframe rate

**Run Command:**
```bash
# Build baseline
./build.sh

# Run 1000 frames
./build/ov2slam parameters_files/pohang00.yaml ~/datasets/pohang00 0 1000 2>&1 | tee baseline_test.log

# Count keyframes
grep "Creating keyframe" baseline_test.log | wc -l
grep "Processed" baseline_test.log
```

**Metrics to Collect:**
- Total frames processed
- Keyframes created
- Keyframe rate (keyframes/frames)
- Exit code (should be 0)
- Processing time (seconds)

---

## Priority 0: Critical Fix Test

### Test P0-1: Disable High 3D Rejection

**Hypothesis:** Disabling the backwards High 3D Rejection will INCREASE keyframe rate (removes blocker), potentially improving trajectory quality.

**Change:**
```cpp
// src/visual_front_end.cpp:1382-1388
// COMMENT OUT the entire block:
/*
if( pcurframe_->nb3dkps_ > 0.5 * pslamstate_->nbmaxkps_
    && (pslamstate_->blocalba_is_on_ || nbimfromkf < 2) )
{
    if( pslamstate_->debug_ )
        std::cout << "  [CONDITION 3] FALSE → Rejecting keyframe (too many 3D keypoints)" << std::endl;
    return false;
}
*/
```

**Expected:** Keyframe rate may increase to 30-40% (removes rejection), but quality should improve

**Run:** 1000 frames

**If rate too high (>25%):** Combine with increased watchdog (Test P1-1 or P1-2)

---

## Priority 1: High-Impact Parameters

### Test P1-1: Watchdog Timeout 1.5s

**Hypothesis:** Increasing watchdog from 1.0s → 1.5s will reduce keyframe rate by ~33% (45 frames/cycle vs 30 frames/cycle).

**Change:**
```cpp
// src/visual_front_end.cpp:1328
if( time_since_last_kf < 1.5 ) {  // Was: 1.0
    return false;
}
```

**Expected:** ~15-17% keyframe rate

**Run:** 1000 frames

**Success Criteria:** Rate in 10-15% target range

---

### Test P1-2: Watchdog Timeout 2.0s

**Hypothesis:** Increasing watchdog to 2.0s will reduce keyframe rate by ~50% (60 frames/cycle vs 30 frames/cycle).

**Change:**
```cpp
// src/visual_front_end.cpp:1328
if( time_since_last_kf < 2.0 ) {  // Was: 1.0
    return false;
}
```

**Expected:** ~11-13% keyframe rate

**Run:** 1000 frames

**Success Criteria:** Rate in 10-15% target range

---

### Test P1-3: Watchdog Timeout 2.5s

**Hypothesis:** Increasing watchdog to 2.5s will reduce keyframe rate by ~60% (75 frames/cycle).

**Change:**
```cpp
// src/visual_front_end.cpp:1328
if( time_since_last_kf < 2.5 ) {  // Was: 1.0
    return false;
}
```

**Expected:** ~9-11% keyframe rate (below target)

**Run:** 1000 frames

**Purpose:** Find lower boundary of acceptable range

---

### Test P1-4: Low 3D Threshold 50 (Increase from 20)

**Hypothesis:** Increasing low 3D threshold from 20 → 50 will reduce keyframes created during marginal tracking.

**Change:**
```cpp
// src/visual_front_end.cpp:1375
if( pcurframe_->nb3dkps_ < 50 &&  // Was: 20
    nbimfromkf >= 2 ) {
    return true;
}
```

**Expected:** -3-5% keyframe rate reduction

**Run:** 1000 frames

---

### Test P1-5: Stereo Time 3.0s (Decrease from 5.0s)

**Hypothesis:** Reducing stereo time from 5.0s → 3.0s will create more frequent keyframes, improving trajectory accuracy.

**Change:**
```cpp
// src/visual_front_end.cpp:1394
if( pslamstate_->stereo_ && time_diff > 3.0  // Was: 5.0
    && !pslamstate_->blocalba_is_on_ ) {
    return true;
}
```

**Expected:** +2-3% keyframe rate, better ATE/RPE

**Run:** 1000 frames

**Purpose:** Test if more frequent stereo keyframes improve accuracy

---

## Priority 2: Medium-Impact Parameters

### Test P2-1: Parallax Threshold 25° (Decrease from 30°)

**Hypothesis:** Lowering parallax threshold will create keyframes more frequently during turns.

**Change:**
```yaml
# parameters_files/pohang00.yaml
finit_parallax: 25.0  # Was: 30.0
```

**Expected:** +2-4% keyframe rate

**Run:** 1000 frames

---

### Test P2-2: Parallax Threshold 40° (Increase from 30°)

**Hypothesis:** Increasing parallax threshold will reduce keyframes during moderate motion.

**Change:**
```yaml
# parameters_files/pohang00.yaml
finit_parallax: 40.0  # Was: 30.0
```

**Expected:** -2-3% keyframe rate

**Run:** 1000 frames

---

### Test P2-3: Frame Count 15 (Increase from 10)

**Hypothesis:** Increasing frame count in parallax condition will reduce transient keyframes.

**Change:**
```cpp
// src/visual_front_end.cpp:1405
bool cx = med_rot_parallax >= pslamstate_->finit_parallax_ / 2.
    || (pslamstate_->stereo_ && !pslamstate_->blocalba_is_on_ && pcurframe_->id_-pkf->id_ > 15);  // Was: 10
```

**Expected:** -1-2% keyframe rate

**Run:** 1000 frames

---

## Priority 3: Combined Configurations

### Test P3-1: Conservative Configuration

**Goal:** Achieve 10-12% keyframe rate with multiple conservative changes.

**Changes:**
- Watchdog: 2.0s
- Low 3D threshold: 50
- Parallax: 35°
- Frame count: 12
- Stereo time: 4.0s

**Expected:** ~10-12% keyframe rate

**Run:** 1000 frames

**Success Criteria:** Rate in target range, no tracking loss

---

### Test P3-2: Aggressive Configuration

**Goal:** Test if lower rate (8-10%) degrades tracking quality.

**Changes:**
- Watchdog: 2.5s
- Low 3D threshold: 60
- Parallax: 40°
- Frame count: 15
- Stereo time: 5.0s (baseline)

**Expected:** ~8-10% keyframe rate

**Run:** 1000 frames

**Watch for:** Tracking failures, trajectory drift

---

### Test P3-3: High 3D Disabled + Watchdog 2.0s

**Goal:** Fix backwards logic while maintaining target rate.

**Changes:**
- Disable High 3D Rejection (Comment out lines 1382-1388)
- Watchdog: 2.0s
- All other: baseline

**Expected:** ~12-15% keyframe rate, improved quality

**Run:** 1000 frames

**Hypothesis:** Removing backwards logic improves quality, watchdog compensates for rate

---

## Test Execution Protocol

### For Each Test:

1. **Apply changes:**
   - Edit `src/visual_front_end.cpp` for code changes
   - Edit `parameters_files/pohang00.yaml` for parameter changes
   - Document changes in test log

2. **Rebuild:**
   ```bash
   ./build.sh
   ```

3. **Run test:**
   ```bash
   ./build/ov2slam parameters_files/pohang00.yaml ~/datasets/pohang00 0 1000 2>&1 | tee test_PX-Y.log
   ```

4. **Extract metrics:**
   ```bash
   # Count keyframes
   grep "Creating keyframe" test_PX-Y.log | wc -l

   # Count frames
   grep "Processed" test_PX-Y.log | tail -1

   # Calculate rate
   KEYFRAMES=$(grep "Creating keyframe" test_PX-Y.log | wc -l)
   FRAMES=$(grep "Processed" test_PX-Y.log | tail -1 | awk '{print $3}')
   python3 -c "print(f'Rate: {100*$KEYFRAMES/$FRAMES:.1f}%')"
   ```

5. **Check for errors:**
   ```bash
   grep -i "error\|fail\|crash" test_PX-Y.log
   ```

6. **Log results:**
   - Edit this file, add result to Results Table below
   - Note any anomalies or unexpected behavior

---

## Results Table

| Test ID | Description | Keyframes | Frames | Rate (%) | ATE (m) | Notes |
|---------|-------------|-----------|--------|----------|---------|-------|
| BASELINE | Current config | TBD | TBD | 23-26 | TBD | Baseline |
| P0-1 | Disable High 3D Rejection | - | - | - | - | Priority 0 |
| P1-1 | Watchdog 1.5s | - | - | - | - | Target: 15-17% |
| P1-2 | Watchdog 2.0s | - | - | - | - | Target: 11-13% |
| P1-3 | Watchdog 2.5s | - | - | - | - | Target: 9-11% |
| P1-4 | Low 3D threshold 50 | - | - | - | - | Target: -3-5% |
| P1-5 | Stereo time 3.0s | - | - | - | - | Target: +2-3% |
| P2-1 | Parallax 25° | - | - | - | - | Target: +2-4% |
| P2-2 | Parallax 40° | - | - | - | - | Target: -2-3% |
| P2-3 | Frame count 15 | - | - | - | - | Target: -1-2% |
| P3-1 | Conservative config | - | - | - | - | Target: 10-12% |
| P3-2 | Aggressive config | - | - | - | - | Target: 8-10% |
| P3-3 | No High 3D + WD 2.0s | - | - | - | - | Target: 12-15% |

---

## Decision Criteria

### Success Definition:
- Keyframe rate in 10-15% range
- No tracking failures or crashes
- Processing time < 30ms/frame (real-time)
- Exit code 0 (clean shutdown)

### Optimal Configuration Selection:
After all tests, select configuration that:
1. **Primary:** Keyframe rate closest to 12.5% (center of target range)
2. **Secondary:** Lowest ATE (if evaluation run)
3. **Tertiary:** Most stable (no tracking failures)

### If Multiple Configurations Succeed:
Choose most conservative (highest thresholds) for robustness across different datasets.

### If No Configuration Succeeds:
- If all rates too high (>15%): Test watchdog 3.0s
- If all rates too low (<10%): Test watchdog 1.2s or re-enable High 3D rejection
- If tracking failures: Reduce thresholds (more permissive)

---

## Next Steps After Phase 3.1

1. **Complete all tests** (3-4 hours estimated)
2. **Analyze results** - Identify optimal configuration
3. **Phase 3.2:** Design final configuration based on test data
4. **Phase 3.3:** Implement chosen configuration
5. **Phase 3.4:** Validate on 5000 frames with trajectory evaluation

---

**Test Plan Status:** READY FOR EXECUTION
**Estimated Time:** 3-4 hours (14 tests × 10-15 min each)
**Priority Order:** P0-1 → P1-1 → P1-2 → P1-3 → P1-4 → P1-5 → P2-1 → P2-2 → P2-3 → P3-1 → P3-2 → P3-3

**Note:** Stop early if a test achieves 10-15% rate with no issues. That configuration can be used directly in Phase 3.3.
