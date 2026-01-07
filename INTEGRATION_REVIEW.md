# Integration Review: EKF Filter with OV²SLAM Systems

**Date:** 2026-01-07
**Review Focus:** Compatibility with Loop Closure, Bundle Adjustment, Keyframe Decisions, Mapper, and Overall SLAM Pipeline
**Reviewer:** Manager Mode Analysis

---

## Executive Summary

The EKF filter integration introduces **several compatibility concerns** with the existing OV²SLAM pipeline. While the validation layer approach is well-designed, there are **potential conflicts** with:

1. **Loop closure pose updates** - Motion model may not be reset after LC corrections
2. **Bundle adjustment feedback** - BA-optimized poses don't update motion model
3. **Keyframe triggering logic** - Validation layer interacts with KF decision timing
4. **Thread safety** - No mutex protection for nav_mode_ state
5. **Velocity state management** - Conflicting velocity sources (IMU vs Vision vs GPS)

**Critical Issues:** 2
**Important Issues:** 3
**Minor Issues:** 4

---

## 1. Loop Closure Compatibility

### Issue 1.1: Motion Model Not Reset After Loop Closure (CRITICAL)

**Location:** `src/loop_closer.cpp`, `include/visual_front_end.hpp`

**Problem:**
When loop closure corrects poses (via `Optimizer::localPoseGraph()` or `Optimizer::fullPoseGraph()`), the `MotionModel` state (`prevTwc_`, `prev_time_`, `prev_velocity_`) is **not reset**. This causes:

- **Pose jumps:** The motion model still has the old `prevTwc_` which is now inconsistent with the corrected pose
- **Velocity errors:** `prev_velocity_` is pre-correction, causing wrong IMU predictions
- **Accumulated drift:** Subsequent predictions start from wrong state

**Code Evidence:**
```cpp
// visual_front_end.cpp:58
void MotionModel::applyMotionModel(Sophus::SE3d &Twc, double time) {
    if( prev_time_ > 0 ) {
        // This check handles LC, but only if Twc != prevTwc_
        if( !(Twc * prevTwc_.inverse()).log().isZero(1.e-5) ) {
            // Might happen in case of LC!
            // So update prevPose to stay consistent
            prevTwc_ = Twc;  // ✓ GOOD: Handles LC pose updates
        }
        double dt = (time - prev_time_);
        Twc = Twc * Sophus::SE3d::exp(log_relT_ * dt);
    }
}
```

**Analysis:**
- The code **does handle LC** for `prevTwc_` (lines 50-55), but only if `Twc != prevTwc_`
- **Missing:** Reset of `prev_velocity_` and `log_relT_` after LC
- **Race condition:** If `applyMotionModel()` is called **before** the loop closure update propagates, the check fails

**Impact:** HIGH
- After loop closure, predictions will be wrong for several frames until convergence
- Can trigger false GPS mode switches due to poor PnP inliers

**Recommendation:**
Add explicit motion model reset after loop closure:
```cpp
// In loop_closer.cpp after pose graph optimization
// OR in ov2slam.cpp after detecting LC completion
if( loop_closure_detected ) {
    pfrontend_->motion_model_.reset();
    // Or at minimum: update prevTwc_ to the corrected pose
}
```

---

### Issue 1.2: Loop Closure May Trigger False GPS Mode (IMPORTANT)

**Location:** `src/visual_front_end.cpp:445-501`

**Problem:**
When loop closure corrects poses, the next frame's PnP may have **temporarily low inliers** (features haven't converged to new pose yet). This can trigger:

```
VISION → GPS transition: inliers < min_inliers_gps_ (e.g., 40 inliers)
```

**Scenario:**
1. Loop closure at frame 1000 corrects pose by 5 meters
2. Frame 1001: Features still project poorly → PnP inliers = 35 (< 50)
3. Validation layer switches to GPS mode
4. **Result:** Overwrites LC-corrected pose with GPS dead reckoning → **undo LC benefit**

**Impact:** MEDIUM
- Loop closure benefits may be lost temporarily
- Increased state transitions (GPS → Vision → GPS → Vision)

**Recommendation:**
- Add **hysteresis after loop closure** (freeze mode switching for N frames)
- OR: Increase `min_inliers_gps_` threshold temporarily after LC
- OR: Use `nav_mode_counter_` more aggressively (require more consecutive failures)

---

## 2. Bundle Adjustment Compatibility

### Issue 2.1: BA-Optimized Poses Don't Update Motion Model (CRITICAL)

**Location:** `src/optimizer.cpp:38` (localBA), `src/estimator.cpp:89`

**Problem:**
After `Optimizer::localBA()` refines the current frame pose, the **motion model is not notified**:

1. Frame tracked with PnP → pose T1
2. Keyframe created → local BA refines pose to T2 (T2 != T1)
3. Motion model still has `prevTwc_ = T1`
4. Next frame prediction starts from wrong state

**Code Evidence:**
```cpp
// estimator.cpp:89-103
void Estimator::processNewKeyframe() {
    // ... setup ...

    // Local BA refines pnewkf_ pose
    poptimizer_->localBA(*pnewkf_, use_robust_cost);  // Line 97

    // ❌ NO: motion_model_.updateMotionModel(pnewkf_->Twc_, time);
    // ❌ NO: motion_model_.updateMotionModelVelocity(...);
}
```

**Data Flow:**
```
trackStereo()
  ├─ computePose() → T_pnp (inliers stored)
  ├─ checkNewKfReq() → TRUE
  └─ return is_kf_req

visualTracking()
  └─ if( is_kf_req ) createKeyframe()  ← Pose still T_pnp here!

Estimator thread (separate):
  processNewKeyframe()
    └─ localBA() → T_ba  ← Pose refined, but motion model not updated!
```

**Impact:** HIGH
- Motion model always lags behind BA-optimized poses
- Creates bias in velocity estimates
- IMU prediction uses stale state

**Root Cause:**
**Threading separation:** Motion model update happens in SLAM Manager thread (`trackStereo()`), but BA happens in Estimator thread. No synchronization point.

**Recommendation:**
1. **Option A:** Update motion model in BA thread after optimization:
   ```cpp
   // estimator.cpp after localBA
   if( poptimizer_->localBA(...) ) {
       // Get refined pose
       Sophus::SE3d refined_pose = pnewkf_->getTwc();
       double kf_time = pnewkf_->img_time_;

       // Update motion model (need thread-safe access)
       pfrontend_->motion_model_.updateMotionModel(refined_pose, kf_time);
   }
   ```

2. **Option B:** Store BA-corrected pose flag, update motion model in next `trackStereo()` call
3. **Option C:** Add mutex to motion model and update from both threads

**Complexity:** Requires thread-safe motion model access (currently unprotected)

---

### Issue 2.2: Velocity State After BA (IMPORTANT)

**Location:** `src/visual_front_end.cpp:244-262`, `src/visual_front_end.cpp:526-533`

**Problem:**
Velocity correction logic assumes **PnP pose is final**, but BA may change the pose later:

```cpp
// trackMono() line 244-262 (VISION mode)
if( !pslamstate_->imu_only_mode_ ) {
    if( motion_model_.prev_time_ > 0 && time > motion_model_.prev_time_ ) {
        Eigen::Vector3d p_cur = pcurframe_->getTwc().translation();  // T_pnp
        Eigen::Vector3d p_prev = motion_model_.prevTwc_.translation();
        double dt = time - motion_model_.prev_time_;
        Eigen::Vector3d v_visual = (p_cur - p_prev) / dt;  // ❌ Wrong if BA changes T_pnp
        pcurframe_->setVelocity(v_visual);
    }
}

// But later: BA refines T_pnp → T_ba
// Velocity should be: (T_ba - T_prev) / dt
// But we already stored: v_visual = (T_pnp - T_prev) / dt
```

**Impact:** MEDIUM
- Velocity bias proportional to BA correction magnitude
- If BA corrects pose by 0.5m over 0.1s → velocity error = 5 m/s
- IMU prediction will drift

**Current Mitigation:**
- Velocity is re-computed in `updateMotionModelVelocity()` (line 270)
- But this uses `pcurframe_->getTwc()`, which may still be T_pnp (BA hasn't run yet)

**Recommendation:**
- Defer velocity computation until after BA completes
- OR: Re-compute velocity after BA in Estimator thread

---

## 3. Keyframe Decision Compatibility

### Issue 3.1: Validation Layer Interacts with KF Timing (IMPORTANT)

**Location:** `src/visual_front_end.cpp:445-541` (trackStereo)

**Problem:**
The validation layer decision (GPS vs Vision) happens **after** `computePose()` but **before** `checkNewKfReq()`:

```cpp
// trackStereo() sequence:
1. computePose() → T_vision, inliers_vision
2. Validation layer → decide GPS vs Vision
   ├─ If GPS: T_gps = getPoseAt(time), v_gps = ...
   └─ If Vision: T_vision (unchanged), v_visual = ...
3. motion_model_.updateMotionModel(T_final, time)
4. checkNewKfReq()  ← Uses T_final for parallax computation
```

**Impact:**
- **GPS mode:** Keyframe decision based on GPS pose, not visual tracking quality
- **Vision mode:** Keyframe decision based on visual pose

**Example Scenario:**
```
Frame 500:
- PnP inliers = 35 (< 50) → Switch to GPS mode
- GPS pose smooth (high quality GPS) → Low parallax with prev KF
- checkNewKfReq() → FALSE (no new KF created)

But:
- Visual tracking was poor (should create KF to triangulate new points)
- GPS mode masked the need for new keyframe
```

**Impact Severity:** MEDIUM
- In GPS mode, keyframes may be created **too infrequently**
- Map density decreases, tracking degrades further
- Potential vicious cycle: poor tracking → GPS mode → fewer KFs → worse tracking

**Recommendation:**
- **Decouple keyframe decision from nav mode**
- Use **PnP inliers count** (before validation) for KF decision, not final pose
- OR: Force KF creation when transitioning between modes

**Code Fix:**
```cpp
// trackStereo() after validation (line 501)
// Store inliers BEFORE validation for KF decision
size_t inliers_for_kf = last_nbinliers_;

// ... validation logic ...

// Keyframe decision: use inliers, not nav mode
bool is_kf_req = checkNewKfReqWithInliers(inliers_for_kf);
```

---

### Issue 3.2: Watchdog Timer Interacts with Mode Switching (MINOR)

**Location:** `src/visual_front_end.cpp:1439-1447`

**Problem:**
The watchdog timer (`last_keyframe_time_`) is checked **before** validation layer runs:

```cpp
// checkNewKfReq() line 1439-1447
if( pslamstate_->stereo_ && last_keyframe_time_ > 0 ) {
    double time_since_last_kf = pcurframe_->img_time_ - last_keyframe_time_;
    if( time_since_last_kf < 1.0 ) {
        return false;  // Reject KF due to watchdog
    }
}
```

**Impact:**
- If GPS mode causes poor tracking for 2 seconds, then switches to Vision mode
- Vision mode tries to create KF → rejected by watchdog (only 0.5s since GPS mode KF)
- But GPS mode KF was created for wrong reasons (pose quality, not tracking need)

**Recommendation:**
- Reset watchdog timer when nav mode changes:
  ```cpp
  if( nav_mode_switched ) {
      last_keyframe_time_ = -1.0;  // Reset watchdog
  }
  ```

---

## 4. Mapper Integration

### Issue 4.1: Stereo Matching Uses Wrong Pose (IMPORTANT)

**Location:** `src/map_manager.cpp` (stereoMatching), `src/visual_front_end.cpp:432-434`

**Problem:**
Stereo matching (`MapManager::stereoMatching()`) is called during `createKeyframe()`, which uses the **current frame pose**:

```cpp
// trackStereo() line 432-434
// NOTE: Stereo matching happens in MapManager::stereoMatching() when keyframes are created
// This uses ZNCC (proper for stereo) instead of KLT (which is for temporal tracking)
// The mapper will automatically match left↔right features and triangulate 3D points
```

**Issue:** If validation layer switches to GPS mode, the pose passed to `createKeyframe()` is the **GPS pose**, not the visual pose:

```cpp
// trackStereo() line 504-523
if( use_gps_mode && gt_loader_ && ... ) {
    Twc.translation() = p_gps_cur;  // GPS translation
    pcurframe_->setTwc(Twc);  // Overwrites visual pose with GPS
}

// Later: createKeyframe() uses this GPS pose
```

**Impact on Stereo Matching:**
- Stereo matching searches for corresponding left-right features
- Uses camera projection based on `Twc` (GPS pose)
- If GPS pose is wrong (high uncertainty), projection is wrong
- Result: **Poor stereo matches**, **few 3D points**

**Code Evidence (MapManager::stereoMatching):**
```cpp
// map_manager.cpp (stereo matching uses frame pose for projection)
// If Twc is GPS-based (high uncertainty in rotation), projection fails
```

**Impact:** MEDIUM
- GPS mode keyframes have fewer stereo 3D points
- Reduces map quality in GPS mode
- Makes recovery to Vision mode harder

**Recommendation:**
- **Always use visual pose for stereo matching**, even in GPS mode
- OR: Don't create keyframes in GPS mode (only create when transitioning back to Vision)

---

### Issue 4.2: Triangulation Uses GPS Pose (MINOR)

**Location:** `src/map_manager.cpp` (triangulate)

**Problem:**
New map points are triangulated from keyframe poses. If GPS mode keyframe has GPS pose, triangulation uses GPS pose.

**Impact:** LOW
- Similar to stereo matching issue
- GPS pose uncertainty affects triangulation quality

**Recommendation:** Same as Issue 4.1

---

## 5. Thread Safety and Race Conditions

### Issue 5.1: nav_mode_ State Not Protected (IMPORTANT)

**Location:** `include/visual_front_end.hpp:170-171`

**Problem:**
The validation layer state (`nav_mode_`, `nav_mode_counter_`) is **not mutex-protected**:

```cpp
// visual_front_end.hpp
enum class NavMode {
    VISION,
    GPS
};
NavMode nav_mode_ = NavMode::VISION;
int nav_mode_counter_ = 0;
```

**Threading Context:**
- **SLAM Manager thread:** Calls `trackStereo()` → modifies `nav_mode_`
- **Loop Closer thread:** No direct access, but...
- **Estimator thread:** No access currently

**Current Safety:**
- Only SLAM Manager thread accesses `nav_mode_` → **currently safe**
- **BUT:** If future code reads `nav_mode_` from other threads (e.g., for visualization), race condition occurs

**Recommendation:**
- Add mutex protection for future-proofing:
  ```cpp
  std::mutex nav_mode_mutex_;
  NavMode getNavMode() const {
      std::lock_guard<std::mutex> lock(nav_mode_mutex_);
      return nav_mode_;
  }
  ```

**Impact:** LOW (currently safe, but fragile)

---

### Issue 5.2: Motion Model Thread Safety (CRITICAL for Future)

**Location:** `include/visual_front_end.hpp:96-103`

**Problem:**
If we implement Issue 2.1 recommendation (update motion model from BA thread), we need mutex protection:

```cpp
// Current: No mutex (only accessed from SLAM Manager thread)
class MotionModel {
    double prev_time_ = -1.;
    Sophus::SE3d prevTwc_;
    Eigen::Matrix<double, 6, 1> log_relT_;
    Eigen::Vector3d prev_velocity_;
    bool has_prev_velocity_;
};
```

**If BA thread updates motion model:**
- **Race:** SLAM Manager thread reads `prev_velocity_` while BA thread writes
- **Result:** Inconsistent state, NaN crashes

**Recommendation:**
- Add `std::mutex motion_model_mutex_` before implementing cross-thread updates
- Use `std::lock_guard` for all access

---

## 6. Data Flow Analysis

### Issue 6.1: Velocity State Confusion (IMPORTANT)

**Location:** Multiple sources of velocity updates

**Problem:**
Velocity is computed/updated from **three different sources**:

1. **IMU prediction** (line 157 in trackMono, line 398 in trackStereo):
   ```cpp
   Eigen::Vector3d v_pred = v_prev + R_prev * dv;
   pcurframe_->setVelocity(v_pred);
   motion_model_.updateMotionModelVelocity(v_pred, true);
   ```

2. **Visual correction** (line 249 in trackMono, line 530 in trackStereo):
   ```cpp
   Eigen::Vector3d v_visual = (p_cur - p_prev) / dt;
   pcurframe_->setVelocity(v_visual);
   ```

3. **GPS dead reckoning** (line 521 in trackStereo):
   ```cpp
   Eigen::Vector3d v_gps = (p_gps_cur - p_gps_prev) / dt;
   pcurframe_->setVelocity(v_gps);
   motion_model_.updateMotionModelVelocity(v_gps, true);
   ```

**Then:** `updateMotionModel()` is called (line 265 in trackMono, line 536 in trackStereo):
```cpp
motion_model_.updateMotionModel(pcurframe_->Twc_, time);
```

**And:** `updateMotionModelVelocity()` is called AGAIN (line 270):
```cpp
motion_model_.updateMotionModelVelocity(vel, true);  // Where does vel come from?
```

**Confusion:**
- Which velocity is used for the next frame's prediction?
- What's the source of `vel` in line 270? → It's `pcurframe_->getVelocity()`, but which update set it?

**Execution Order Analysis (trackStereo):**
```cpp
// Line 362-430: IMU prediction
if( gt_loader_ && has_prev_velocity_ ) {
    // ... preintegrate IMU ...
    v_pred = v_prev + R_prev * dv;
    pcurframe_->setVelocity(v_pred);  #1: Set IMU velocity
    motion_model_.updateMotionModelVelocity(v_pred, true);  #1a: Store IMU velocity
}

// Line 438-443: computePose() (PnP)
if( !imu_only_mode_ ) {
    computePose();  // Sets pcurframe_->Twc_ to visual pose
}

// Line 445-533: Validation layer
if( use_gps_mode ) {
    v_gps = (p_gps_cur - p_gps_prev) / dt;
    pcurframe_->setVelocity(v_gps);  #2: Overwrites with GPS velocity
    motion_model_.updateMotionModelVelocity(v_gps, true);  #2a: Overwrites with GPS velocity
} else {
    v_visual = (p_cur - p_prev) / dt;
    pcurframe_->setVelocity(v_visual);  #2: Overwrites with visual velocity
    // No updateMotionModelVelocity() call here! ❌
}

// Line 536: Update motion model pose
motion_model_.updateMotionModel(pcurframe_->Twc_, time);  #3

// ❌ MISSING: No final updateMotionModelVelocity() call in trackStereo!
// In trackMono() line 270, there's another updateMotionModelVelocity() call
```

**Inconsistency:**
- **trackMono()** (line 268-278): Calls `updateMotionModelVelocity()` **after** `updateMotionModel()`
- **trackStereo()** (line 536): Calls `updateMotionModel()` but **NOT** `updateMotionModelVelocity()`

**Impact:**
- `motion_model_.prev_velocity_` may be stale in stereo mode
- Next frame's IMU prediction uses wrong `v_prev`

**Recommendation:**
- **Unify velocity update logic** between mono and stereo
- Clarify which velocity source is authoritative (IMU vs Visual vs GPS)
- Add final `updateMotionModelVelocity()` call in trackStereo()

---

## 7. Compatibility with Existing Features

### 7.1 Profiling Integration ✓

**Status:** COMPATIBLE

**Analysis:**
- Validation layer doesn't interfere with `ProfiledMutex`
- No new mutexes introduced (nav_mode_ should be protected, see Issue 5.1)

**Recommendation:** None

---

### 7.2 Rerun Visualization ✓

**Status:** COMPATIBLE (if implemented)

**Analysis:**
- Validation layer doesn't change frame structure
- Rerun can log `nav_mode_` state for debugging

**Recommendation:**
- Add Rerun entity for nav mode visualization:
  ```cpp
  #ifdef ENABLE_RERUN
  rec_->log("navigation_mode", rerun::TextLog(nav_mode_ == NavMode::VISION ? "VISION" : "GPS"));
  #endif
  ```

---

### 7.3 IMU Preintegration ✓

**Status:** COMPATIBLE (with caveats)

**Analysis:**
- IMU preintegration code (lines 116-157 in trackMono, lines 366-420 in trackStereo) works independently
- Validation layer runs **after** IMU prediction

**Caveats:**
- GPS mode velocity overwrites IMU-predicted velocity (Issue 6.1)
- BA-optimized poses don't update IMU bias state (Issue 2.1)

**Recommendation:** See Issue 2.1

---

## 8. Configuration and Parameter Interactions

### Issue 8.1: Hysteresis Frames vs IMU Rate (MINOR)

**Location:** `parameters_files/pohang00.yaml:215`

**Problem:**
```yaml
hysteresis_frames: 30  # Frames to wait before switching mode
```

**Issue:**
- At 20 Hz camera, 30 frames = **1.5 seconds**
- At 10 Hz IMU, 1.5 seconds = **15 IMU measurements**
- If GPS mode triggers during fast motion, IMU drift accumulates for 1.5s before switching back

**Impact:** LOW
- May need adjustment based on camera/IMU rates

**Recommendation:**
- Make `hysteresis_frames` time-based, not frame-count-based:
  ```cpp
  double hysteresis_time = 1.0;  // seconds
  double time_in_mode = time - mode_switch_time_;
  if( time_in_mode >= hysteresis_time ) { ... }
  ```

---

### Issue 8.2: min_inliers Thresholds vs Feature Count (MINOR)

**Location:** `parameters_files/pohang00.yaml:213-214`

**Problem:**
```yaml
min_inliers_vision: 80   # Use vision if inliers >= 80
min_inliers_gps: 50      # Switch to GPS if inliers < 50
```

**Issue:**
- If `nbmaxkps_` (max features) = 300, then 80 inliers = 27% success rate
- If `nbmaxkps_` = 100, then 80 inliers = 80% success rate
- Thresholds are **absolute**, not **relative** to feature count

**Impact:** LOW
- May need tuning for different scenes

**Recommendation:**
- Consider relative thresholds (e.g., 30% of tracked features)

---

## Summary of Recommendations

### Critical (Must Fix)

1. **Reset motion model after loop closure** (Issue 1.1)
   - Add `motion_model_.reset()` after pose graph optimization
   - Or update `prevTwc_` to corrected pose

2. **Update motion model after BA** (Issue 2.1)
   - Add thread-safe motion model update after `localBA()`
   - Synchronize between Estimator and SLAM Manager threads

### Important (Should Fix)

3. **Prevent false GPS mode after loop closure** (Issue 1.2)
   - Freeze mode switching for N frames after LC
   - Or increase `min_inliers_gps_` threshold temporarily

4. **Defer velocity computation until after BA** (Issue 2.2)
   - Don't set velocity in `trackStereo()` if BA will refine pose
   - Re-compute velocity after BA in Estimator thread

5. **Decouple keyframe decision from nav mode** (Issue 3.1)
   - Use PnP inliers count (before validation) for KF decision
   - Force KF creation when transitioning between modes

### Minor (Nice to Have)

6. **Reset watchdog on mode switch** (Issue 3.2)
7. **Use visual pose for stereo matching** (Issue 4.1)
8. **Add mutex for nav_mode_** (Issue 5.1)
9. **Unify velocity update logic** (Issue 6.1)
10. **Make hysteresis time-based** (Issue 8.1)

---

## Testing Recommendations

### Test 1: Loop Closure Recovery
```
1. Run sequence with loop closure
2. Verify motion model reset after LC
3. Check nav mode doesn't switch to GPS after LC
```

### Test 2: Bundle Adjustment Feedback
```
1. Create keyframe in GPS mode
2. Verify BA-refined pose updates motion model
3. Check next frame prediction uses refined pose
```

### Test 3: Mode Switching Hysteresis
```
1. Force low inliers for 50 frames
2. Verify mode switches to GPS after hysteresis_frames
3. Restore good tracking
4. Verify mode switches back to Vision after hysteresis
```

### Test 4: Keyframe Creation in GPS Mode
```
1. Switch to GPS mode
2. Verify keyframes are still created based on inliers, not nav mode
3. Check map density doesn't degrade
```

---

## Conclusion

The EKF filter validation layer is **generally well-integrated** but has **thread synchronization issues** with the existing OV²SLAM pipeline. The main concerns are:

1. **Loop closure and BA corrections don't propagate to motion model** - This will cause prediction errors
2. **Keyframe decisions in GPS mode** - May reduce map quality
3. **Velocity state confusion** - Multiple sources, unclear priority

**Priority:**
- **Fix Issue 1.1 and 2.1 before production use** (prevent prediction drift)
- **Address Issue 3.1** (maintain map quality in GPS mode)
- **Monitor Issues 1.2 and 6.1** (may not cause immediate failures)

**Overall Assessment:** The integration is **functional but needs refinement** for robustness in all scenarios.
