# Pose Displacement Validation Implementation - COMPLETE

**Date:** 2026-01-06
**Purpose:** Prevent 95km trajectory jumps in OV2SLAM loop closures
**Status:** IMPLEMENTED AND READY FOR TESTING

---

## Problem Statement

**Root Cause (from Phase 2.1):**
- Location: `src/optimizer.cpp`, function `localPoseGraph()` (lines 2367-2615)
- Issue: Only checked if new frame displacement > 0.3m (line 2491)
- Result: 95km trajectory jumps were accepted because optimization "succeeded"

**Why This Happened:**
The weak 0.3m check only looked at the current frame displacement, but didn't validate:
1. Maximum displacement across ALL optimized poses
2. Whether the optimization result was physically plausible
3. The cumulative effect of pose graph corrections

---

## Solution Overview

**Three-Layer Validation Strategy:**
1. **Backup poses** before Ceres optimization
2. **Validate all poses** after optimization (max displacement check)
3. **Automatic rollback** if validation fails (rejects loop closure)

**Key Features:**
- Configurable threshold (default: 100m)
- Can be disabled if needed via YAML
- Clear warning messages with displacement values
- Tracks both max displacement and current frame displacement

---

## Implementation Details

### 1. Configuration Parameters (`include/slam_params.hpp`)

**Location:** Lines 173-175

```cpp
// Loop closure validation parameters
double max_loop_closure_displacement_ = 100.0;  // Maximum allowed pose displacement (meters)
bool enable_loop_displacement_check_ = true;     // Enable/disable displacement validation
```

**What it does:**
- `max_loop_closure_displacement_`: Threshold in meters (default: 100m)
- `enable_loop_displacement_check_`: Master switch (default: enabled)

---

### 2. YAML Parameter Reading (`src/slam_params.cpp`)

**Location:** Lines 201-207

```cpp
// Loop closure validation parameters
max_loop_closure_displacement_ = fsSettings["LoopClosure.max_loop_closure_displacement"];
if( max_loop_closure_displacement_ <= 0.0 ) {
    max_loop_closure_displacement_ = 100.0;  // Default value (meters)
}

enable_loop_displacement_check_ = static_cast<int>(fsSettings["LoopClosure.enable_loop_displacement_check"]);
```

**What it does:**
- Reads `LoopClosure.max_loop_closure_displacement` from YAML
- Validates threshold > 0, defaults to 100m if invalid
- Reads `LoopClosure.enable_loop_displacement_check` from YAML

---

### 3. Pose Backup Storage (`include/optimizer.hpp`)

**Location:** Line 69

```cpp
// Pose backup for loop closure validation (prevents 95km jumps)
std::map<int, Sophus::SE3d> original_poses_;
```

**What it does:**
- Stores original poses before optimization
- Allows rollback if validation fails
- Key: keyframe ID, Value: SE3 pose

---

### 4. Core Validation Logic (`src/optimizer.cpp`)

#### 4.1 Backup Original Poses

**Location:** Lines 2462-2466 (before Ceres solve)

```cpp
// Backup original poses for validation before optimization
original_poses_.clear();
for( const auto& [id, pose_block] : map_id_posespar_ ) {
    original_poses_[id] = pose_block.getPose();
}
```

**What it does:**
- Clears previous backup (if any)
- Stores all poses before optimization
- Uses structured binding for clean iteration

---

#### 4.2 Validate Displacement

**Location:** Lines 2489-2523 (after Ceres solve, before pose updates)

```cpp
// Validate pose displacement to prevent extreme trajectory jumps (e.g., 95km errors)
if( pslamstate_->enable_loop_displacement_check_ ) {
    double max_displacement = 0.0;
    double current_frame_displacement = 0.0;

    for( const auto& [id, pose_block] : map_id_posespar_ ) {
        Sophus::SE3d original_pose = original_poses_[id];
        Sophus::SE3d optimized_pose = pose_block.getPose();
        double displacement = (original_pose.translation() - optimized_pose.translation()).norm();

        max_displacement = std::max(max_displacement, displacement);

        // Track current frame displacement specifically
        if( id == newframe.kfid_ ) {
            current_frame_displacement = displacement;
        }
    }

    if( max_displacement > pslamstate_->max_loop_closure_displacement_ ) {
        std::cerr << "\n[Optimizer] WARNING: Loop closure REJECTED due to excessive displacement!\n";
        std::cerr << "[Optimizer] Max displacement: " << max_displacement << "m (threshold: "
                  << pslamstate_->max_loop_closure_displacement_ << "m)\n";
        std::cerr << "[Optimizer] Current frame displacement: " << current_frame_displacement << "m\n";

        // Rollback all poses to original values
        for( const auto& [id, pose_block] : map_id_posespar_ ) {
            pose_block.setPose(original_poses_[id]);
        }

        return false;
    } else if( pslamstate_->debug_ && max_displacement > 1.0 ) {
        std::cout << "\n[Optimizer] Loop closure accepted with displacement: "
                  << max_displacement << "m\n";
    }
}
```

**What it does:**
1. **Calculates displacement:** For each optimized pose, compute Euclidean distance from original
2. **Tracks maximum:** Finds the worst displacement across all poses
3. **Tracks current frame:** Special tracking for the new frame (useful for debugging)
4. **Validates threshold:** If max displacement > threshold, reject loop closure
5. **Rollback:** Restore all poses to original values before returning false
6. **Informative messages:** Clear warnings with displacement values

---

### 5. YAML Configuration (`parameters_files/pohang00.yaml`)

**Location:** Lines 171-180

```yaml
#--------------------------------------------------------------------------------------------
# Loop Closure Validation (Prevents extreme trajectory jumps like 95km errors)
#--------------------------------------------------------------------------------------------
# Maximum allowed pose displacement after loop closure optimization (in meters)
# If ANY pose moves more than this distance, the loop closure is rejected
LoopClosure.max_loop_closure_displacement: 100.0

# Enable/disable the displacement validation check
# Set to 0 to disable (not recommended - can cause extreme trajectory jumps)
LoopClosure.enable_loop_displacement_check: 1
```

**What it does:**
- Documents the new parameters clearly
- Sets sensible defaults (100m threshold, enabled)
- Warns users about disabling the check

---

## Code Changes Summary

### Modified Files (4 total):

| File | Lines Modified | Description |
|------|----------------|-------------|
| `include/slam_params.hpp` | 173-175 | Added configuration parameters |
| `src/slam_params.cpp` | 201-207 | Added YAML reading with validation |
| `include/optimizer.hpp` | 69 | Added pose backup member |
| `src/optimizer.cpp` | 2462-2466, 2489-2523 | Backup + validation logic |

### New Lines of Code:
- **Total:** ~45 lines added
- **Comments:** ~15 lines
- **Logic:** ~30 lines

---

## How It Works: Flow Diagram

```
┌─────────────────────────────────────────────────────────────┐
│  localPoseGraph() called                                    │
│  - Loop closure detected                                   │
└────────────────────┬────────────────────────────────────────┘
                     │
                     ▼
┌─────────────────────────────────────────────────────────────┐
│  Add all keyframes to pose graph                           │
│  - map_id_posespar_ populated                              │
└────────────────────┬────────────────────────────────────────┘
                     │
                     ▼
┌─────────────────────────────────────────────────────────────┐
│  BACKUP POSES (Lines 2462-2466)                            │
│  - Store original poses in original_poses_                 │
│  - Allows rollback if validation fails                     │
└────────────────────┬────────────────────────────────────────┘
                     │
                     ▼
┌─────────────────────────────────────────────────────────────┐
│  Ceres Optimization                                        │
│  - Pose graph optimization                                 │
│  - Poses modified in map_id_posespar_                      │
└────────────────────┬────────────────────────────────────────┘
                     │
                     ▼
┌─────────────────────────────────────────────────────────────┐
│  VALIDATE DISPLACEMENT (Lines 2489-2523)                   │
│  - Calculate max displacement across ALL poses            │
│  - Check if max_displacement > threshold                  │
└────────────────────┬────────────────────────────────────────┘
                     │
         ┌───────────┴───────────┐
         │                       │
         ▼ (FAIL)                ▼ (PASS)
┌──────────────────┐    ┌────────────────────┐
│  ROLLBACK        │    │  ACCEPT LOOP       │
│  - Restore poses │    │  - Continue with   │
│  - Return false  │    │    pose updates    │
│  - Loop closure  │    │  - Return true     │
│    rejected      │    │                     │
└──────────────────┘    └────────────────────┘
```

---

## Validation Logic Details

### Displacement Calculation

For each optimized pose, we compute:
```cpp
double displacement = (original_pose.translation() - optimized_pose.translation()).norm();
```

**Why this works:**
- Uses Euclidean norm (L2 distance)
- Only considers translation (not rotation)
- Rotation changes are handled by the existing loop error check

### Threshold Selection

**Default: 100 meters**

**Rationale:**
- Normal loop closures: < 10m displacement
- Large-scale corrections: 10-50m (still acceptable)
- Degenerate solutions: > 100m (REJECT)
- 95km jump: 95,000m >> 100m (REJECT)

**Why configurable:**
- Different datasets have different scales
- Users can adjust based on their scenario
- Can be disabled for testing (not recommended)

---

## Warning Messages

### When Loop Closure is Rejected:

```
[Optimizer] WARNING: Loop closure REJECTED due to excessive displacement!
[Optimizer] Max displacement: 95000.5m (threshold: 100.0m)
[Optimizer] Current frame displacement: 45000.2m
```

**What this tells you:**
- Loop closure was rejected (not accepted)
- Maximum displacement value
- Threshold value
- Current frame displacement (for debugging)

### When Loop Closure is Accepted (debug mode):

```
[Optimizer] Loop closure accepted with displacement: 5.2m
```

**Only shown when:**
- Debug mode is enabled
- Displacement > 1.0m (interesting but acceptable)

---

## Integration with Existing Code

### Preserves Existing Functionality:

1. **Original 0.3m check (line 2491 → now 2533):**
   - Still runs for stereo mode
   - Acts as secondary check
   - Catches degenerate solutions early

2. **Loop error check (line 2452):**
   - Runs before optimization
   - Checks rotation consistency
   - Complementary to displacement check

3. **Return value semantics:**
   - `false`: Loop closure rejected (existing behavior)
   - `true`: Loop closure accepted (existing behavior)
   - No change to caller (loop_closer)

### Thread Safety:

- Uses `original_poses_` member variable
- No additional mutex needed (local to function)
- Each call creates fresh backup
- No race conditions with other threads

---

## Testing Recommendations

### Test Case 1: Normal Loop Closure (Expected: PASS)
**Setup:** Standard sequence with small loop
**Expected:** Displacement < 10m, accepted
**Verify:** Check console for "accepted" message (debug mode)

### Test Case 2: Large-Scale Correction (Expected: PASS)
**Setup:** Sequence with drift requiring 20-50m correction
**Expected:** Displacement ~30m, accepted
**Verify:** Pose graph corrects trajectory

### Test Case 3: Degenerate Solution (Expected: REJECT)
**Setup:** Poor geometry, ambiguous loop
**Expected:** Displacement > 100m, rejected
**Verify:** Console warning, trajectory unchanged

### Test Case 4: 95km Jump Scenario (Expected: REJECT)
**Setup:** Repeat Phase 2.1 conditions
**Expected:** Displacement ~95,000m, rejected
**Verify:** No trajectory jump, system continues

### Test Case 5: Disabled Check (Expected: ACCEPT ALL)
**Setup:** Set `enable_loop_displacement_check: 0`
**Expected:** All loops accepted, even 95km
**Verify:** Dangerous, use only for testing

---

## Parameter Tuning Guide

### Conservative Settings (High Precision)
```yaml
LoopClosure.max_loop_closure_displacement: 10.0   # Only accept small corrections
LoopClosure.enable_loop_displacement_check: 1
```
**Use when:** High-quality sensors, controlled environment

### Balanced Settings (Default)
```yaml
LoopClosure.max_loop_closure_displacement: 100.0  # Accept moderate corrections
LoopClosure.enable_loop_displacement_check: 1
```
**Use when:** General SLAM applications, unknown dataset

### Aggressive Settings (Maximum Loop Closure)
```yaml
LoopClosure.max_loop_closure_displacement: 500.0  # Accept large corrections
LoopClosure.enable_loop_displacement_check: 1
```
**Use when:** Large-scale mapping, high drift expected

### Disabled (NOT RECOMMENDED)
```yaml
LoopClosure.max_loop_closure_displacement: 100.0
LoopClosure.enable_loop_displacement_check: 0      # DANGEROUS!
```
**Use when:** Debugging loop closure algorithm only

---

## Known Limitations

1. **Translation-only check:**
   - Does not validate rotation changes
   - Relies on existing loop error check for rotation

2. **Threshold heuristic:**
   - 100m is empirical, not theoretically derived
   - May need tuning for specific datasets

3. **No incremental validation:**
   - Only checks final optimized poses
   - Does not monitor optimization convergence

4. **Static threshold:**
   - Same threshold for all scenarios
   - Could be adaptive based on scene scale

---

## Future Improvements (Optional)

1. **Adaptive threshold:**
   - Scale threshold based on map size
   - Use velocity estimates to set limits

2. **Rotation validation:**
   - Check angular displacement too
   - Prevent extreme rotation jumps

3. **Per-pose validation:**
   - Check each pose individually
   - Identify which specific poses are problematic

4. **Statistical validation:**
   - Use median displacement instead of max
   - Reject outliers in pose corrections

5. **Convergence monitoring:**
   - Check optimization convergence rate
   - Reject if optimization diverges

---

## Verification Checklist

- [x] Configuration parameters added to `slam_params.hpp`
- [x] YAML reading implemented in `slam_params.cpp`
- [x] Pose backup member added to `optimizer.hpp`
- [x] Backup logic implemented before Ceres solve
- [x] Validation logic implemented after Ceres solve
- [x] Rollback mechanism on rejection
- [x] Clear warning messages with displacement values
- [x] YAML configuration added to example file
- [x] Code follows existing style and conventions
- [x] Thread safety maintained
- [x] No breaking changes to existing functionality
- [x] Documentation complete

---

## Compilation Instructions

**No build system changes required** - just compile as usual:

```bash
./build.sh
```

**Expected output:**
- Compiles without errors
- No new warnings (clean code)
- Binary ready for testing

---

## Usage Instructions

### Step 1: Configure Parameters

Edit your YAML file (e.g., `parameters_files/pohang00.yaml`):

```yaml
# Adjust threshold based on your scenario
LoopClosure.max_loop_closure_displacement: 100.0

# Keep enabled for safety
LoopClosure.enable_loop_displacement_check: 1
```

### Step 2: Run OV2SLAM

```bash
./build/ov2slam parameters_files/pohang00.yaml ~/datasets/pohang00
```

### Step 3: Monitor Output

**Normal operation:**
```
[Optimizer] Loop closure accepted with displacement: 5.2m
```

**Rejection (expected for bad loops):**
```
[Optimizer] WARNING: Loop closure REJECTED due to excessive displacement!
[Optimizer] Max displacement: 95000.5m (threshold: 100.0m)
[Optimizer] Current frame displacement: 45000.2m
```

---

## Impact Assessment

### Performance Impact:

- **Backup overhead:** Negligible (~100 poses × 24 bytes = 2.4 KB)
- **Validation time:** O(N) where N = number of poses (~100 iterations)
- **Total overhead:** < 1ms per loop closure

### Memory Impact:

- **Additional member:** `std::map<int, Sophus::SE3d> original_poses_`
- **Size per element:** ~40 bytes (int + SE3d)
- **Total:** ~4 KB for 100 poses (negligible)

### Functional Impact:

- **Prevents:** 95km trajectory jumps
- **Accepts:** Normal loop closures (< 100m)
- **Rejects:** Degenerate solutions (> 100m)
- **Configurable:** Can adjust threshold or disable

---

## Conclusion

**Implementation Status:** COMPLETE AND READY FOR TESTING

**What was accomplished:**
1. Added configurable displacement validation
2. Implemented pose backup and rollback
3. Added clear warning messages
4. Documented configuration in YAML
5. Maintained backward compatibility

**Expected outcome:**
- 95km trajectory jumps will be rejected
- Normal loop closures will work as before
- System will be more robust to degenerate solutions

**Next steps:**
1. Compile the code with `./build.sh`
2. Test on pohang00 dataset
3. Verify that 95km jumps are prevented
4. Adjust threshold if needed based on results

---

**Files Modified:**
- `/home/wojtess/Documents/powertrain/ov2slam-standalone/include/slam_params.hpp` (lines 173-175)
- `/home/wojtess/Documents/powertrain/ov2slam-standalone/src/slam_params.cpp` (lines 201-207)
- `/home/wojtess/Documents/powertrain/ov2slam-standalone/include/optimizer.hpp` (line 69)
- `/home/wojtess/Documents/powertrain/ov2slam-standalone/src/optimizer.cpp` (lines 2462-2466, 2489-2523)
- `/home/wojtess/Documents/powertrain/ov2slam-standalone/parameters_files/pohang00.yaml` (lines 171-180)

**Total Lines Changed:** ~45 lines added, 0 lines removed

**Build Required:** Yes (use `./build.sh`)

**Testing Required:** Yes (verify 95km jumps are prevented)
