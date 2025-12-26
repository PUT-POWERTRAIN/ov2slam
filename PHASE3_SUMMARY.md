# GPS Initialization Phase 3: Testing Infrastructure

## Overview

Phase 3 implements comprehensive testing infrastructure for the GPS initialization system, including the TrajectoryEvaluator class and unit tests using GoogleTest framework.

## Files Created

### 1. `/home/wojtess/Documents/powertrain/ov2slam-standalone/include/trajectory_evaluator.hpp`

Header file for the TrajectoryEvaluator class with the following interface:

- **Constructor**: Loads SLAM and ground truth trajectories
- **computeInitialHeading(int num_frames)**: Computes heading from first N frames (radians)
- **computeGTInitialHeading(int num_frames)**: Computes GT heading from first N frames
- **computeDirectionChanges()**: Returns vector of direction change angles along trajectory
- **estimateScale()**: Estimates scale factor by comparing trajectory lengths
- **computeATE()**: Computes Absolute Trajectory Error vs ground truth
- **computeDirectionConsistency()**: Returns 0-1 metric for trajectory smoothness

### 2. `/home/wojtess/Documents/powertrain/ov2slam-standalone/src/trajectory_evaluator.cpp`

Implementation of all TrajectoryEvaluator methods:

**loadTrajectory()**
- Parses TUM format trajectory files (timestamp x y z qx qy qz qw)
- Handles header comments and validates quaternions
- Loads both poses and timestamps

**computeInitialHeading() / computeGTInitialHeading()**
- Computes average direction vector from first N frames
- Returns heading angle in XY plane using atan2
- Handles edge cases (insufficient motion, not enough poses)

**computeDirectionChanges()**
- Iterates through consecutive pose triplets
- Computes angle between direction vectors
- Returns vector of angles in radians

**estimateScale()**
- Computes total path length for both trajectories
- Returns ratio of SLAM length to GT length
- Expected to be ~1.0 for correctly initialized stereo SLAM

**computeATE()**
- Matches poses by closest timestamps (within 100ms)
- Computes RMS position error
- Returns mean absolute error in meters

**computeDirectionConsistency()**
- Computes average direction change
- Converts to 0-1 metric using exponential decay
- Higher values indicate more consistent (smoother) trajectories

### 3. `/home/wojtess/Documents/powertrain/ov2slam-standalone/test/test_gps_converter.cpp`

Comprehensive GPSConverter unit tests:

- **OriginIsZero**: Verifies origin converts to (0, 0, 0)
- **NorthDisplacement**: Tests ~111m per degree latitude
- **EastDisplacement**: Tests ~100m per degree longitude
- **AltitudeDisplacement**: Tests altitude → Z mapping
- **MultipleConversions**: Verifies consistency
- **SouthernHemisphere**: Tests Sydney coordinates
- **WesternHemisphere**: Tests New York coordinates

### 4. `/home/wojtess/Documents/powertrain/ov2slam-standalone/test/test_direction.cpp`

TrajectoryEvaluator unit tests using synthetic data:

- **InitialHeadingMatches**: Validates heading computation with identical trajectories
- **DirectionConsistency**: Tests straight-line trajectories (expect >0.9 consistency)
- **ScaleEstimation**: Creates trajectories with known scale ratio (0.5x)
- **ATEComputation**: Tests ATE with identical trajectories (expect ~0 error)
- **DirectionChanges**: Validates direction change computation
- **EmptyTrajectory**: Tests graceful handling of empty files
- **SinglePose**: Tests minimal trajectory edge case

### 5. `/home/wojtess/Documents/powertrain/ov2slam-standalone/test/README.md`

Documentation for test infrastructure including:
- Build instructions
- Test suite descriptions
- TrajectoryEvaluator API documentation
- Trajectory file format specification
- Guidelines for adding new tests

## Build System Updates

### CMakeLists.txt Changes

**Added to main library:**
```cmake
src/trajectory_evaluator.cpp
```

**Added test infrastructure (lines 226-265):**
- `ENABLE_TESTS` option (OFF by default)
- GoogleTest v1.14.0 via FetchContent
- Test executable `gps_tests` linking against library
- Automatic test discovery with `gtest_discover_tests`
- Status message when tests enabled

### build.sh Updates

**Added support for ENABLE_TESTS:**
```bash
ENABLE_TESTS=ON ./build.sh
```

**Features:**
- Passes ENABLE_TESTS to CMake
- Automatically runs tests via CTest after build
- Displays manual test execution instructions

## Trajectory File Format

Tests expect TUM format trajectories:
```
# timestamp tx ty tz qx qy qz qw
1625124349.169322968 0.000000000 0.000000000 0.000000000 0.000000000 0.000000000 0.000000000 1.000000000
1625124349.269270897 -0.004385955 -0.000373339 -0.098481393 0.000082965 0.000687113 -0.000389859 0.999999685
```

This matches the output format of OV²SLAM:
- `ov2slam_trajectory.txt`: Visual odometry trajectory
- `ov2slam_keyframes.txt`: Keyframe poses
- `ov2slam_full_trajectory.txt`: Loop-closure optimized trajectory

## Usage Examples

### Building with Tests
```bash
ENABLE_TESTS=ON ./build.sh
```

### Running Tests Manually
```bash
cd build
./gps_tests
```

### Using TrajectoryEvaluator in Code
```cpp
#include "trajectory_evaluator.hpp"

TrajectoryEvaluator eval("ov2slam_trajectory.txt", "gt_trajectory.txt");

double slam_heading = eval.computeInitialHeading(10);
double gt_heading = eval.computeGTInitialHeading(10);
double heading_error = std::abs(slam_heading - gt_heading);

double scale = eval.estimateScale();
double ate = eval.computeATE();
double consistency = eval.computeDirectionConsistency();

std::cout << "Heading error: " << (heading_error * 180.0 / M_PI) << " deg\n";
std::cout << "Scale: " << scale << " (expected ~1.0)\n";
std::cout << "ATE: " << ate << "m\n";
std::cout << "Direction consistency: " << consistency << "\n";
```

## Testing Strategy

### Synthetic Data Approach

Tests use synthetic trajectory data generated at runtime to:
- Avoid dependency on external dataset files
- Ensure tests are reproducible and fast
- Validate all code paths with known inputs

### GPSConverter Tests

Validate coordinate transformations using:
- Known geographic locations (Pohang, Sydney, New York)
- Expected displacement values (111m/deg latitude, 100m/deg longitude)
- Hemisphere edge cases

### TrajectoryEvaluator Tests

Validate metrics using:
- Straight-line trajectories (known high consistency)
- Scaled trajectories (known scale ratios)
- Identical trajectories (known zero error)
- Edge cases (empty, single-pose)

## Design Decisions

1. **Eigen Alignment**: All classes use `EIGEN_MAKE_ALIGNED_OPERATOR_NEW` for proper Eigen alignment

2. **Error Handling**: Graceful degradation for edge cases (empty files, single poses)

3. **Metric Design**:
   - Scale: Simple length ratio (SLAM/GT)
   - ATE: Standard RMS position error
   - Consistency: Exponential decay from average direction change

4. **Test Independence**: Tests create temporary files in `/tmp/` and clean up after themselves

5. **Minimal Dependencies**: Tests don't require GeographicLib for TrajectoryEvaluator tests (only GPSConverter tests)

## Future Enhancements

Potential improvements for future phases:

1. **Rotation Error Metrics**: Add angular error computation (degrees/radians)

2. **Drift Analysis**: Compute positional drift over time

3. **Loop Closure Validation**: Test trajectory consistency before/after loop closure

4. **Real Dataset Tests**: Add integration tests using actual Pohang dataset

5. **Performance Benchmarks**: Add timing tests for trajectory evaluation

6. **Visualization**: Create trajectory comparison plots (matplotlib/Python)

7. **Scale Refinement**: Implement Umeyama alignment for better scale estimation

## Integration with GPS Initialization

The TrajectoryEvaluator provides validation metrics for Phase 4 (GPS initialization):

1. **Heading Validation**: `computeInitialHeading()` verifies SLAM heading matches GPS heading

2. **Scale Validation**: `estimateScale()` confirms metric scale (should be ~1.0 for stereo)

3. **Quality Assessment**: `computeDirectionConsistency()` measures trajectory smoothness

4. **Accuracy Measurement**: `computeATE()` provides absolute error metrics

These metrics will be used to:
- Validate GPS-based scale correction
- Detect initialization failures
- Compare performance with/without GPS initialization
- Generate quality reports

## Files Summary

| File | Lines | Purpose |
|------|-------|---------|
| `include/trajectory_evaluator.hpp` | 62 | Header defining TrajectoryEvaluator interface |
| `src/trajectory_evaluator.cpp` | 292 | Implementation of trajectory evaluation metrics |
| `test/test_gps_converter.cpp` | 97 | GPS coordinate conversion tests |
| `test/test_direction.cpp` | 184 | Trajectory evaluation tests |
| `test/README.md` | 108 | Test documentation |
| `CMakeLists.txt` (modified) | +48 | Build system updates for testing |
| `build.sh` (modified) | +18 | Test execution support |
| **Total** | **809** | Phase 3 implementation |

## Next Steps

Phase 4 will integrate these components into the SLAM pipeline:
1. Modify SlamManager to accept GPS data
2. Implement scale correction from GPS trajectory
3. Add heading initialization from GPS
4. Integrate TrajectoryEvaluator for validation
5. Add GPS initialization to YAML configuration
