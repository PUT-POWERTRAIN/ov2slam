# OV²SLAM Unit Tests

This directory contains unit tests for the GPS initialization functionality.

## Building Tests

To build the tests, use the build script with the `ENABLE_TESTS` flag:

```bash
ENABLE_TESTS=ON ./build.sh
```

This will:
1. Download GoogleTest (v1.14.0) via CMake FetchContent
2. Build the test executable (`gps_tests`)
3. Run all tests automatically

## Running Tests Manually

After building with tests enabled, you can run tests manually:

```bash
cd build
./gps_tests
```

Or using CTest:

```bash
cd build
ctest --output-on-failure
```

## Test Suites

### GPS Converter Tests (`test_gps_converter.cpp`)

Tests the GPS coordinate conversion functionality:

- **OriginIsZero**: Verifies that the origin point converts to (0, 0, 0)
- **NorthDisplacement**: Tests northward displacement (~111m per degree)
- **EastDisplacement**: Tests eastward displacement (~100m per degree at this latitude)
- **AltitudeDisplacement**: Tests altitude changes map directly to Z
- **MultipleConversions**: Ensures repeated conversions are consistent
- **SouthernHemisphere**: Tests conversion in southern hemisphere (Sydney)
- **WesternHemisphere**: Tests conversion in western hemisphere (New York)

These tests validate the GeographicLib-based WGS84 to ENU coordinate conversion.

### Trajectory Direction Tests (`test_direction.cpp`)

Tests the trajectory evaluation functionality:

- **InitialHeadingMatches**: Compares SLAM and GT initial heading angles
- **DirectionConsistency**: Measures trajectory smoothness (0-1 metric)
- **ScaleEstimation**: Validates scale estimation between trajectories
- **ATEComputation**: Tests Absolute Trajectory Error calculation
- **DirectionChanges**: Analyzes direction changes along trajectory
- **EmptyTrajectory**: Tests graceful handling of empty trajectories
- **SinglePose**: Tests handling of minimal trajectories

These tests create synthetic trajectory data to validate the evaluation metrics without requiring actual dataset files.

## TrajectoryEvaluator Class

The `TrajectoryEvaluator` class provides metrics for comparing SLAM trajectories against ground truth:

### Constructor
```cpp
TrajectoryEvaluator(const std::string& slam_file, const std::string& gt_file);
```

### Methods

- **computeInitialHeading(int num_frames)**: Returns initial heading (radians) from first N frames
- **computeGTInitialHeading(int num_frames)**: Returns GT initial heading (radians)
- **computeDirectionChanges()**: Returns vector of direction change angles
- **estimateScale()**: Returns scale factor (SLAM length / GT length)
- **computeATE()**: Returns Absolute Trajectory Error in meters
- **computeDirectionConsistency()**: Returns 0-1 consistency metric

### Trajectory File Format

Tests expect trajectory files in TUM format:
```
# timestamp tx ty tz qx qy qz qw
1625124349.169322968 0.000000000 0.000000000 0.000000000 0.000000000 0.000000000 0.000000000 1.000000000
1625124349.269270897 -0.004385955 -0.000373339 -0.098481393 0.000082965 0.000687113 -0.000389859 0.999999685
...
```

Where:
- `timestamp`: Unix timestamp (seconds)
- `tx ty tz`: Position in meters
- `qx qy qz qw`: Quaternion rotation (Hamilton convention, qw is scalar)

## Adding New Tests

To add a new test:

1. Create a new `.cpp` file in `test/`
2. Include `gtest/gtest.h` and necessary headers
3. Use `TEST(TestSuite, TestName)` macro
4. Add the file to `CMakeLists.txt` in the test executable:
   ```cmake
   add_executable(gps_tests
       test/test_gps_converter.cpp
       test/test_direction.cpp
       test/your_new_test.cpp  # Add here
   )
   ```

## Dependencies

Tests require:
- GoogleTest (auto-downloaded via FetchContent)
- Eigen3
- Sophus
- GeographicLib (for GPSConverter tests)

## Notes

- Tests create temporary trajectory files in `/tmp/` for synthetic data
- GPSConverter tests require `ENABLE_GPS_INIT` to be defined (GeographicLib available)
- Tests are designed to be fast and should complete in under 10 seconds
