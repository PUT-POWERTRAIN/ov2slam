# IMU Preintegration Unit Test Specification

## Document Overview

**Purpose:** This document specifies the unit test suite for the `IMUPreintegration` class implementation.

**Target Component:** `include/core/imu_preintegration.hpp` and `src/core/imu_preintegration.cpp`

**Test Framework:** Google Test (gtest)

**Dependencies:**
- Eigen3 (linear algebra, quaternions)
- Sophus (SO(3) Lie group operations)
- C++17 standard library

**Numerical Precision:** Double precision (64-bit floating point)

---

## Test Strategy

### Testing Philosophy

The IMU preintegration tests verify mathematical correctness of the inertial measurement integration equations. Tests are designed to:

1. **Validate kinematic equations** under controlled conditions
2. **Verify bias correction** accuracy (first-order approximation)
3. **Test edge cases** (zero input, small rotations)
4. **Ensure numerical stability** across realistic motion scenarios

### Test Categories

| Category | Tests | Purpose |
|----------|-------|---------|
| Basic Integration | 1, 3, 4 | Validate core kinematic equations |
| Bias Handling | 2 | Verify first-order bias correction |
| Combined Motion | 5 | Test realistic multi-axis motion |

### Numerical Tolerances

| Quantity | Absolute Tolerance | Relative Tolerance | Rationale |
|----------|-------------------|-------------------|-----------|
| Velocity | 1e-2 m/s | 1% | Double precision integration |
| Position | 1e-2 m | 1% | Accumulated error over time |
| Rotation | 1e-2 rad | 1% | Quaternion normalization |
| Time | 1e-6 s | N/A | Direct computation |
| Zero values | 1e-6 | N/A | Numerical precision limit |

---

## Test Suite 1: Constant Acceleration Integration

### Test ID: TEST_F(IMUPreintegrationTest, ConstantAcceleration)

#### Purpose
Verify kinematic equations for motion under constant acceleration in the absence of rotation.

#### Test Setup
```cpp
// Initial conditions
IMUPreintegration preint;
Eigen::Vector3d initial_bias_gyro(0.0, 0.0, 0.0);  // rad/s
Eigen::Vector3d initial_bias_accel(0.0, 0.0, 0.0); // m/s²

// Integration parameters
const double dt = 0.005;  // 5ms (200 Hz IMU)
const int n_steps = 200;  // Total time: 1.0 second

// IMU measurements
Eigen::Vector3d accel_meas(0.0, 0.0, 9.81);  // m/s² (gravity)
Eigen::Vector3d gyro_meas(0.0, 0.0, 0.0);    // rad/s (no rotation)
```

#### Input Data
| Step | Acceleration (m/s²) | Angular Velocity (rad/s) | Delta Time (s) |
|------|---------------------|--------------------------|----------------|
| 0-199 | (0, 0, 9.81) | (0, 0, 0) | 0.005 |

#### Expected Results
```cpp
// Analytical solution (constant acceleration)
double t = 1.0;  // Total integration time

// Velocity: v = a * t
Eigen::Vector3d expected_delta_v(0.0, 0.0, 9.81 * t);  // (0, 0, 9.81) m/s

// Position: p = 0.5 * a * t²
Eigen::Vector3d expected_delta_p(0.0, 0.0, 0.5 * 9.81 * t * t);  // (0, 0, 4.905) m

// Rotation: R = I (no angular velocity)
Sophus::SO3d expected_delta_R = Sophus::SO3d::exp(Eigen::Vector3d::Zero());

// Time: t = 1.0 s
double expected_delta_t = 1.0;
```

#### Validation Method
```cpp
// Get integration results
Eigen::Vector3d actual_delta_v = preint.getDeltaV();
Eigen::Vector3d actual_delta_p = preint.getDeltaP();
Sophus::SO3d actual_delta_R = preint.getDeltaR();
double actual_delta_t = preint.getDeltaTime();

// Position error
double pos_error = (actual_delta_p - expected_delta_p).norm();
EXPECT_LT(pos_error, 0.01) << "Position error: " << pos_error << " m";

// Velocity error
double vel_error = (actual_delta_v - expected_delta_v).norm();
EXPECT_LT(vel_error, 0.01) << "Velocity error: " << vel_error << " m/s";

// Rotation error (angular distance)
double rot_error = (actual_delta_R.inverse() * expected_delta_R).log().norm();
EXPECT_LT(rot_error, 0.01) << "Rotation error: " << rot_error << " rad";

// Time error
EXPECT_NEAR(actual_delta_t, expected_delta_t, 1e-6);
```

#### Tolerance Specification
- **Position:** ±0.01 m (1% of expected 4.905 m)
- **Velocity:** ±0.01 m/s (0.1% of expected 9.81 m/s)
- **Rotation:** ±0.01 rad (≈0.6°)
- **Time:** ±1e-6 s

#### Edge Cases
- Zero acceleration: verify all deltas remain zero
- Negative acceleration: verify direction is preserved
- Very short integration: test numerical stability with dt → 0

---

## Test Suite 2: Bias Correction Validation

### Test ID: TEST_F(IMUPreintegrationTest, BiasCorrectionFirstOrder)

#### Purpose
Verify that the first-order bias correction approximation matches full re-integration for small bias changes.

#### Test Setup
```cpp
// Step 1: Integrate with initial bias (zero)
IMUPreintegration preint_zero_bias;
Eigen::Vector3d bias_gyro_initial(0.0, 0.0, 0.0);
Eigen::Vector3d bias_accel_initial(0.0, 0.0, 0.0);

const double dt = 0.005;
const int n_steps = 200;

// IMU measurements
Eigen::Vector3d accel_meas(0.0, 0.0, 9.81);
Eigen::Vector3d gyro_meas(0.0, 0.0, 0.0);

for (int i = 0; i < n_steps; ++i) {
    preint_zero_bias.update(accel_meas, gyro_meas, dt);
}

// Step 2: Apply bias correction (small change)
Eigen::Vector3d bias_gyro_new(0.0, 0.0, 0.1);   // δb_g = 0.1 rad/s
Eigen::Vector3d bias_accel_new(0.0, 0.0, 0.5);  // δb_a = 0.5 m/s²

auto corrected = preint_zero_bias.getBiasCorrectedDelta(bias_gyro_new, bias_accel_new);
```

#### Input Data
| Parameter | Initial Bias | New Bias | Change (δb) |
|-----------|--------------|----------|-------------|
| Gyro Z | 0.0 rad/s | 0.1 rad/s | 0.1 rad/s |
| Accel Z | 0.0 m/s² | 0.5 m/s² | 0.5 m/s² |

#### Expected Results
```cpp
// Step 3: Re-integrate with new bias (ground truth)
IMUPreintegration preint_new_bias;
for (int i = 0; i < n_steps; ++i) {
    preint_new_bias.update(accel_meas, gyro_meas, dt);
}
Sophus::SO3d delta_R_new = preint_new_bias.getDeltaR();
Eigen::Vector3d delta_v_new = preint_new_bias.getDeltaV();
Eigen::Vector3d delta_p_new = preint_new_bias.getDeltaP();

// Analytical prediction (first-order correction)
// Δv_corrected = Δv_original + J_v_ba * δb_a
// Δp_corrected = Δp_original + J_p_ba * δb_a + J_p_bg * δb_g
```

#### Validation Method
```cpp
// Compare bias-corrected vs. re-integrated
double vel_error = (corrected.delta_v - delta_v_new).norm();
double pos_error = (corrected.delta_p - delta_p_new).norm();
double rot_error = (corrected.delta_R.inverse() * delta_R_new).log().norm();

// First-order approximation: error should be O(δb²)
// For δb ~ 0.1, error should be < 1%
double vel_rel_error = vel_error / delta_v_new.norm();
double pos_rel_error = pos_error / delta_p_new.norm();

EXPECT_LT(vel_rel_error, 0.01) << "Velocity bias correction error: " << vel_rel_error * 100 << "%";
EXPECT_LT(pos_rel_error, 0.01) << "Position bias correction error: " << pos_rel_error * 100 << "%";
EXPECT_LT(rot_error, 0.01) << "Rotation bias correction error: " << rot_error << " rad";
```

#### Tolerance Specification
- **Velocity:** ±1% relative error
- **Position:** ±1% relative error
- **Rotation:** ±0.01 rad

#### Additional Validation: Large Bias Change
```cpp
// Test with larger bias change (should show second-order errors)
Eigen::Vector3d bias_gyro_large(0.0, 0.0, 1.0);   // 10x larger
Eigen::Vector3d bias_accel_large(0.0, 0.0, 5.0);  // 10x larger

auto corrected_large = preint_zero_bias.getBiasCorrectedDelta(bias_gyro_large, bias_accel_large);

// Re-integrate with large bias
IMUPreintegration preint_large_bias;
for (int i = 0; i < n_steps; ++i) {
    preint_large_bias.update(accel_meas, gyro_meas, dt);
}

// First-order approximation should degrade
double vel_error_large = (corrected_large.delta_v - preint_large_bias.getDeltaV()).norm();
// Expect error ~5% for 10x bias change (second-order effects)
```

---

## Test Suite 3: Zero Input Handling

### Test ID: TEST_F(IMUPreintegrationTest, ZeroInputNoDrift)

#### Purpose
Verify that the preintegration state does not drift when all inputs are zero.

#### Test Setup
```cpp
IMUPreintegration preint;
Eigen::Vector3d zero_accel(0.0, 0.0, 0.0);
Eigen::Vector3d zero_gyro(0.0, 0.0, 0.0);

const double dt = 0.005;
const int n_steps = 200;  // 1.0 second
```

#### Input Data
| Step | Acceleration (m/s²) | Angular Velocity (rad/s) |
|------|---------------------|--------------------------|
| 0-199 | (0, 0, 0) | (0, 0, 0) |

#### Expected Results
```cpp
// All deltas should remain zero
Eigen::Vector3d expected_delta_v = Eigen::Vector3d::Zero();
Eigen::Vector3d expected_delta_p = Eigen::Vector3d::Zero();
Sophus::SO3d expected_delta_R = Sophus::SO3d::exp(Eigen::Vector3d::Zero());
double expected_delta_t = 1.0;
```

#### Validation Method
```cpp
for (int i = 0; i < n_steps; ++i) {
    preint.update(zero_accel, zero_gyro, dt);
}

Eigen::Vector3d actual_delta_v = preint.getDeltaV();
Eigen::Vector3d actual_delta_p = preint.getDeltaP();
Sophus::SO3d actual_delta_R = preint.getDeltaR();
double actual_delta_t = preint.getDeltaTime();

// Strict tolerance: no drift should occur
EXPECT_LT(actual_delta_v.norm(), 1e-6) << "Velocity drift: " << actual_delta_v.norm();
EXPECT_LT(actual_delta_p.norm(), 1e-6) << "Position drift: " << actual_delta_p.norm();

// Rotation: quaternion should remain identity
double rot_error = actual_delta_R.log().norm();
EXPECT_LT(rot_error, 1e-6) << "Rotation drift: " << rot_error << " rad";

// Time should be exact
EXPECT_NEAR(actual_delta_t, 1.0, 1e-10);
```

#### Tolerance Specification
- **Position/Velocity:** ±1e-6 (machine epsilon for double)
- **Rotation:** ±1e-6 rad (numerical precision)
- **Time:** ±1e-10 s

#### Edge Cases
- Very short integration (dt = 1e-6 s): verify no accumulation errors
- Very long integration (dt = 100.0 s): verify numerical stability
- Repeated zero updates: verify state remains consistent

---

## Test Suite 4: Rotation Integration

### Test ID: TEST_F(IMUPreintegrationTest, ConstantAngularVelocity)

#### Purpose
Verify rotation preintegration under constant angular velocity.

#### Test Setup
```cpp
IMUPreintegration preint;
Eigen::Vector3d zero_accel(0.0, 0.0, 0.0);
Eigen::Vector3d gyro_z(0.0, 0.0, 1.0);  // 1 rad/s around Z-axis

const double dt = 0.005;
const int n_steps = 200;  // Total: 1.0 rad rotation
```

#### Input Data
| Step | Angular Velocity (rad/s) | Acceleration (m/s²) |
|------|--------------------------|---------------------|
| 0-199 | (0, 0, 1.0) | (0, 0, 0) |

#### Expected Results
```cpp
// Analytical solution: rotation around Z-axis by 1.0 radian
double expected_angle = 1.0;  // rad
Eigen::Vector3d expected_axis(0.0, 0.0, 1.0);

Sophus::SO3d expected_delta_R = Sophus::SO3d::exp(
    Eigen::Vector3d(0.0, 0.0, expected_angle)
);

Eigen::Vector3d expected_delta_v = Eigen::Vector3d::Zero();
Eigen::Vector3d expected_delta_p = Eigen::Vector3d::Zero();
double expected_delta_t = 1.0;
```

#### Validation Method
```cpp
for (int i = 0; i < n_steps; ++i) {
    preint.update(zero_accel, gyro_z, dt);
}

Sophus::SO3d actual_delta_R = preint.getDeltaR();
Eigen::Vector3d actual_delta_v = preint.getDeltaV();
Eigen::Vector3d actual_delta_p = preint.getDeltaP();

// Extract rotation axis and angle
Eigen::Vector3d rot_log = actual_delta_R.log();
double actual_angle = rot_log.norm();
Eigen::Vector3d actual_axis = rot_log.normalized();

// Verify rotation angle
EXPECT_NEAR(actual_angle, expected_angle, 0.01)
    << "Rotation angle: " << actual_angle << " vs " << expected_angle << " rad";

// Verify rotation axis (Z-axis)
double axis_error = (actual_axis - expected_axis).norm();
EXPECT_LT(axis_error, 0.01) << "Rotation axis error: " << axis_error;

// Verify no translation (zero acceleration)
EXPECT_LT(actual_delta_v.norm(), 1e-6) << "Velocity should be zero";
EXPECT_LT(actual_delta_p.norm(), 1e-6) << "Position should be zero";
```

#### Tolerance Specification
- **Rotation angle:** ±0.01 rad (≈0.6°)
- **Rotation axis:** ±0.01 (unit vector error)
- **Velocity/Position:** ±1e-6 (should be exactly zero)

#### Edge Cases: Multi-Axis Rotation
```cpp
// Test simultaneous rotation around multiple axes
Eigen::Vector3d gyro_multi(0.1, 0.2, 0.3);  // rad/s
const double dt = 0.01;
const int n_steps = 100;  // 1.0 second

// Expected: rotation by ||ω|| * t around normalized ω axis
double expected_angle = gyro_multi.norm() * 1.0;
Eigen::Vector3d expected_axis = gyro_multi.normalized();

Sophus::SO3d expected_delta_R_multi = Sophus::SO3d::exp(
    expected_axis * expected_angle
);

// Verify result matches expected rotation
```

---

## Test Suite 5: Combined Motion

### Test ID: TEST_F(IMUPreintegrationTest, CombinedTranslationAndRotation)

#### Purpose
Verify preintegration with realistic simultaneous translation and rotation.

#### Test Setup
```cpp
IMUPreintegration preint;

// Realistic motion: small rotation + gravity compensation
Eigen::Vector3d accel(0.5, 0.2, 9.81);   // m/s²
Eigen::Vector3d gyro(0.1, 0.0, 0.5);     // rad/s

const double dt = 0.005;
const int n_steps = 20;  // 0.1 seconds
```

#### Input Data
| Step | Acceleration (m/s²) | Angular Velocity (rad/s) | Delta Time (s) |
|------|---------------------|--------------------------|----------------|
| 0-19 | (0.5, 0.2, 9.81) | (0.1, 0.0, 0.5) | 0.005 |

#### Expected Results (Approximate)
```cpp
double t = 0.1;  // Total time

// Rotation: angle = ||ω|| * t
double omega_norm = std::sqrt(0.1*0.1 + 0.0*0.0 + 0.5*0.5);  // ≈0.51 rad/s
double expected_angle = omega_norm * t;  // ≈0.051 rad
Eigen::Vector3d expected_axis = gyro.normalized();  // ≈(0.196, 0.0, 0.981)

// Velocity (first-order, neglecting rotation coupling)
Eigen::Vector3d expected_delta_v_approx = accel * t;  // (0.05, 0.02, 0.981) m/s

// Position (first-order, neglecting rotation coupling)
Eigen::Vector3d expected_delta_p_approx = 0.5 * accel * t * t;  // (0.0025, 0.001, 0.049) m
```

#### Validation Method
```cpp
for (int i = 0; i < n_steps; ++i) {
    preint.update(accel, gyro, dt);
}

Sophus::SO3d actual_delta_R = preint.getDeltaR();
Eigen::Vector3d actual_delta_v = preint.getDeltaV();
Eigen::Vector3d actual_delta_p = preint.getDeltaP();

// Verify rotation
Eigen::Vector3d rot_log = actual_delta_R.log();
double actual_angle = rot_log.norm();
double rot_error = std::abs(actual_angle - expected_angle);
EXPECT_LT(rot_error, 0.01) << "Rotation error: " << rot_error << " rad";

// Verify velocity (allow for rotation-velocity coupling)
double vel_error = (actual_delta_v - expected_delta_v_approx).norm();
double vel_rel_error = vel_error / expected_delta_v_approx.norm();
EXPECT_LT(vel_rel_error, 0.05) << "Velocity error: " << vel_rel_error * 100 << "%";

// Verify position (allow for higher-order terms)
double pos_error = (actual_delta_p - expected_delta_p_approx).norm();
double pos_rel_error = pos_error / expected_delta_p_approx.norm();
EXPECT_LT(pos_rel_error, 0.05) << "Position error: " << pos_rel_error * 100 << "%";
```

#### Tolerance Specification
- **Rotation angle:** ±0.01 rad (±20% for small angle)
- **Velocity:** ±5% relative error
- **Position:** ±5% relative error

#### Validation Against Numerical Integration
```cpp
// Cross-check with high-resolution numerical integration
const double dt_fine = 0.0001;  // 10 kHz
const int n_steps_fine = 1000;  // 0.1 seconds

IMUPreintegration preint_fine;
for (int i = 0; i < n_steps_fine; ++i) {
    preint_fine.update(accel, gyro, dt_fine);
}

// Compare: fine integration should be very close to coarse integration
double vel_conv_error = (preint_fine.getDeltaV() - actual_delta_v).norm();
EXPECT_LT(vel_conv_error, 0.01) << "Convergence error: " << vel_conv_error << " m/s";
```

---

## Test Dependencies

### Required Libraries
```cmake
# CMakeLists.txt additions
find_package(Eigen3 REQUIRED)
find_package(Sophus REQUIRED)
find_package(GTest REQUIRED)

target_link_libraries(test_imu_preintegration
    GTest::gtest
    GTest::gtest_main
    Eigen3::Eigen
    Sophus::Sophus
)
```

### Test File Structure
```
tests/
├── CMakeLists.txt
├── test_main.cpp          # gtest main
├── test_imu_preintegration.cpp  # Test suite
└── test_data/
    └── imu_sequences/     # (optional) ground truth datasets
```

---

## Test Execution Instructions

### Build Tests
```bash
cd /home/wojtess/Documents/powertrain/ov2slam-standalone
mkdir -p build && cd build
cmake .. -DBUILD_TESTS=ON
make test_imu_preintegration
```

### Run All Tests
```bash
./tests/test_imu_preintegration
```

### Run Specific Test
```bash
./tests/test_imu_preintegration --gtest_filter=IMUPreintegrationTest.ConstantAcceleration
```

### Run with Verbose Output
```bash
./tests/test_imu_preintegration --gtest_print_time=1
```

### Generate XML Report (for CI/CD)
```bash
./tests/test_imu_preintegration --gtest_output=xml:test_results.xml
```

---

## Test Implementation Checklist

### Prerequisites
- [ ] IMUPreintegration class header (`include/core/imu_preintegration.hpp`)
- [ ] IMUPreintegration class implementation (`src/core/imu_preintegration.cpp`)
- [ ] Google Test installed and linked

### Implementation Steps
1. [ ] Create test file: `tests/test_imu_preintegration.cpp`
2. [ ] Implement test fixture: `IMUPreintegrationTest`
3. [ ] Implement Test 1: ConstantAcceleration
4. [ ] Implement Test 2: BiasCorrectionFirstOrder
5. [ ] Implement Test 3: ZeroInputNoDrift
6. [ ] Implement Test 4: ConstantAngularVelocity
7. [ ] Implement Test 5: CombinedTranslationAndRotation
8. [ ] Add CMakeLists.txt build configuration
9. [ ] Build and run tests
10. [ ] Verify all tests pass with specified tolerances

### Success Criteria
- All 5 tests pass with specified tolerances
- No memory leaks (run with `valgrind --leak-check=full`)
- Test execution time < 5 seconds
- Code coverage > 90% for `imu_preintegration.cpp`

---

## Appendix: Mathematical Background

### Preintegration Equations

Given IMU measurements `{a_k, ω_k}` at time intervals `Δt`, the preintegrated measurements are:

**Rotation:**
```
ΔR̅_{ij} = ∏_{k=i}^{j-1} Exp((ω_k - b_i^g) Δt)
```

**Velocity:**
```
Δv̅_{ij} = ∑_{k=i}^{j-1} ΔR̅_{ik} (a_k - b_i^a) Δt
```

**Position:**
```
Δp̅_{ij} = ∑_{k=i}^{j-1} [Δv̅_{ik} Δt + 0.5 ΔR̅_{ik} (a_k - b_i^a) Δt²]
```

### Bias Jacobians (First-Order Correction)

**Velocity w.r.t. accel bias:**
```
∂Δv̅_{ij} / ∂b_i^a = -∑_{k=i}^{j-1} ΔR̅_{ik} Δt
```

**Position w.r.t. accel bias:**
```
∂Δp̅_{ij} / ∂b_i^a = -∑_{k=i}^{j-1} [∑_{l=i}^{k} ΔR̅_{il} Δt] Δt - 0.5 ∑_{k=i}^{j-1} ΔR̅_{ik} Δt²
```

**Position w.r.t. gyro bias:**
```
∂Δp̅_{ij} / ∂b_i^g = -∑_{k=i}^{j-1} [∂Δp̅_{ij} / ∂ΔR̅_{ik}] [∂ΔR̅_{ik} / ∂b_i^g]
```

### Bias Correction Formula

When bias changes from `b_i` to `b_j`:

```
Δv̅_{ij}(b_j) ≈ Δv̅_{ij}(b_i) + J_v_ba · (b_j^a - b_i^a)
Δp̅_{ij}(b_j) ≈ Δp̅_{ij}(b_i) + J_p_ba · (b_j^a - b_i^a) + J_p_bg · (b_j^g - b_i^g)
```

---

## Revision History

| Date | Version | Author | Changes |
|------|---------|--------|---------|
| 2026-01-04 | 1.0 | Claude Code | Initial test specification |

---

**END OF DOCUMENT**
