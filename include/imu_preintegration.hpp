/**
*    This file is part of OV²SLAM.
*
*    Copyright (C) 2020 ONERA
*
*    For more information see <https://github.com/ov2slam/ov2slam>
*
*    OV²SLAM is free software: you can redistribute it and/or modify
*    it under the terms of the GNU General Public License as published by
*    the Free Software Foundation, either version 3 of the License, or
*    (at your option) any later version.
*
*    OV²SLAM is distributed in the hope that it will be useful,
*    but WITHOUT ANY WARRANTY; without even the implied warranty of
*    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*    GNU General Public License for more details.
*
*    You should have received a copy of the GNU General Public License
*    along with OV²SLAM.  If not, see <https://www.gnu.org/licenses/>.
*
*    Authors: Maxime Ferrera     <maxime.ferrera at gmail dot com> (ONERA, DTIS - IVA),
*             Alexandre Eudes    <first.last at onera dot fr>      (ONERA, DTIS - IVA),
*             Julien Moras       <first.last at onera dot fr>      (ONERA, DTIS - IVA),
*             Martial Sanfourche <first.last at onera dot fr>      (ONERA, DTIS - IVA)
*/

/**
 * IMU Preintegration (Forster et al. 2016)
 *
 * Theory:
 * Preintegrates IMU measurements between two consecutive keyframes such that
 * bias updates can be applied via first-order correction without re-integrating
 * all IMU measurements between the frames.
 *
 * Key Equations (Forster et al. 2016, Eq. 4-8):
 *
 * 1. Preintegrated measurements:
 *    Δt_ij = Σ Δt
 *    ΔR_ij = Π R_{k+1} * exp((ω_k - b_g) * Δt)
 *    Δv_ij = Σ ΔR_ik * (a_k - b_a) * Δt
 *    Δp_ij = Σ [Δv_ik + 0.5 * ΔR_ik * (a_k - b_a) * Δt] * Δt
 *
 * 2. Bias correction (first-order approximation):
 *    ΔR̄_ij ≈ ΔR_ij * Exp(J_dbg * δb_g)
 *    Δv̄_ij ≈ Δv_ij + J_dba * δb_a
 *    Δp̄_ij ≈ Δp_ij + J_dba * δb_a * Δt_ij
 *
 * where:
 * - ΔR_ij, Δv_ij, Δp_ij: preintegrated measurements between frames i and j
 * - b_g, b_a: gyroscope and accelerometer biases
 * - J_dbg, J_dba: Jacobians wrt biases (for first-order correction)
 * - Exp(): SO(3) exponential map
 *
 * Reference:
 * "On-Manifold Preintegration for Real-Time Visual-Inertial Odometry"
 * C. Forster, L. Carlone, F. Dellaert, D. Scaramuzza, IEEE TRO 2016
 *
 * NOTE: This MVP implementation does NOT include gravity compensation.
 * Gravity will be added in Phase 3 (full IMU factor).
 */
#pragma once

#include <Eigen/Core>
#include <vector>
#include "gt_loader.hpp"  // For GTLoader::AHRSPose

namespace ov2slam {

/**
 * IMU Preintegration Class
 *
 * Accumulates IMU measurements between two keyframes using manifold
 * preintegration (Forster et al. 2016). Stores preintegrated ΔR, Δv, Δp
 * and Jacobians for efficient bias updates.
 */
class IMUPreintegration {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /**
     * Bias structure for gyroscope and accelerometer
     *
     * Units:
     * - gyro: rad/s (angular velocity bias)
     * - accel: m/s² (linear acceleration bias)
     */
    struct Bias {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        Eigen::Vector3d gyro;    // [rad/s] gyroscope bias
        Eigen::Vector3d accel;   // [m/s²] accelerometer bias

        Bias() : gyro(Eigen::Vector3d::Zero()),
                 accel(Eigen::Vector3d::Zero()) {}

        Bias(const Eigen::Vector3d& g, const Eigen::Vector3d& a)
            : gyro(g), accel(a) {}
    };

    /**
     * Constructor with initial bias
     *
     * @param bias_init Initial bias estimate (typically from previous state)
     *
     * Initializes:
     * - delta_R_ = I₃ (identity rotation)
     * - delta_p_ = 0₃ (zero position change)
     * - delta_v_ = 0₃ (zero velocity change)
     * - J_dbg_ = 0₃ (zero Jacobian for gyro bias)
     * - J_dba_ = 0₃ (zero Jacobian for accel bias)
     * - delta_t_ = 0.0 (zero elapsed time)
     */
    explicit IMUPreintegration(const Bias& bias_init);

    /**
     * Integrate a single IMU measurement
     *
     * Updates preintegrated measurements using mid-point integration
     * (Forster et al. 2016, Eq. 7-8).
     *
     * @param imu_meas IMU measurement containing gyro and accel data
     * @param dt Time interval since previous measurement [seconds]
     *
     * Algorithm:
     * 1. Bias-compensated measurements:
     *    ω̃ = ω - b_g (gyro)
     *    ã = a - b_a (accel)
     *
     * 2. Update ΔR (rotation):
     *    ΔR ← ΔR * Exp(ω̃ * dt)
     *
     * 3. Update Δv (velocity):
     *    Δv ← Δv + ΔR * ã * dt
     *
     * 4. Update Δp (position):
     *    Δp ← Δp + Δv * dt + 0.5 * ΔR * ã * dt²
     *
     * 5. Update Jacobians (Eq. 11-12 in paper):
     *    J_dbg ← J_dbg - ΔR * [ω̃]× * dt
     *    J_dba ← J_dba + ΔR * dt
     *
     * 6. Accumulate time:
     *    Δt ← Δt + dt
     *
     * @note This is the NUMERICAL integration step. Bias correction is
     *       handled symbolically in updateBias().
     */
    void integrate(const GTLoader::AHRSPose& imu_meas, double dt);

    /**
     * Get preintegrated rotation (ΔR_ij)
     *
     * @return Rotation matrix R_i^j (from frame i to frame j)
     *
     * This represents the relative rotation between frames i and j,
     * independent of the initial bias estimate.
     *
     * Frame convention:
     * - Expressed in BODY frame of frame i
     * - Can be transformed to World frame as: R_w_i^T * ΔR_ij * R_w_i
     */
    Eigen::Matrix3d getDeltaRotation() const { return delta_R_; }

    /**
     * Get preintegrated position change (Δp_ij)
     *
     * @return Position change [meters]
     *
     * This represents the relative position displacement between frames i
     * and j, expressed in the BODY frame of frame i.
     *
     * Units: meters
     * Frame: BODY frame of frame i
     */
    Eigen::Vector3d getDeltaPosition() const { return delta_p_; }

    /**
     * Get preintegrated velocity change (Δv_ij)
     *
     * @return Velocity change [m/s]
     *
     * This represents the relative velocity change between frames i and j,
     * expressed in the BODY frame of frame i.
     *
     * Units: m/s
     * Frame: BODY frame of frame i
     */
    Eigen::Vector3d getDeltaVelocity() const { return delta_v_; }

    /**
     * Get total integration time (Δt_ij)
     *
     * @return Accumulated time interval [seconds]
     *
     * This is the sum of all dt values passed to integrate().
     */
    double getDeltaT() const { return delta_t_; }

    /**
     * Update bias with first-order correction
     *
     * Applies a bias change δb = b_new - b_old to the preintegrated
     * measurements WITHOUT re-integrating all IMU samples.
     *
     * @param new_bias New bias estimate
     *
     * Theory (Forster et al. 2016, Eq. 10):
     * When bias changes from b to b̄ = b + δb, the preintegrated measurements
     * are corrected as:
     *
     * 1. Rotation:
     *    ΔR̄_ij = ΔR_ij * Exp(J_dbg * δb_g)
     *
     * 2. Velocity:
     *    Δv̄_ij = Δv_ij + J_dba * δb_a
     *
     * 3. Position:
     *    Δp̄_ij = Δp_ij + J_dba * δb_a * Δt_ij
     *
     * where δb_g = new_bias.gyro - bias_init_.gyro
     *       δb_a = new_bias.accel - bias_init_.accel
     *
     * This method MODIFIES delta_R_, delta_v_, delta_p_ in-place using the
     * first-order correction above. After calling this method, subsequent
     * calls to getDeltaRotation(), getDeltaVelocity(), getDeltaPosition()
     * will return the bias-corrected preintegrated measurements.
     *
     * @note This is an APPROXIMATION valid for small bias changes.
     *       For large bias changes (> 0.1 rad/s or > 0.5 m/s²),
     *       re-integration may be necessary.
     */
    void updateBias(const Bias& new_bias);

private:
    Bias bias_init_;           // Initial bias used for integration
                                // Used as reference for bias correction

    // Preintegrated measurements
    Eigen::Matrix3d delta_R_;  // ΔR_ij: relative rotation R_i^j
    Eigen::Vector3d delta_p_;  // Δp_ij: relative position change [m]
    Eigen::Vector3d delta_v_;  // Δv_ij: relative velocity change [m/s]

    // Jacobians for first-order bias correction
    Eigen::Matrix3d J_dbg_;    // ∂(ΔR_ij)/∂(b_g): Jacobian of ΔR wrt gyro bias
                                // Shape: 3x3, units: rad/(rad/s)
    Eigen::Matrix3d J_dba_;    // ∂(Δv_ij)/∂(b_a): Jacobian of Δv wrt accel bias
                                // Shape: 3x3, units: (m/s)/(m/s²) = s

    double delta_t_;           // Δt_ij: total integration time [seconds]
};

} // namespace ov2slam
