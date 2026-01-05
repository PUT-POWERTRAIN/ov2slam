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
 * IMU Preintegration Implementation (Forster et al. 2016)
 *
 * Implementation of on-manifold preintegration for real-time visual-inertial
 * odometry. This class accumulates IMU measurements between keyframes and
 * provides efficient bias update mechanisms.
 *
 * Reference:
 * "On-Manifold Preintegration for Real-Time Visual-Inertial Odometry"
 * C. Forster, L. Carlone, F. Dellaert, D. Scaramuzza, IEEE TRO 2016
 */

#include "imu_preintegration.hpp"
#include <sophus/so3.hpp>

namespace ov2slam {

IMUPreintegration::IMUPreintegration(const Bias& bias_init)
    : bias_init_(bias_init),
      delta_R_(Eigen::Matrix3d::Identity()),
      delta_p_(Eigen::Vector3d::Zero()),
      delta_v_(Eigen::Vector3d::Zero()),
      J_dbg_(Eigen::Matrix3d::Zero()),
      J_dba_(Eigen::Matrix3d::Zero()),
      delta_t_(0.0) {
    // Already initialized above
}

void IMUPreintegration::integrate(const GTLoader::AHRSPose& imu_meas, double dt) {
    // 1. Bias-compensated measurements
    Eigen::Vector3d omega = imu_meas.angular_velocity - bias_init_.gyro;
    Eigen::Vector3d acc = imu_meas.linear_acceleration - bias_init_.accel;

    // 2. Save current state BEFORE updates
    // Forster et al. 2016, Eq. 5-7: All updates use current state, not updated state
    Eigen::Matrix3d delta_R_current = delta_R_;
    Eigen::Vector3d delta_v_current = delta_v_;

    // 3. Compute rotation increment: Exp(ω̃ * dt)
    Eigen::Matrix3d R_update = Sophus::SO3d::exp(omega * dt).matrix();

    // 4. Update rotation: ΔR ← ΔR * Exp(ω̃ * dt) [RIGHT multiplication]
    // Forster et al. 2016, Eq. 5: ΔR_{k+1} = ΔR_k * ΔR_{k→k+1}
    delta_R_ = delta_R_current * R_update;

    // 5. Update velocity: Δv ← Δv + ΔR * ã * dt
    // Forster et al. 2016, Eq. 6: Uses CURRENT rotation (before update)
    delta_v_ = delta_v_current + delta_R_current * acc * dt;

    // 6. Update position: Δp ← Δp + Δv * dt + 0.5 * ΔR * ã * dt²
    // Forster et al. 2016, Eq. 7: Uses CURRENT velocity and rotation
    delta_p_ = delta_p_ + delta_v_current * dt + 0.5 * delta_R_current * acc * dt * dt;

    // 7. Update Jacobians
    // Forster et al. 2016, Eq. 11-12: Use CURRENT rotation
    Eigen::Matrix3d omega_hat = Sophus::SO3d::hat(omega);

    // J_dbg ← J_dbg - ΔR * [ω̃]× * dt
    // Forster et al. 2016, Eq. 11 (NO transpose)
    J_dbg_ = J_dbg_ - delta_R_current * omega_hat * dt;

    // J_dba ← J_dba + ΔR * dt
    // Forster et al. 2016, Eq. 12 (ADDITION, not subtraction)
    J_dba_ = J_dba_ + delta_R_current * dt;

    // 8. Accumulate time
    delta_t_ += dt;
}

void IMUPreintegration::updateBias(const Bias& new_bias) {
    // Compute bias deltas
    Eigen::Vector3d dbg = new_bias.gyro - bias_init_.gyro;
    Eigen::Vector3d dba = new_bias.accel - bias_init_.accel;

    // Apply first-order corrections (Forster et al. 2016, Eq. 8)

    // 1. Rotation: ΔR̄ = ΔR * Exp(J_dbg * δbg) [RIGHT multiplication]
    // Forster et al. 2016, Eq. 8a: ΔR̄_ij = ΔR_ij * Exp(J_dbg * δbg)
    Eigen::Matrix3d R_correction = Sophus::SO3d::exp(J_dbg_ * dbg).matrix();
    delta_R_ = delta_R_ * R_correction;

    // 2. Velocity: Δv̄ = Δv + J_dba * δba
    // Forster et al. 2016, Eq. 8b: Δv̄_ij = Δv_ij + J_dba * δba
    delta_v_ = delta_v_ + J_dba_ * dba;

    // 3. Position: Δp̄ = Δp + J_dba * δba * Δt
    // Forster et al. 2016, Eq. 8c: Δp̄_ij = Δp_ij + J_dba * δba * Δt_ij
    delta_p_ = delta_p_ + J_dba_ * dba * delta_t_;

    // Update initial bias reference
    bias_init_ = new_bias;
}

} // namespace ov2slam
