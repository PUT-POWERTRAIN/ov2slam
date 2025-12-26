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

#include "trajectory_evaluator.hpp"
#include <fstream>
#include <sstream>
#include <iostream>

// Define M_PI if not available (non-standard POSIX constant)
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include <cmath>
#include <algorithm>
#include <numeric>

TrajectoryEvaluator::TrajectoryEvaluator(const std::string& slam_file,
                                         const std::string& gt_file) {
    loadTrajectory(slam_file, slam_poses_, slam_timestamps_);
    loadTrajectory(gt_file, gt_poses_, gt_timestamps_);

    std::cout << "[TrajectoryEvaluator] Loaded " << slam_poses_.size()
              << " SLAM poses and " << gt_poses_.size() << " GT poses\n";
}

void TrajectoryEvaluator::loadTrajectory(const std::string& file,
                                        std::vector<Sophus::SE3d>& poses,
                                        std::vector<double>& timestamps) {
    std::ifstream file_stream(file);
    if (!file_stream.is_open()) {
        std::cerr << "[TrajectoryEvaluator] Failed to open file: " << file << std::endl;
        return;
    }

    poses.clear();
    timestamps.clear();

    std::string line;
    bool first_line = true;

    while (std::getline(file_stream, line)) {
        if (line.empty() || line[0] == '#') continue;

        // Skip header line
        if (first_line) {
            first_line = false;
            continue;
        }

        std::istringstream iss(line);
        double timestamp, tx, ty, tz, qx, qy, qz, qw;

        // Format: timestamp tx ty tz qx qy qz qw
        iss >> timestamp >> tx >> ty >> tz >> qx >> qy >> qz >> qw;

        // Create SE3 pose
        Eigen::Vector3d t(tx, ty, tz);
        Eigen::Quaterniond q(qw, qx, qy, qz);

        if (q.norm() > 0.9) {  // Valid quaternion
            q.normalize();
            poses.push_back(Sophus::SE3d(q, t));
            timestamps.push_back(timestamp);
        }
    }

    std::cout << "[TrajectoryEvaluator] Loaded " << poses.size()
              << " poses from " << file << "\n";
}

double TrajectoryEvaluator::computeInitialHeading(int num_frames) {
    if (slam_poses_.size() < 2) {
        std::cerr << "[TrajectoryEvaluator] Not enough poses to compute heading\n";
        return 0.0;
    }

    num_frames = std::min(num_frames, (int)slam_poses_.size());

    // Compute average direction vector from first N frames
    Eigen::Vector3d avg_direction = Eigen::Vector3d::Zero();

    for (int i = 1; i < num_frames; ++i) {
        Eigen::Vector3d delta = slam_poses_[i].translation() - slam_poses_[i-1].translation();
        avg_direction += delta;
    }

    if (avg_direction.norm() < 1e-6) {
        std::cerr << "[TrajectoryEvaluator] Insufficient motion to compute heading\n";
        return 0.0;
    }

    avg_direction /= (num_frames - 1);

    // Compute heading angle in XY plane
    double heading = std::atan2(avg_direction.y(), avg_direction.x());

    std::cout << "[TrajectoryEvaluator] Initial SLAM heading: " << heading
              << " rad (" << (heading * 180.0 / M_PI) << " deg)\n";

    return heading;
}

double TrajectoryEvaluator::computeGTInitialHeading(int num_frames) {
    if (gt_poses_.size() < 2) {
        std::cerr << "[TrajectoryEvaluator] Not enough GT poses to compute heading\n";
        return 0.0;
    }

    num_frames = std::min(num_frames, (int)gt_poses_.size());

    // Compute average direction vector from first N frames
    Eigen::Vector3d avg_direction = Eigen::Vector3d::Zero();

    for (int i = 1; i < num_frames; ++i) {
        Eigen::Vector3d delta = gt_poses_[i].translation() - gt_poses_[i-1].translation();
        avg_direction += delta;
    }

    if (avg_direction.norm() < 1e-6) {
        std::cerr << "[TrajectoryEvaluator] Insufficient GT motion to compute heading\n";
        return 0.0;
    }

    avg_direction /= (num_frames - 1);

    // Compute heading angle in XY plane
    double heading = std::atan2(avg_direction.y(), avg_direction.x());

    std::cout << "[TrajectoryEvaluator] Initial GT heading: " << heading
              << " rad (" << (heading * 180.0 / M_PI) << " deg)\n";

    return heading;
}

std::vector<double> TrajectoryEvaluator::computeDirectionChanges() {
    std::vector<double> direction_changes;

    if (slam_poses_.size() < 3) {
        std::cerr << "[TrajectoryEvaluator] Not enough poses to compute direction changes\n";
        return direction_changes;
    }

    for (size_t i = 2; i < slam_poses_.size(); ++i) {
        // Compute direction vectors
        Eigen::Vector3d dir1 = slam_poses_[i-1].translation() - slam_poses_[i-2].translation();
        Eigen::Vector3d dir2 = slam_poses_[i].translation() - slam_poses_[i-1].translation();

        if (dir1.norm() < 1e-6 || dir2.norm() < 1e-6) {
            continue;  // Skip if no motion
        }

        dir1.normalize();
        dir2.normalize();

        // Compute angle between directions
        double dot_product = dir1.dot(dir2);
        dot_product = std::max(-1.0, std::min(1.0, dot_product));  // Clamp to valid range
        double angle = std::acos(dot_product);

        direction_changes.push_back(angle);
    }

    std::cout << "[TrajectoryEvaluator] Computed " << direction_changes.size()
              << " direction changes\n";

    return direction_changes;
}

double TrajectoryEvaluator::estimateScale() {
    if (slam_poses_.size() < 2 || gt_poses_.size() < 2) {
        std::cerr << "[TrajectoryEvaluator] Not enough poses to estimate scale\n";
        return 1.0;
    }

    // Compute total path length for both trajectories
    double slam_length = 0.0;
    double gt_length = 0.0;

    for (size_t i = 1; i < slam_poses_.size(); ++i) {
        slam_length += (slam_poses_[i].translation() - slam_poses_[i-1].translation()).norm();
    }

    for (size_t i = 1; i < gt_poses_.size(); ++i) {
        gt_length += (gt_poses_[i].translation() - gt_poses_[i-1].translation()).norm();
    }

    if (gt_length < 1e-6) {
        std::cerr << "[TrajectoryEvaluator] GT trajectory too short for scale estimation\n";
        return 1.0;
    }

    double scale = slam_length / gt_length;

    std::cout << "[TrajectoryEvaluator] Estimated scale: " << scale
              << " (SLAM: " << slam_length << "m, GT: " << gt_length << "m)\n";

    return scale;
}

double TrajectoryEvaluator::computeATE() {
    if (slam_poses_.empty() || gt_poses_.empty()) {
        std::cerr << "[TrajectoryEvaluator] No poses to compute ATE\n";
        return -1.0;
    }

    // Find overlapping timestamps
    std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> matched_poses;

    for (size_t i = 0; i < slam_poses_.size(); ++i) {
        double slam_time = slam_timestamps_[i];

        // Find closest GT timestamp
        double min_diff = 1.0;
        size_t closest_idx = 0;

        for (size_t j = 0; j < gt_poses_.size(); ++j) {
            double diff = std::abs(gt_timestamps_[j] - slam_time);
            if (diff < min_diff) {
                min_diff = diff;
                closest_idx = j;
            }
        }

        if (min_diff < 0.1) {  // Within 100ms
            matched_poses.push_back({
                slam_poses_[i].translation(),
                gt_poses_[closest_idx].translation()
            });
        }
    }

    if (matched_poses.empty()) {
        std::cerr << "[TrajectoryEvaluator] No matching poses found for ATE computation\n";
        return -1.0;
    }

    // Compute ATE
    double sum_sq_error = 0.0;
    for (const auto& pair : matched_poses) {
        double error = (pair.first - pair.second).norm();
        sum_sq_error += error * error;
    }

    double ate = std::sqrt(sum_sq_error / matched_poses.size());

    std::cout << "[TrajectoryEvaluator] ATE: " << ate
              << "m (from " << matched_poses.size() << " matched poses)\n";

    return ate;
}

double TrajectoryEvaluator::computeDirectionConsistency() {
    auto direction_changes = computeDirectionChanges();

    if (direction_changes.empty()) {
        return 0.0;
    }

    // Compute average direction change
    double avg_change = std::accumulate(direction_changes.begin(),
                                       direction_changes.end(), 0.0)
                       / direction_changes.size();

    // Convert to consistency metric (0-1, higher is better)
    // Low direction changes = high consistency
    // Normalize assuming reasonable direction changes are < 45 degrees (pi/4)
    double consistency = std::exp(-avg_change / (M_PI / 4.0));

    std::cout << "[TrajectoryEvaluator] Direction consistency: " << consistency
              << " (avg change: " << (avg_change * 180.0 / M_PI) << " deg)\n";

    return consistency;
}
