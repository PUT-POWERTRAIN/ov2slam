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
#pragma once

#include <sophus/se3.hpp>
#include <Eigen/Dense>
#include <string>
#include <vector>

class TrajectoryEvaluator {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    TrajectoryEvaluator(const std::string& slam_file,
                       const std::string& gt_file);

    // Initial heading from first N frames (radians)
    double computeInitialHeading(int num_frames = 10);

    // Heading from GT trajectory
    double computeGTInitialHeading(int num_frames = 10);

    // Direction changes along trajectory (returns angles in radians)
    std::vector<double> computeDirectionChanges();

    // Scale estimation (stereo should be ~1.0)
    double estimateScale();

    // ATE (Absolute Trajectory Error) vs GT
    double computeATE();

    // Direction consistency metric (0-1, higher is better)
    double computeDirectionConsistency();

private:
    std::vector<Sophus::SE3d> slam_poses_;
    std::vector<Sophus::SE3d> gt_poses_;
    std::vector<double> slam_timestamps_;
    std::vector<double> gt_timestamps_;

    void loadTrajectory(const std::string& file,
                       std::vector<Sophus::SE3d>& poses,
                       std::vector<double>& timestamps);
};
