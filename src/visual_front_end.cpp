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

#include <opencv2/video/tracking.hpp>

#include "visual_front_end.hpp"
#include "multi_view_geometry.hpp"
#include <Eigen/SVD>

#include <opencv2/highgui.hpp>

#include "sync_profiler.hpp"


VisualFrontEnd::VisualFrontEnd(std::shared_ptr<SlamParams> pstate, std::shared_ptr<Frame> pframe,
        std::shared_ptr<MapManager> pmap, std::shared_ptr<FeatureTracker> ptracker,
        std::shared_ptr<GTLoader> gt_loader)
    : pslamstate_(pstate), pcurframe_(pframe), pmap_(pmap), ptracker_(ptracker),
      gt_loader_(gt_loader)
{}

bool VisualFrontEnd::visualTracking(cv::Mat &iml, cv::Mat &imr, double time)
{
    PROFILE_FUNCTION();

    ProfiledLockGuard lock(pmap_->map_mutex_);

    if( pslamstate_->debug_ || pslamstate_->log_timings_ )
        Profiler::Start("0.Full-Front_End");

    bool iskfreq;

    // Route to stereo or mono tracking based on configuration
    if( pslamstate_->stereo_ ) {
        iskfreq = trackStereo(iml, imr, time);
    } else {
        iskfreq = trackMono(iml, time);
    }

    if( iskfreq ) {
        pmap_->createKeyframe(cur_img_, iml);

        if( pslamstate_->btrack_keyframetoframe_ ) {
            cv::buildOpticalFlowPyramid(cur_img_, kf_pyr_, pslamstate_->klt_win_size_, pslamstate_->nklt_pyr_lvl_);
        }
    }

    if( pslamstate_->debug_ || pslamstate_->log_timings_ )
        Profiler::StopAndDisplay(pslamstate_->debug_, "0.Full-Front_End");

    return iskfreq;
}


// Perform tracking in one image, update kps and MP obs, return true if a new KF is req.
bool VisualFrontEnd::trackMono(cv::Mat &im, double time)
{
    PROFILE_FUNCTION();

    if( pslamstate_->debug_ )
        std::cout << "\n\n - [Visual-Front-End]: Track Mono Image\n";
    
    if( pslamstate_->debug_ || pslamstate_->log_timings_ )
        Profiler::Start("1.FE_Track-Mono");

    // Preprocess the new image
    preprocessImage(im);

    // Create KF if 1st frame processed
    if( motion_model_.prev_time_ < 0 ) {
        // First frame - initialize velocity to zero
        pcurframe_->setVelocity(Eigen::Vector3d::Zero());
        std::cout << "[INIT] First frame (id=" << pcurframe_->id_ << "): velocity initialized to zero" << std::endl;
        // NOTE: Don't return! Continue to tracking so motion model gets updated
    }

    // Apply Motion model to predict cur Frame pose
    // Phase 3: Try IMU-based prediction first, fallback to constant velocity
    Sophus::SE3d Twc = pcurframe_->getTwc();

    // Only attempt IMU prediction if we have a previous frame with velocity
    if( motion_model_.prev_time_ >= 0 && gt_loader_ && motion_model_.has_prev_velocity_ ) {
        std::cout << "[DEBUG] frame=" << pcurframe_->id_
                  << " gt_loader=" << (gt_loader_ != nullptr)
                  << " has_vel=" << motion_model_.has_prev_velocity_
                  << " prev_time=" << motion_model_.prev_time_ << std::endl;

        // IMU-based prediction (Forster et al. 2016, Eq. 5-7)
        double t_prev = motion_model_.prev_time_;
        double t_cur = time;

        // Get IMU measurements between frames
        std::vector<GTLoader::AHRSPose> imu_data =
            gt_loader_->getIMUData(t_prev, t_cur);

        std::cout << "[IMU_ATTEMPT] frame=" << pcurframe_->id_
                  << " nb_imu=" << imu_data.size() << std::endl;

        if( !imu_data.empty() ) {
            // Initialize preintegration (assuming zero bias for MVP)
            ov2slam::IMUPreintegration::Bias bias;
            ov2slam::IMUPreintegration preint(bias);

            // Integrate all IMU measurements
            for( size_t i = 0; i < imu_data.size(); ++i ) {
                double dt = (i == imu_data.size() - 1) ?
                           (t_cur - imu_data[i].timestamp) :
                           (imu_data[i+1].timestamp - imu_data[i].timestamp);
                preint.integrate(imu_data[i], dt);
            }

            // Get previous frame state
            Eigen::Matrix3d R_prev = motion_model_.prevTwc_.rotationMatrix();
            Eigen::Vector3d p_prev = motion_model_.prevTwc_.translation();
            Eigen::Vector3d v_prev = motion_model_.prev_velocity_;

            // Get preintegrated measurements
            double dt = preint.getDeltaT();
            Eigen::Matrix3d dR = preint.getDeltaRotation();
            Eigen::Vector3d dp = preint.getDeltaPosition();
            Eigen::Vector3d dv = preint.getDeltaVelocity();

            // Predict pose: R_pred = R_prev * dR, p_pred = p_prev + v_prev*dt + R_prev*dp
            Eigen::Matrix3d R_pred = R_prev * dR;
            Eigen::Vector3d p_pred = p_prev + v_prev * dt + R_prev * dp;

            // Orthogonalize R_pred to avoid numerical errors
            Eigen::JacobiSVD<Eigen::Matrix3d> svd(R_pred, Eigen::ComputeFullU | Eigen::ComputeFullV);
            R_pred = svd.matrixU() * svd.matrixV().transpose();

            // Velocity prediction: v_pred = v_prev + R_prev * dv
            // NOTE: AHRS data has gravity already removed (proper acceleration)
            // Forster et al. 2016 Eq. 32 assumes raw IMU, so we omit g*Δt term
            Eigen::Vector3d v_pred = v_prev + R_prev * dv;

            // Sanity check: reject unrealistic velocities (>50 m/s = 180 km/h)
            double v_norm = v_pred.norm();
            if( v_norm > 50.0 ) {
                std::cerr << "[WARNING] Unrealistic velocity: " << v_norm << " m/s at frame " << pcurframe_->id_ << std::endl;
                // Clamp to reasonable maximum
                v_pred = (v_pred / v_norm) * 50.0;
            }

            // Set predicted pose and velocity
            Twc = Sophus::SE3d(R_pred, p_pred);
            pcurframe_->setTwc(Twc);
            pcurframe_->setVelocity(v_pred);

            if( pslamstate_->debug_ ) {
                std::cout << "[IMU_PRED] frame=" << pcurframe_->id_
                          << " dt=" << dt
                          << " nb_imu=" << imu_data.size()
                          << " Z_pred=" << p_pred.z()
                          << " v_pred=" << v_pred.norm() << std::endl;
            }
        } else {
            // No IMU data available - fallback to constant velocity
            std::cout << "[FALLBACK] No IMU data - using constant velocity" << std::endl;
            motion_model_.applyMotionModel(Twc, time);
            pcurframe_->setTwc(Twc);
        }
    } else {
        // First frame or no velocity/GTLoader - use constant velocity
        if( motion_model_.prev_time_ >= 0 ) {
            std::cout << "[FALLBACK] frame=" << pcurframe_->id_
                      << " gt_loader=" << (gt_loader_ != nullptr)
                      << " has_vel=" << motion_model_.has_prev_velocity_
                      << " prev_time=" << motion_model_.prev_time_ << std::endl;
        }
        motion_model_.applyMotionModel(Twc, time);
        pcurframe_->setTwc(Twc);
    }

    // SCI-LOG-1: Motion model prediction (all frames for debugging)
    std::cout << "[POSE_PRED] frame=" << pcurframe_->id_
              << " Z=" << Twc.translation().z()
              << " nb_3d=" << pcurframe_->nb3dkps_ << std::endl;
    
    // Track the new image
    if( pslamstate_->btrack_keyframetoframe_ ) {
        kltTrackingFromKF();
    } else {
        kltTracking();
    }

    if( pslamstate_->doepipolar_ ) {
        // Check2d2dOutliers
        epipolar2d2dFiltering();
    }

    if( pslamstate_->mono_ && !pslamstate_->bvision_init_ ) 
    {
        if( pcurframe_->nb2dkps_ < 50 ) {
            pslamstate_->breset_req_ = true;
            return false;
        } 
        else if( checkReadyForInit() ) {
            std::cout << "\n\n - [Visual-Front-End]: Mono Visual SLAM ready for initialization!";
            pslamstate_->bvision_init_ = true;
            return true;
        } 
        else {
            std::cout << "\n\n - [Visual-Front-End]: Not ready to init yet!";
            return false;
        }
    }

    // Compute Pose (2D-3D)
    if( !pslamstate_->imu_only_mode_ ) {
        computePose(); // Normal mode: PnP estimation
    } else {
        std::cout << "[IMU_ONLY] Skipping PnP, using IMU prediction only\n";
        // IMU prediction already done (lines 105-184)
        // Keep predicted pose, don't compute PnP
    }

    // CORRECT velocity from visual pose estimate (fixes IMU drift)
    // WITHOUT THIS: velocity accumulates IMU bias errors and explodes to 50 m/s
    // Must compute BEFORE updateMotionModel() which overwrites prev_time_
    // SKIP in IMU-only mode to preserve IMU velocity
    if( !pslamstate_->imu_only_mode_ ) {
        if( motion_model_.prev_time_ > 0 && time > motion_model_.prev_time_ ) {
            Eigen::Vector3d p_cur = pcurframe_->getTwc().translation();
            Eigen::Vector3d p_prev = motion_model_.prevTwc_.translation();
            double dt = time - motion_model_.prev_time_;
            Eigen::Vector3d v_visual = (p_cur - p_prev) / dt;
            pcurframe_->setVelocity(v_visual);

            std::cout << "[VELOCITY_CORRECTION] frame=" << pcurframe_->id_
                      << " v_visual=" << v_visual.transpose()
                      << " dt=" << dt << " s" << std::endl;
        } else {
            std::cout << "[VELOCITY_CORRECTION] SKIP frame=" << pcurframe_->id_
                      << " prev_time=" << motion_model_.prev_time_
                      << " time=" << time << std::endl;
        }
    } else {
        std::cout << "[IMU_ONLY] Using IMU velocity only, no visual correction\n";
    }

    // Update Motion model from estimated pose (must be AFTER velocity correction!)
    motion_model_.updateMotionModel(pcurframe_->Twc_, time);

    // Update velocity in motion model (Phase 3: IMU)
    if( pcurframe_->hasVelocity() ) {
        Eigen::Vector3d vel = pcurframe_->getVelocity();
        motion_model_.updateMotionModelVelocity(vel, true);
        std::cout << "[VELOCITY_UPDATE] frame=" << pcurframe_->id_
                  << " vel=" << vel.transpose()
                  << " has_vel=" << pcurframe_->hasVelocity() << std::endl;
    } else {
        motion_model_.updateMotionModelVelocity(Eigen::Vector3d::Zero(), false);
        std::cout << "[VELOCITY_UPDATE] frame=" << pcurframe_->id_
                  << " NO VELOCITY" << std::endl;
    }

    // SCI-LOG-2: After PnP (all frames for debugging)
    std::cout << "[POSE_PNP] frame=" << pcurframe_->id_
              << " Z=" << pcurframe_->getTwc().translation().z()
              << " nb_3d=" << pcurframe_->nb3dkps_ << std::endl;

    // Check if New KF req.
    bool is_kf_req = checkNewKfReq();

    // SCI-LOG-3: Keyframe decision (all frames for debugging)
    std::cout << "[KF_DEC] frame=" << pcurframe_->id_
              << " is_kf=" << is_kf_req << std::endl;

    if( pslamstate_->debug_ || pslamstate_->log_timings_ )
        Profiler::StopAndDisplay(pslamstate_->debug_, "1.FE_Track-Mono");

    return is_kf_req;
}


// Perform stereo tracking with left and right images
bool VisualFrontEnd::trackStereo(cv::Mat &iml, cv::Mat &imr, double time)
{
    PROFILE_FUNCTION();

    if( pslamstate_->debug_ )
        std::cout << "\n\n - [Visual-Front-End]: Track Stereo Image\n";

    if( pslamstate_->debug_ || pslamstate_->log_timings_ )
        Profiler::Start("1.FE_Track-Stereo");

    // 1. Preprocess left image (builds pyramid)
    preprocessImage(iml);

    // Store right image and build KLT pyramid for stereo matching
    if( pslamstate_->use_clahe_ ) {
        ptracker_->pclahe_->apply(imr, right_img_);
    } else {
        right_img_ = imr;
    }

    // Build right pyramid (needed for MapManager::stereoMatching() later)
    if( pslamstate_->do_klt_ ) {
        cv::buildOpticalFlowPyramid(right_img_, right_pyr_,
            pslamstate_->klt_win_size_,
            pslamstate_->nklt_pyr_lvl_);
    }

    // 2. Extract keypoints on first frame (when no features exist yet)
    if( motion_model_.prev_time_ < 0 ) {
        pmap_->extractKeypoints(cur_img_, cur_img_);
    }

    // 3. Temporal tracking (for frames after the first)
    // Track features from previous frame to current frame (same as mono mode)
    if( motion_model_.prev_time_ >= 0 ) {
        if( pslamstate_->btrack_keyframetoframe_ ) {
            kltTrackingFromKF();
        } else {
            kltTracking();
        }
    }

    // Epipolar filtering - remove KLT outliers
    if( pslamstate_->doepipolar_ ) {
        epipolar2d2dFiltering();
    }

    // 4. Apply Motion Model (same as mono tracking)
    if( motion_model_.prev_time_ < 0 ) {
        // First frame - initialize velocity to zero in BOTH frame AND motion model
        pcurframe_->setVelocity(Eigen::Vector3d::Zero());
        motion_model_.updateMotionModelVelocity(Eigen::Vector3d::Zero(), true);  // CRITICAL for IMU prediction
    }

    Sophus::SE3d Twc = pcurframe_->getTwc();

    // Try IMU-based prediction first, fallback to constant velocity
    std::cout << "[DEBUG_STEREO] frame=" << pcurframe_->id_
              << " prev_time=" << motion_model_.prev_time_
              << " has_vel=" << motion_model_.has_prev_velocity_
              << " gt_loader=" << (gt_loader_ != nullptr) << std::endl;

    if( motion_model_.prev_time_ >= 0 && gt_loader_ && motion_model_.has_prev_velocity_ ) {
        double t_prev = motion_model_.prev_time_;
        double t_cur = time;

        std::vector<GTLoader::AHRSPose> imu_data =
            gt_loader_->getIMUData(t_prev, t_cur);

        std::cout << "[DEBUG_STEREO] IMU data: " << imu_data.size() << " measurements" << std::endl;

        if( !imu_data.empty() ) {
            ov2slam::IMUPreintegration::Bias bias;
            ov2slam::IMUPreintegration preint(bias);

            for( size_t i = 0; i < imu_data.size(); ++i ) {
                double dt = (i == imu_data.size() - 1) ?
                           (t_cur - imu_data[i].timestamp) :
                           (imu_data[i+1].timestamp - imu_data[i].timestamp);
                preint.integrate(imu_data[i], dt);
            }

            Eigen::Matrix3d R_prev = motion_model_.prevTwc_.rotationMatrix();
            Eigen::Vector3d p_prev = motion_model_.prevTwc_.translation();
            Eigen::Vector3d v_prev = motion_model_.prev_velocity_;

            double dt = preint.getDeltaT();
            Eigen::Matrix3d dR = preint.getDeltaRotation();
            Eigen::Vector3d dp = preint.getDeltaPosition();
            Eigen::Vector3d dv = preint.getDeltaVelocity();

            Eigen::Matrix3d R_pred = R_prev * dR;
            Eigen::Vector3d p_pred = p_prev + v_prev * dt + R_prev * dp;

            // Orthogonalize R_pred
            Eigen::JacobiSVD<Eigen::Matrix3d> svd(R_pred, Eigen::ComputeFullU | Eigen::ComputeFullV);
            R_pred = svd.matrixU() * svd.matrixV().transpose();

            Eigen::Vector3d v_pred = v_prev + R_prev * dv;

            // Sanity check: reject unrealistic velocities
            double v_norm = v_pred.norm();
            if( v_norm > 50.0 ) {
                std::cerr << "[WARNING] Unrealistic velocity: " << v_norm << " m/s at frame " << pcurframe_->id_ << std::endl;
                v_pred = (v_pred / v_norm) * 50.0;
            }

            Twc = Sophus::SE3d(R_pred, p_pred);
            pcurframe_->setTwc(Twc);
            pcurframe_->setVelocity(v_pred);

            // CRITICAL: Update motion model velocity for next frame's prediction
            motion_model_.updateMotionModelVelocity(v_pred, true);

            if( pslamstate_->debug_ ) {
                std::cout << "[IMU_PRED] frame=" << pcurframe_->id_
                          << " dt=" << dt
                          << " nb_imu=" << imu_data.size()
                          << " Z_pred=" << p_pred.z()
                          << " v_pred=" << v_pred.norm() << std::endl;
            }
        } else {
            std::cout << "[DEBUG_STEREO] No IMU data - using applyMotionModel" << std::endl;
            motion_model_.applyMotionModel(Twc, time);
            pcurframe_->setTwc(Twc);
        }
    } else {
        std::cout << "[DEBUG_STEREO] Condition failed - using applyMotionModel" << std::endl;
        motion_model_.applyMotionModel(Twc, time);
        pcurframe_->setTwc(Twc);
    }

    // NOTE: Stereo matching happens in MapManager::stereoMatching() when keyframes are created
    // This uses ZNCC (proper for stereo) instead of KLT (which is for temporal tracking)
    // The mapper will automatically match left↔right features and triangulate 3D points

    // 5. Compute Pose (PnP with stereo + temporal features)
    // Always run PnP unless in IMU-only mode (needed for inliers count)
    if( !pslamstate_->imu_only_mode_ ) {
        computePose(); // Normal mode: PnP estimation
    } else {
        std::cout << "[IMU_ONLY] Skipping PnP, using IMU prediction only (stereo)\n";
        // IMU prediction already done, keep predicted pose
    }

    // 6. Validation Layer: Hybrid Vision + GPS dead reckoning
    // Decide between Vision (docking) and GPS (transit) based on PnP quality
    bool use_gps_mode = false;

    if( pslamstate_->imu_only_mode_ ) {
        // IMU-only mode: Always use GPS
        use_gps_mode = true;
    }
    else if( pslamstate_->validation_enable_ ) {
        // Validation layer enabled: Automatic switching based on inliers
        int inliers = last_nbinliers_;

        // State machine with hysteresis
        if( nav_mode_ == NavMode::VISION ) {
            // Currently in VISION mode
            if( inliers < pslamstate_->min_inliers_gps_ ) {
                // Poor tracking: Switch to GPS after hysteresis
                nav_mode_counter_++;
                if( nav_mode_counter_ >= pslamstate_->hysteresis_frames_ ) {
                    std::cout << "[VALIDATION] Switching VISION -> GPS (inliers=" << inliers
                              << " < " << pslamstate_->min_inliers_gps_ << " for "
                              << nav_mode_counter_ << " frames)" << std::endl;
                    nav_mode_ = NavMode::GPS;
                    nav_mode_counter_ = 0;
                    use_gps_mode = true;
                }
            } else {
                // Good tracking: Stay in VISION mode
                nav_mode_counter_ = 0;
            }
        } else {
            // Currently in GPS mode
            if( inliers >= pslamstate_->min_inliers_vision_ ) {
                // Good tracking recovered: Switch to VISION after hysteresis
                nav_mode_counter_++;
                if( nav_mode_counter_ >= pslamstate_->hysteresis_frames_ ) {
                    std::cout << "[VALIDATION] Switching GPS -> VISION (inliers=" << inliers
                              << " >= " << pslamstate_->min_inliers_vision_ << " for "
                              << nav_mode_counter_ << " frames)" << std::endl;
                    nav_mode_ = NavMode::VISION;
                    nav_mode_counter_ = 0;
                    use_gps_mode = false;
                }
            } else {
                // Still poor tracking: Stay in GPS mode
                nav_mode_counter_ = 0;
                use_gps_mode = true;
            }
        }

        // Log current mode
        if( nav_mode_ == NavMode::VISION ) {
            std::cout << "[VALIDATION] VISION mode (inliers=" << inliers << ")" << std::endl;
        } else {
            std::cout << "[VALIDATION] GPS mode (inliers=" << inliers << ")" << std::endl;
        }
    }

    // 7. Apply pose based on mode
    if( use_gps_mode && gt_loader_ && motion_model_.prev_time_ > 0 && time > motion_model_.prev_time_ ) {
        // GPS mode: Use GPS position + velocity
        Eigen::Vector3d p_gps_cur;
        Eigen::Quaterniond q_gps_dummy;

        if( gt_loader_->getPoseAt(time, p_gps_cur, q_gps_dummy) ) {
            // Compute GPS velocity
            Eigen::Vector3d p_gps_prev;
            Eigen::Quaterniond q_dummy2;
            gt_loader_->getPoseAt(motion_model_.prev_time_, p_gps_prev, q_dummy2);

            double dt = time - motion_model_.prev_time_;
            Eigen::Vector3d v_gps = (p_gps_cur - p_gps_prev) / dt;

            // Use GPS position for translation, keep IMU rotation
            Twc.translation() = p_gps_cur;
            pcurframe_->setTwc(Twc);
            pcurframe_->setVelocity(v_gps);
            motion_model_.updateMotionModelVelocity(v_gps, true);
        }
    } else {
        // Vision mode: Use PnP pose with velocity correction
        if( motion_model_.prev_time_ > 0 && time > motion_model_.prev_time_ ) {
            Eigen::Vector3d p_cur = pcurframe_->getTwc().translation();
            Eigen::Vector3d p_prev = motion_model_.prevTwc_.translation();
            double dt = time - motion_model_.prev_time_;
            Eigen::Vector3d v_visual = (p_cur - p_prev) / dt;
            pcurframe_->setVelocity(v_visual);
        }
    }

    // Update motion model from estimated pose (must be AFTER velocity correction)
    motion_model_.updateMotionModel(pcurframe_->Twc_, time);

    std::cout << "[trackStereo] Before checkNewKfReq() for frame " << pcurframe_->id_ << std::endl;

    // 7. Keyframe decision (keyframe creation is handled in visualTracking())
    bool is_kf_req = checkNewKfReq();

    std::cout << "[trackStereo] After checkNewKfReq() for frame " << pcurframe_->id_ << " → is_kf_req=" << is_kf_req << std::endl;

    if( pslamstate_->debug_ || pslamstate_->log_timings_ )
        Profiler::StopAndDisplay(pslamstate_->debug_, "1.FE_Track-Stereo");

    return is_kf_req;
}


// KLT Tracking with motion prior
void VisualFrontEnd::kltTracking()
{
    PROFILE_FUNCTION();

    if( pslamstate_->debug_ || pslamstate_->log_timings_ )
        Profiler::Start("2.FE_TM_KLT-Tracking");

    // Get current kps and init priors for tracking
    std::vector<int> v3dkpids, vkpids;
    std::vector<cv::Point2f> v3dkps, v3dpriors, vkps, vpriors;
    std::vector<bool> vkpis3d;

    // First we're gonna track 3d kps on only 2 levels
    v3dkpids.reserve(pcurframe_->nb3dkps_);
    v3dkps.reserve(pcurframe_->nb3dkps_);
    v3dpriors.reserve(pcurframe_->nb3dkps_);

    // Then we'll track 2d kps on full pyramid levels
    vkpids.reserve(pcurframe_->nbkps_);
    vkps.reserve(pcurframe_->nbkps_);
    vpriors.reserve(pcurframe_->nbkps_);

    vkpis3d.reserve(pcurframe_->nbkps_);


    // Front-End is thread-safe so we can direclty access curframe's kps
    for( const auto &it : pcurframe_->mapkps_ ) 
    {
        auto &kp = it.second;

        // Init prior px pos. from motion model
        if( pslamstate_->klt_use_prior_ )
        {
            if( kp.is3d_ ) 
            {
                cv::Point2f projpx = pcurframe_->projWorldToImageDist(pmap_->map_plms_.at(kp.lmid_)->getPoint());

                // Add prior if projected into image
                if( pcurframe_->isInImage(projpx) ) 
                {
                    v3dkps.push_back(kp.px_);
                    v3dpriors.push_back(projpx);
                    v3dkpids.push_back(kp.lmid_);

                    vkpis3d.push_back(true);
                    continue;
                }
            }
        }

        // For other kps init prior with prev px pos.
        vkpids.push_back(kp.lmid_);
        vkps.push_back(kp.px_);
        vpriors.push_back(kp.px_);
    }

    // 1st track 3d kps if using prior
    if( pslamstate_->klt_use_prior_ && !v3dpriors.empty() ) 
    {
        int nbpyrlvl = 1;

        // Good / bad kps vector
        std::vector<bool> vkpstatus;

        auto vprior = v3dpriors;

        ptracker_->fbKltTracking(
                    prev_pyr_, 
                    cur_pyr_, 
                    pslamstate_->nklt_win_size_, 
                    nbpyrlvl, 
                    pslamstate_->nklt_err_, 
                    pslamstate_->fmax_fbklt_dist_, 
                    v3dkps, 
                    v3dpriors, 
                    vkpstatus);

        size_t nbgood = 0;
        size_t nbkps = v3dkps.size();

        for(size_t i = 0 ; i < nbkps  ; i++ ) 
        {
            if( vkpstatus.at(i) ) {
                pcurframe_->updateKeypoint(v3dkpids.at(i), v3dpriors.at(i));
                nbgood++;
            } else {
                // If tracking failed, gonna try on full pyramid size
                vkpids.push_back(v3dkpids.at(i));
                vkps.push_back(v3dkps.at(i));
                vpriors.push_back(v3dpriors.at(i));
            }
        }

        if( pslamstate_->debug_ ) {
            std::cout << "\n >>> KLT Tracking w. priors : " << nbgood;
            std::cout << " out of " << nbkps << " kps tracked!\n";
        }

        if( nbgood < 0.33 * nbkps ) {
            // Motion model might be quite wrong, P3P is recommended next
            // and not using any prior
            bp3preq_ = true;
            vpriors = vkps;
        }
    }

    // 2nd track other kps if any
    if( !vkps.empty() ) 
    {
        // Good / bad kps vector
        std::vector<bool> vkpstatus;

        ptracker_->fbKltTracking(
                    prev_pyr_, 
                    cur_pyr_, 
                    pslamstate_->nklt_win_size_, 
                    pslamstate_->nklt_pyr_lvl_, 
                    pslamstate_->nklt_err_, 
                    pslamstate_->fmax_fbklt_dist_, 
                    vkps, 
                    vpriors, 
                    vkpstatus);
        
        size_t nbgood = 0;
        size_t nbkps = vkps.size();

        for(size_t i = 0 ; i < nbkps  ; i++ ) 
        {
            if( vkpstatus.at(i) ) {
                pcurframe_->updateKeypoint(vkpids.at(i), vpriors.at(i));
                nbgood++;
            } else {
                // MapManager is responsible for all the removing operations
                pmap_->removeObsFromCurFrameById(vkpids.at(i));
            }
        }

        if( pslamstate_->debug_ ) {
            std::cout << "\n >>> KLT Tracking no prior : " << nbgood;
            std::cout << " out of " << nbkps << " kps tracked!\n";
        }
    } 
    
    if( pslamstate_->debug_ || pslamstate_->log_timings_ )
        Profiler::StopAndDisplay(pslamstate_->debug_, "2.FE_TM_KLT-Tracking");
}


void VisualFrontEnd::kltTrackingFromKF()
{
    PROFILE_FUNCTION();

    if( pslamstate_->debug_ || pslamstate_->log_timings_ )
        Profiler::Start("2.FE_TM_KLT-Tracking-from-KF");

    // Get current kps and init priors for tracking
    std::vector<int> v3dkpids, vkpids;
    std::vector<cv::Point2f> v3dkps, v3dpriors, vkps, vpriors;
    std::vector<bool> vkpis3d;

    // First we're gonna track 3d kps on only 2 levels
    v3dkpids.reserve(pcurframe_->nb3dkps_);
    v3dkps.reserve(pcurframe_->nb3dkps_);
    v3dpriors.reserve(pcurframe_->nb3dkps_);

    // Then we'll track 2d kps on full pyramid levels
    vkpids.reserve(pcurframe_->nbkps_);
    vkps.reserve(pcurframe_->nbkps_);
    vpriors.reserve(pcurframe_->nbkps_);

    vkpis3d.reserve(pcurframe_->nbkps_);

    // Get prev KF
    auto pkf = pmap_->map_pkfs_.at(pcurframe_->kfid_);

    if( pkf == nullptr ) {
        return;
    }

    std::vector<int> vbadids;
    vbadids.reserve(pcurframe_->nbkps_ * 0.2);


    // Front-End is thread-safe so we can direclty access curframe's kps
    for( const auto &it : pcurframe_->mapkps_ ) 
    {
        auto &kp = it.second;

        auto kfkpit = pkf->mapkps_.find(kp.lmid_);
        if( kfkpit == pkf->mapkps_.end() ) {
            vbadids.push_back(kp.lmid_);
            continue;
        }

        // Init prior px pos. from motion model
        if( pslamstate_->klt_use_prior_ )
        {
            if( kp.is3d_ ) 
            {
                cv::Point2f projpx = pcurframe_->projWorldToImageDist(pmap_->map_plms_.at(kp.lmid_)->getPoint());

                // Add prior if projected into image
                if( pcurframe_->isInImage(projpx) ) 
                {
                    v3dkps.push_back(kfkpit->second.px_);
                    v3dpriors.push_back(projpx);
                    v3dkpids.push_back(kp.lmid_);

                    vkpis3d.push_back(true);
                    continue;
                }
            }
        }

        // For other kps init prior with prev px pos.
        vkpids.push_back(kp.lmid_);
        vkps.push_back(kfkpit->second.px_);
        vpriors.push_back(kp.px_);
    }

    for( const auto &badid : vbadids ) {
        // MapManager is responsible for all the removing operations
        pmap_->removeObsFromCurFrameById(badid);
    }

    // 1st track 3d kps if using prior
    if( pslamstate_->klt_use_prior_ && !v3dpriors.empty() ) 
    {
        int nbpyrlvl = 1;

        // Good / bad kps vector
        std::vector<bool> vkpstatus;

        auto vprior = v3dpriors;

        ptracker_->fbKltTracking(
                    kf_pyr_, 
                    cur_pyr_, 
                    pslamstate_->nklt_win_size_, 
                    nbpyrlvl, 
                    pslamstate_->nklt_err_, 
                    pslamstate_->fmax_fbklt_dist_, 
                    v3dkps, 
                    v3dpriors, 
                    vkpstatus);

        size_t nbgood = 0;
        size_t nbkps = v3dkps.size();

        for(size_t i = 0 ; i < nbkps  ; i++ ) 
        {
            if( vkpstatus.at(i) ) {
                pcurframe_->updateKeypoint(v3dkpids.at(i), v3dpriors.at(i));
                nbgood++;
            } else {
                // If tracking failed, gonna try on full pyramid size
                vkpids.push_back(v3dkpids.at(i));
                vkps.push_back(v3dkps.at(i));
                vpriors.push_back(pcurframe_->mapkps_.at(v3dkpids.at(i)).px_);
            }
        }

        if( pslamstate_->debug_ ) {
            std::cout << "\n >>> KLT Tracking w. priors : " << nbgood;
            std::cout << " out of " << nbkps << " kps tracked!\n";
        }

        if( nbgood < 0.33 * nbkps ) {
            // Motion model might be quite wrong, P3P is recommended next
            // and not using any prior
            bp3preq_ = true;
            vpriors = vkps;
        }
    }

    // 2nd track other kps if any
    if( !vkps.empty() ) 
    {
        // Good / bad kps vector
        std::vector<bool> vkpstatus;

        ptracker_->fbKltTracking(
                    kf_pyr_, 
                    cur_pyr_, 
                    pslamstate_->nklt_win_size_, 
                    pslamstate_->nklt_pyr_lvl_, 
                    pslamstate_->nklt_err_, 
                    pslamstate_->fmax_fbklt_dist_, 
                    vkps, 
                    vpriors, 
                    vkpstatus);
        
        size_t nbgood = 0;
        size_t nbkps = vkps.size();

        for(size_t i = 0 ; i < nbkps  ; i++ ) 
        {
            if( vkpstatus.at(i) ) {
                pcurframe_->updateKeypoint(vkpids.at(i), vpriors.at(i));
                nbgood++;
            } else {
                // MapManager is responsible for all the removing operations
                pmap_->removeObsFromCurFrameById(vkpids.at(i));
            }
        }

        if( pslamstate_->debug_ ) {
            std::cout << "\n >>> KLT Tracking no prior : " << nbgood;
            std::cout << " out of " << nbkps << " kps tracked!\n";
        }
    } 
    
    if( pslamstate_->debug_ || pslamstate_->log_timings_ )
        Profiler::StopAndDisplay(pslamstate_->debug_, "2.FE_TM_KLT-Tracking");
}


// This function apply a 2d-2d based outliers filtering
void VisualFrontEnd::epipolar2d2dFiltering()
{
    PROFILE_FUNCTION();

    if( pslamstate_->debug_ || pslamstate_->log_timings_ )
        Profiler::Start("2.FE_TM_EpipolarFiltering");
    
    // Get prev. KF (direct access as Front-End is thread safe)
    auto pkf = pmap_->map_pkfs_.at(pcurframe_->kfid_);

    if( pkf == nullptr ) {
        std::cerr << "\nERROR! Previous Kf does not exist yet (epipolar2d2d()).\n";
        exit(-1);
    }

    // Get cur. Frame nb kps
    size_t nbkps = pcurframe_->nbkps_;

    if( nbkps < 8 ) {
        if( pslamstate_->debug_ )
            std::cout << "\nNot enough kps to compute Essential Matrix\n";
        return;
    }

    // Setup Essential Matrix computation for OpenGV-based filtering
    std::vector<int> vkpsids, voutliersidx;
    vkpsids.reserve(nbkps);
    voutliersidx.reserve(nbkps);

    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > vkfbvs, vcurbvs;
    vkfbvs.reserve(nbkps);
    vcurbvs.reserve(nbkps);
    
    size_t nbparallax = 0;
    float avg_parallax = 0.;

    // In stereo mode, we consider 3d kps as better tracks and therefore
    // use only them for computing E with RANSAC, 2d kps are then removed based
    // on the resulting Fundamental Mat.
    bool epifrom3dkps = false;
    if( pslamstate_->stereo_ && pcurframe_->nb3dkps_ > 30 ) {
        epifrom3dkps = true;
    }

    // Compute rotation compensated parallax
    Eigen::Matrix3d Rkfcur = pkf->getRcw() * pcurframe_->getRwc();

    // Init bearing vectors and check parallax
    for( const auto &it : pcurframe_->mapkps_ ) {

        if( epifrom3dkps ) {
            if( !it.second.is3d_ ) {
                continue;
            }
        }

        auto &kp = it.second;

        // Get the prev. KF related kp if it exists
        auto kfkp = pkf->getKeypointById(kp.lmid_);

        if( kfkp.lmid_ != kp.lmid_ ) {
            continue;
        }

        // Store the bvs and their ids
        vkfbvs.push_back(kfkp.bv_);
        vcurbvs.push_back(kp.bv_);
        vkpsids.push_back(kp.lmid_);

        cv::Point2f rotpx = pkf->projCamToImage(Rkfcur * kp.bv_);

        // Compute parallax
        avg_parallax += cv::norm(rotpx - kfkp.unpx_);
        nbparallax++;
    }

    if( nbkps < 8 ) {
        if( pslamstate_->debug_ )
            std::cout << "\nNot enough kps to compute Essential Matrix\n";
        return;
    }

    // Average parallax
    avg_parallax /= nbparallax;

    if( avg_parallax < 2. * pslamstate_->fransac_err_ ) {
        if( pslamstate_->debug_ )
            std::cout << "\n \t>>> Not enough parallax (" << avg_parallax 
                << " px) to compute 5-pt Essential Matrix\n";
        return;
    }

    bool do_optimize = false;

    // In monocular case, we'll use the resulting motion if tracking is poor
    if( pslamstate_->mono_ && pmap_->nbkfs_ > 2 
        && pcurframe_->nb3dkps_ < 30 ) 
    {
        do_optimize = true;
    }

    Eigen::Matrix3d Rkfc;
    Eigen::Vector3d tkfc;

    if( pslamstate_->debug_ ) {
        std::cout << "\n \t>>> 5-pt EssentialMatrix Ransac :";
        std::cout << "\n \t>>> only on 3d kps : " << epifrom3dkps;
        std::cout << "\n \t>>> nb pts : " << nbkps;
        std::cout << " / avg. parallax : " << avg_parallax;
        std::cout << " / nransac_iter_ : " << pslamstate_->nransac_iter_;
        std::cout << " / fransac_err_ : " << pslamstate_->fransac_err_;
        std::cout << "\n\n";
    }
    
    bool success = 
        MultiViewGeometry::compute5ptEssentialMatrix(
                    vkfbvs, vcurbvs, 
                    pslamstate_->nransac_iter_, 
                    pslamstate_->fransac_err_, 
                    do_optimize, 
                    pslamstate_->bdo_random, 
                    pcurframe_->pcalib_leftcam_->fx_, 
                    pcurframe_->pcalib_leftcam_->fy_, 
                    Rkfc, tkfc, 
                    voutliersidx);

    if( pslamstate_->debug_ )
        std::cout << "\n \t>>> Epipolar nb outliers : " << voutliersidx.size();

    if( !success) {
        if( pslamstate_->debug_ )
            std::cout << "\n \t>>> No pose could be computed from 5-pt EssentialMatrix\n";
        return;
    }

    if( voutliersidx.size() > 0.5 * vkfbvs.size() ) {
        if( pslamstate_->debug_ )
            std::cout << "\n \t>>> Too many outliers, skipping as might be degenerate case\n";
        return;
    }

    // Remove outliers
    for( const auto & idx : voutliersidx ) {
        // MapManager is responsible for all the removing operations.
        pmap_->removeObsFromCurFrameById(vkpsids.at(idx));
    }

    // In case we wanted to use the resulting motion 
    // (mono mode - can help when tracking is poor)
    if( do_optimize && pmap_->nbkfs_ > 2 ) 
    {
        // Get motion model translation scale from last KF
        Sophus::SE3d Tkfw = pkf->getTcw();
        Sophus::SE3d Tkfcur = Tkfw * pcurframe_->getTwc();

        double scale = Tkfcur.translation().norm();
        tkfc.normalize();

        // Update current pose with Essential Mat. relative motion
        // and current trans. scale
        Sophus::SE3d Tkfc(Rkfc, scale * tkfc);

        pcurframe_->setTwc(pkf->getTwc() * Tkfc);
    }

    // In case we only used 3d kps for computing E (stereo mode)
    if( epifrom3dkps ) {

        if( pslamstate_->debug_ )
            std::cout << "\n Applying found Essential Mat to 2D kps!\n";

        Sophus::SE3d Tidentity;
        Sophus::SE3d Tkfcur(Rkfc, tkfc);

        Eigen::Matrix3d Fkfcur = MultiViewGeometry::computeFundamentalMat12(Tidentity, Tkfcur, pcurframe_->pcalib_leftcam_->K_);

        std::vector<int> vbadkpids;
        vbadkpids.reserve(pcurframe_->nb2dkps_);

        for( const auto &it : pcurframe_->mapkps_ ) 
        {
            if( it.second.is3d_ ) {
                continue;
            }

            auto &kp = it.second;

            // Get the prev. KF related kp if it exists
            auto kfkp = pkf->getKeypointById(kp.lmid_);

            // Normalized coord.
            Eigen::Vector3d curpt(kp.unpx_.x, kp.unpx_.y, 1.);
            Eigen::Vector3d kfpt(kfkp.unpx_.x, kfkp.unpx_.y, 1.);

            float epi_err = MultiViewGeometry::computeSampsonDistance(Fkfcur, curpt, kfpt);

            if( epi_err > pslamstate_->fransac_err_ ) {
                vbadkpids.push_back(kp.lmid_);
            }
        }

        for( const auto & kpid : vbadkpids ) {
            pmap_->removeObsFromCurFrameById(kpid);
        }

        if( pslamstate_->debug_ )
            std::cout << "\n Nb of 2d kps removed : " << vbadkpids.size() << " \n";
    }

    if( pslamstate_->debug_ || pslamstate_->log_timings_ )
        Profiler::StopAndDisplay(pslamstate_->debug_, "2.FE_TM_EpipolarFiltering");
}


void VisualFrontEnd::computePose()
{
    PROFILE_FUNCTION();

    if( pslamstate_->debug_ || pslamstate_->log_timings_ )
        Profiler::Start("2.FE_TM_computePose");

    // Get cur nb of 3D kps    
    size_t nb3dkps = pcurframe_->nb3dkps_;

    if( nb3dkps < 4 ) {
        if( pslamstate_->debug_ )
            std::cout << "\n \t>>> Not enough kps to compute P3P / PnP\n";
        return;
    }

    // Setup P3P-Ransac computation for OpenGV-based Pose estimation
    // + motion-only BA with Ceres
    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > vbvs, vwpts;
    std::vector<int> vkpids, voutliersidx, vscales;

    std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> > vkps;

    vbvs.reserve(nb3dkps);
    vwpts.reserve(nb3dkps);
    vkpids.reserve(nb3dkps);
    voutliersidx.reserve(nb3dkps);

    vkps.reserve(nb3dkps);
    vscales.reserve(nb3dkps);

    bool bdop3p = bp3preq_ || pslamstate_->dop3p_;

    // Store every 3D bvs, MPs and their related ids
    for( const auto &it : pcurframe_->mapkps_ ) 
    {
        if( !it.second.is3d_ ) {
            continue;
        }

        auto &kp = it.second;
        // auto plm = pmap_->getMapPoint(kp.lmid_);
        auto plm = pmap_->map_plms_.at(kp.lmid_);
        if( plm == nullptr ) {
            continue;
        }

        if( bdop3p ) {
            vbvs.push_back(kp.bv_);
        }

        vkps.push_back(Eigen::Vector2d(kp.unpx_.x, kp.unpx_.y));
        vwpts.push_back(plm->getPoint());
        vscales.push_back(kp.scale_);
        vkpids.push_back(kp.lmid_);
    }

    Sophus::SE3d Twc = pcurframe_->getTwc();
    bool do_optimize = false;
    bool success = false;

    if( bdop3p ) 
    {
        if( pslamstate_->debug_ ) {
            std::cout << "\n \t>>>P3P Ransac : ";
            std::cout << "\n \t>>> nb 3d pts : " << nb3dkps;
            std::cout << " / nransac_iter_ : " << pslamstate_->nransac_iter_;
            std::cout << " / fransac_err_ : " << pslamstate_->fransac_err_;
            std::cout << "\n\n";
        }

        // Only effective with OpenGV
        bool use_lmeds = true;

        success = 
            MultiViewGeometry::p3pRansac(
                            vbvs, vwpts, 
                            pslamstate_->nransac_iter_, 
                            pslamstate_->fransac_err_, 
                            do_optimize, 
                            pslamstate_->bdo_random, 
                            pcurframe_->pcalib_leftcam_->fx_, 
                            pcurframe_->pcalib_leftcam_->fy_, 
                            Twc,
                            voutliersidx,
                            use_lmeds);

        if( pslamstate_->debug_ )
            std::cout << "\n \t>>> P3P-LMeds nb outliers : " << voutliersidx.size();

        // Check that pose estim. was good enough
        size_t nbinliers = vwpts.size() - voutliersidx.size();
        last_nbinliers_ = nbinliers;  // Store for validation layer

        if( !success
            || nbinliers < 5
            || Twc.translation().array().isInf().any()
            || Twc.translation().array().isNaN().any() )
        {
            if( pslamstate_->debug_ )
                std::cout << "\n \t>>> Not enough inliers for reliable pose est. Resetting KF state\n";

            resetFrame();

            return;
        } 

        // Pose seems to be OK!

        // Update frame pose
        pcurframe_->setTwc(Twc);

        // Remove outliers before PnP refinement (a bit dirty)
        int k = 0;
        for( const auto &idx : voutliersidx ) {
            // MapManager is responsible for all removing operations
            pmap_->removeObsFromCurFrameById(vkpids.at(idx-k));
            vkps.erase(vkps.begin() + idx - k);
            vwpts.erase(vwpts.begin() + idx - k);
            vkpids.erase(vkpids.begin() + idx - k);
            vscales.erase(vscales.begin() + idx - k);
            k++;
        }

        // Clear before robust PnP refinement using Ceres
        voutliersidx.clear();
    }

    // Ceres-based PnP (motion-only BA)
    bool buse_robust = true;
    bool bapply_l2_after_robust = pslamstate_->apply_l2_after_robust_;
    
    size_t nbmaxiters = 5;

    success =
        MultiViewGeometry::ceresPnP(
                        vkps, vwpts, 
                        vscales,
                        Twc, 
                        nbmaxiters, 
                        pslamstate_->robust_mono_th_, 
                        buse_robust, 
                        bapply_l2_after_robust,
                        pcurframe_->pcalib_leftcam_->fx_, pcurframe_->pcalib_leftcam_->fy_,
                        pcurframe_->pcalib_leftcam_->cx_, pcurframe_->pcalib_leftcam_->cy_,
                        voutliersidx);
    
    // Check that pose estim. was good enough
    size_t nbinliers = vwpts.size() - voutliersidx.size();
    last_nbinliers_ = nbinliers;  // Store for validation layer

    if( pslamstate_->debug_ )
        std::cout << "\n \t>>> Ceres PnP nb outliers : " << voutliersidx.size();

    // SCI-LOG-4: PnP solver results (all frames for debugging)
    std::cout << "[PNP_RESULT] frame=" << pcurframe_->id_
              << " Z=" << Twc.translation().z()
              << " inliers=" << nbinliers
              << " outliers=" << voutliersidx.size()
              << " success=" << success << std::endl;

    if( !success
        || nbinliers < 5
        || voutliersidx.size() > 0.5 * vwpts.size()
        || Twc.translation().array().isInf().any()
        || Twc.translation().array().isNaN().any() )
    {
        if( !bdop3p ) {
            // Weird results, skipping here and applying p3p next
            bp3preq_ = true;
        }
        else if( pslamstate_->mono_ ) {

            if( pslamstate_->debug_ )
                std::cout << "\n \t>>> Not enough inliers for reliable pose est. Resetting KF state\n";

            resetFrame();
        } 
        // else {
            // resetFrame();
            // motion_model_.reset();
        // }

        return;
    } 

    // Pose seems to be OK!

    // Update frame pose
    pcurframe_->setTwc(Twc);

    // Set p3p req to false as it is triggered either because
    // of bad PnP or by bad klt tracking
    bp3preq_ = false;

    // Remove outliers
    for( const auto & idx : voutliersidx ) {
        // MapManager is responsible for all removing operations
        pmap_->removeObsFromCurFrameById(vkpids.at(idx));
    }

    if( pslamstate_->debug_ || pslamstate_->log_timings_ )
        Profiler::StopAndDisplay(pslamstate_->debug_, "2.FE_TM_computePose");
}



bool VisualFrontEnd::checkReadyForInit()
{
    double avg_rot_parallax = computeParallax(pcurframe_->kfid_, false);

    std::cout << "\n \t>>> Init current parallax (" << avg_rot_parallax <<" px)\n"; 

    if( avg_rot_parallax > pslamstate_->finit_parallax_ ) {
        auto cb = std::chrono::high_resolution_clock::now();
        
        // Get prev. KF
        auto pkf = pmap_->map_pkfs_.at(pcurframe_->kfid_);
        if( pkf == nullptr ) {
            return false;
        }

        // Get cur. Frame nb kps
        size_t nbkps = pcurframe_->nbkps_;

        if( nbkps < 8 ) {
            std::cout << "\nNot enough kps to compute 5-pt Essential Matrix\n";
            return false;
        }

        // Setup Essential Matrix computation for OpenGV-based filtering
        std::vector<int> vkpsids, voutliersidx;
        vkpsids.reserve(nbkps);
        voutliersidx.reserve(nbkps);

        std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > vkfbvs, vcurbvs;
        vkfbvs.reserve(nbkps);
        vcurbvs.reserve(nbkps);

        Eigen::Matrix3d Rkfcur = pkf->getTcw().rotationMatrix() * pcurframe_->getTwc().rotationMatrix();
        int nbparallax = 0;
        float avg_rot_parallax = 0.;

        // Get bvs and compute the rotation compensated parallax for all cur kps
        // for( const auto &kp : pcurframe_->getKeypoints() ) {
        for( const auto &it : pcurframe_->mapkps_ ) {
            auto &kp = it.second;
            // Get the prev. KF related kp if it exists
            auto kfkp = pkf->getKeypointById(kp.lmid_);

            if( kfkp.lmid_ != kp.lmid_ ) {
                continue;
            }

            // Store the bvs and their ids
            vkfbvs.push_back(kfkp.bv_);
            vcurbvs.push_back(kp.bv_);
            vkpsids.push_back(kp.lmid_);

            // Compute rotation compensated parallax
            Eigen::Vector3d rotbv = Rkfcur * kp.bv_;

            Eigen::Vector3d unpx = pcurframe_->pcalib_leftcam_->K_ * rotbv;
            cv::Point2f rotpx(unpx.x() / unpx.z(), unpx.y() / unpx.z());

            avg_rot_parallax += cv::norm(rotpx - kfkp.unpx_);
            nbparallax++;
        }

        if( nbparallax < 8 ) {
            std::cout << "\nNot enough prev KF kps to compute 5-pt Essential Matrix\n";
            return false;
        }

        // Average parallax
        avg_rot_parallax /= (nbparallax);

        if( avg_rot_parallax < pslamstate_->finit_parallax_ ) {
            std::cout << "\n \t>>> Not enough parallax (" << avg_rot_parallax <<" px) to compute 5-pt Essential Matrix\n";
            return false;
        }

        bool do_optimize = true;

        Eigen::Matrix3d Rkfc;
        Eigen::Vector3d tkfc;
        Rkfc.setIdentity();
        tkfc.setZero();

        std::cout << "\n \t>>> 5-pt EssentialMatrix Ransac :";
        std::cout << "\n \t>>> nb pts : " << nbkps;
        std::cout << " / avg. parallax : " << avg_rot_parallax;
        std::cout << " / nransac_iter_ : " << pslamstate_->nransac_iter_;
        std::cout << " / fransac_err_ : " << pslamstate_->fransac_err_;
        std::cout << " / bdo_random : " << pslamstate_->bdo_random;
        std::cout << "\n\n";
        
        bool success = 
            MultiViewGeometry::compute5ptEssentialMatrix
                    (vkfbvs, vcurbvs, pslamstate_->nransac_iter_, pslamstate_->fransac_err_, 
                    do_optimize, pslamstate_->bdo_random, 
                    pcurframe_->pcalib_leftcam_->fx_, 
                    pcurframe_->pcalib_leftcam_->fy_, 
                    Rkfc, tkfc, 
                    voutliersidx);

        std::cout << "\n \t>>> Epipolar nb outliers : " << voutliersidx.size();

        if( !success ) {
            std::cout << "\n \t>>> No pose could be computed from 5-pt EssentialMatrix\n";
            return false;
        }

        // Remove outliers from cur. Frame
        for( const auto & idx : voutliersidx ) {
            // MapManager is responsible for all the removing operations.
            pmap_->removeObsFromCurFrameById(vkpsids.at(idx));
        }

        // Arbitrary scale
        tkfc.normalize();
        tkfc = tkfc.eval() * 0.25;

        std::cout << "\n \t>>> Essential Mat init : " << tkfc.transpose();

        pcurframe_->setTwc(Rkfc, tkfc);
        
        auto ce = std::chrono::high_resolution_clock::now();
        std::cout << "\n \t>>> Essential Mat Intialization run time : " 
            << std::chrono::duration_cast<std::chrono::milliseconds>(ce-cb).count()
            << "[ms]" << std::endl;

        return true;
    }

    return false;
}

bool VisualFrontEnd::checkNewKfReq()
{
    std::cout << "[ENTER] checkNewKfReq() for frame " << pcurframe_->id_ << std::endl;

    if( pslamstate_->debug_ || pslamstate_->log_timings_ )
        Profiler::Start("2.FE_TM_checkNewKfReq");

    // FORCE FIRST KEYFRAME: If map is empty, always create first keyframe
    if( pmap_->map_pkfs_.empty() ) {
        std::cout << "  [FIRST KF] Map is empty → Creating first keyframe!" << std::endl;
        last_keyframe_time_ = pcurframe_->img_time_; // Faza 3: Initialize watchdog
        return true;
    }

    // Faza 3: WATCHDOG - Anti-starvation (prevent excessive keyframe rate)
    if( pslamstate_->stereo_ && last_keyframe_time_ > 0 ) {
        double time_since_last_kf = pcurframe_->img_time_ - last_keyframe_time_;
        if( time_since_last_kf < 1.0 ) {
            if( pslamstate_->debug_ )
                std::cout << "  [WATCHDOG] REJECTING keyframe - too soon since last KF: "
                          << time_since_last_kf << "s < 1.0s" << std::endl;
            return false;
        }
    }

    // Get prev. KF
    auto pkfit = pmap_->map_pkfs_.find(pcurframe_->kfid_);

    if( pkfit == pmap_->map_pkfs_.end() ) {
        std::cout << "[checkNewKfReq] ERROR: Previous KF #" << pcurframe_->kfid_ << " not found in map!" << std::endl;
        std::cout << "  map_pkfs_ size: " << pmap_->map_pkfs_.size() << std::endl;
        return false; // Should not happen
    }
    auto pkf = pkfit->second;

    // Compute median parallax
    double med_rot_parallax = 0.;

    // unrot : false / median : true / only_2d : false
    med_rot_parallax = computeParallax(pkf->kfid_, true, true, false);

    // Id diff with last KF
    int nbimfromkf = pcurframe_->id_-pkf->id_;

    if( pslamstate_->debug_ ) {
        std::cout << "\n[KF-DEBUG] Frame " << pcurframe_->id_ << " vs KF #" << pkf->kfid_
                  << " (id=" << pkf->id_ << "):" << std::endl;
        std::cout << "  nbimfromkf=" << nbimfromkf << std::endl;
        std::cout << "  nb3dkps_=" << pcurframe_->nb3dkps_ << " (prev KF: " << pkf->nb3dkps_ << ")" << std::endl;
        std::cout << "  noccupcells_=" << pcurframe_->noccupcells_ << " vs 0.33*nbmaxkps_=" << (0.33 * pslamstate_->nbmaxkps_) << std::endl;
    }

    if( pcurframe_->noccupcells_ < 0.33 * pslamstate_->nbmaxkps_
        && nbimfromkf >= 5
        && !pslamstate_->blocalba_is_on_ )
    {
        if( pslamstate_->debug_ )
            std::cout << "  [CONDITION 1] TRUE → Creating keyframe (low occupancy)" << std::endl;
        last_keyframe_time_ = pcurframe_->img_time_; // Faza 3: Update watchdog
        return true;
    }

    if( pcurframe_->nb3dkps_ < 20 &&
        nbimfromkf >= 2 )
    {
        if( pslamstate_->debug_ )
            std::cout << "  [CONDITION 2] TRUE → Creating keyframe (low 3D keypoints)" << std::endl;
        last_keyframe_time_ = pcurframe_->img_time_; // Faza 3: Update watchdog
        return true;
    }

    if( pcurframe_->nb3dkps_ > 0.5 * pslamstate_->nbmaxkps_
        && (pslamstate_->blocalba_is_on_ || nbimfromkf < 2) )
    {
        if( pslamstate_->debug_ )
            std::cout << "  [CONDITION 3] FALSE → Rejecting keyframe (too many 3D keypoints)" << std::endl;
        return false;
    }

    // Time diff since last KF in sec.
    double time_diff = pcurframe_->img_time_ - pkf->img_time_;

    // Faza 3: Changed time threshold from 1.0s to 5.0s (less aggressive)
    if( pslamstate_->stereo_ && time_diff > 5.0
        && !pslamstate_->blocalba_is_on_ )
    {
        if( pslamstate_->debug_ )
            std::cout << "  [CONDITION 4] TRUE → Creating keyframe (stereo time diff: " << time_diff << "s)" << std::endl;
        last_keyframe_time_ = pcurframe_->img_time_; // Faza 3: Update watchdog
        return true;
    }

    // Faza 3: Changed frame diff from 2 to 10 (less aggressive)
    bool cx = med_rot_parallax >= pslamstate_->finit_parallax_ / 2.
        || (pslamstate_->stereo_ && !pslamstate_->blocalba_is_on_ && pcurframe_->id_-pkf->id_ > 10);

    bool c0 = med_rot_parallax >= pslamstate_->finit_parallax_;
    bool c1 = pcurframe_->nb3dkps_ < 0.75 * pkf->nb3dkps_;
    bool c2 = pcurframe_->noccupcells_ < 0.5 * pslamstate_->nbmaxkps_
                && pcurframe_->nb3dkps_ < 0.85 * pkf->nb3dkps_
                && !pslamstate_->blocalba_is_on_;

    bool bkfreq = (c0 || c1 || c2) && cx;

    if( pslamstate_->debug_ ) {
        std::cout << "  time_diff=" << time_diff << "s" << std::endl;
        std::cout << "  med_rot_parallax=" << med_rot_parallax << " vs finit_parallax_/2=" << (pslamstate_->finit_parallax_ / 2.) << std::endl;
        std::cout << "  cx=" << cx << ", c0=" << c0 << ", c1=" << c1 << ", c2=" << c2 << std::endl;
        std::cout << "  bkfreq=" << bkfreq << " → " << (bkfreq ? "CREATING KEYFRAME" : "NOT creating keyframe") << std::endl;
    }

    if( pslamstate_->debug_ || pslamstate_->log_timings_ )
        Profiler::StopAndDisplay(pslamstate_->debug_, "2.FE_TM_checkNewKfReq");

    // Faza 3: Update watchdog timer when keyframe will be created
    if( bkfreq ) {
        last_keyframe_time_ = pcurframe_->img_time_;
    }

    return bkfreq;
}


// This function computes the parallax (in px.) between cur. Frame 
// and the provided KF id.
float VisualFrontEnd::computeParallax(const int kfid, bool do_unrot, bool bmedian, bool b2donly)
{
    // Get prev. KF
    auto pkfit = pmap_->map_pkfs_.find(kfid);
    
    if( pkfit == pmap_->map_pkfs_.end() ) {
        if( pslamstate_->debug_ )
            std::cout << "\n[Visual Front End] Error in computeParallax ! Prev KF #" 
                    << kfid << " does not exist!\n";
        return 0.;
    }

    // Compute relative rotation between cur Frame 
    // and prev. KF if required
    Eigen::Matrix3d Rkfcur(Eigen::Matrix3d::Identity());
    if( do_unrot ) {
        Eigen::Matrix3d Rkfw = pkfit->second->getRcw();
        Eigen::Matrix3d Rwcur = pcurframe_->getRwc();
        Rkfcur = Rkfw * Rwcur;
    }

    // Compute parallax 
    float avg_parallax = 0.;
    int nbparallax = 0;

    std::set<float> set_parallax;

    // Compute parallax for all kps seen in prev. KF{
    for( const auto &it : pcurframe_->mapkps_ ) 
    {
        if( b2donly && it.second.is3d_ ) {
            continue;
        }

        auto &kp = it.second;
        // Get prev. KF kp if it exists
        auto kfkp = pkfit->second->getKeypointById(kp.lmid_);

        if( kfkp.lmid_ != kp.lmid_ ) {
            continue;
        }

        // Compute parallax with unpx pos.
        cv::Point2f unpx = kp.unpx_;

        // Rotate bv into KF cam frame and back project into image
        if( do_unrot ) {
            unpx = pkfit->second->projCamToImage(Rkfcur * kp.bv_);
        }

        // Compute rotation-compensated parallax
        float parallax = cv::norm(unpx - kfkp.unpx_);
        avg_parallax += parallax;
        nbparallax++;

        if( bmedian ) {
            set_parallax.insert(parallax);
        }
    }

    if( nbparallax == 0 ) {
        return 0.;
    }

    // Average parallax
    avg_parallax /= nbparallax;

    if( bmedian ) 
    {
        auto it = set_parallax.begin();
        std::advance(it, set_parallax.size() / 2);
        avg_parallax = *it;
    }

    return avg_parallax;
}

void VisualFrontEnd::preprocessImage(cv::Mat &img_raw)
{
    PROFILE_FUNCTION();

    if( pslamstate_->debug_ || pslamstate_->log_timings_ )
        Profiler::Start("2.FE_TM_preprocessImage");

    // Set cur raw img
    // left_raw_img_ = img_raw;

    // Update prev img
    if( !pslamstate_->btrack_keyframetoframe_ ) {
        // cur_img_.copyTo(prev_img_);
        cv::swap(cur_img_, prev_img_);
    }

    // Update cur img
    if( pslamstate_->use_clahe_ ) {
        ptracker_->pclahe_->apply(img_raw, cur_img_);
    } else {
        cur_img_ = img_raw;
    }

    // Pre-building the pyramid used for KLT speed-up
    if( pslamstate_->do_klt_ ) {

        // If tracking from prev image, swap the pyramid
        if( !cur_pyr_.empty() && !pslamstate_->btrack_keyframetoframe_ ) {
            prev_pyr_.swap(cur_pyr_);
        }

        cv::buildOpticalFlowPyramid(cur_img_, cur_pyr_, pslamstate_->klt_win_size_, pslamstate_->nklt_pyr_lvl_);
    }

    if( pslamstate_->debug_ || pslamstate_->log_timings_ )
        Profiler::StopAndDisplay(pslamstate_->debug_, "2.FE_TM_preprocessImage");
}


// Reset current Frame state
void VisualFrontEnd::resetFrame()
{
    auto mapkps = pcurframe_->mapkps_;
    for( const auto &kpit : mapkps ) {
        pmap_->removeObsFromCurFrameById(kpit.first);
    }
    pcurframe_->mapkps_.clear();
    pcurframe_->vgridkps_.clear();
    pcurframe_->vgridkps_.resize( pcurframe_->ngridcells_ );

    // Do not clear those as we keep the same pose
    // and hence keep a chance to retrack the previous map
    //
    // pcurframe_->map_covkfs_.clear();
    // pcurframe_->set_local_mapids_.clear();

    pcurframe_->nbkps_ = 0;
    pcurframe_->nb2dkps_ = 0;
    pcurframe_->nb3dkps_ = 0;
    pcurframe_->nb_stereo_kps_ = 0;

    pcurframe_->noccupcells_ = 0;
}

// Reset VisualFrontEnd
void VisualFrontEnd::reset()
{
    cur_img_.release();
    prev_img_.release();

    // left_raw_img_.release();

    cur_pyr_.clear();
    prev_pyr_.clear();
    kf_pyr_.clear();
}
