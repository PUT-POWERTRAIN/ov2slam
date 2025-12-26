#include "rerun_visualizer.hpp"
#include "gt_loader.hpp"
#include "map_point.hpp"
#include <iostream>
#include <limits>

RerunVisualizer::RerunVisualizer(const std::string& output_file) {
#ifdef ENABLE_RERUN
    try {
        rec_ = std::make_unique<rerun::RecordingStream>("ov2slam");

        if( output_file.empty() ) {
            // Live viewer mode
            (void)rec_->spawn();
            std::cout << "[Rerun] Live viewer mode\n";
        } else {
            // File recording mode
            rec_->save(output_file);
            std::cout << "[Rerun] Recording to file: " << output_file << "\n";
            std::cout << "[Rerun] View with: rerun " << output_file << "\n";
        }

        enabled_ = true;
    } catch(const std::exception& e) {
        std::cerr << "[Rerun] Failed to initialize: " << e.what() << "\n";
        enabled_ = false;
    }
#else
    std::cout << "[Rerun] Not compiled in (ENABLE_RERUN=OFF)\n";
#endif
}

RerunVisualizer::~RerunVisualizer() {
#ifdef ENABLE_RERUN
    if( rec_ ) {
        std::cout << "[Rerun] Flushing data to disk...\n";
        auto result = rec_->flush_blocking(std::numeric_limits<double>::infinity());
        if( result.is_err() ) {
            std::cerr << "[Rerun] WARNING: Flush failed: " << result.description << "\n";
        }
        std::cout << "[Rerun] Data flushed, shutting down...\n";
        rec_.reset();
    }
#endif
}

void RerunVisualizer::logPose(const Sophus::SE3d& Twc, const double time) {
#ifdef ENABLE_RERUN
    if(!enabled_ || !rec_) return;

    rec_->set_time_duration_secs("frame", time);

    Eigen::Vector3d t = Twc.translation();
    trajectory_.push_back(t);

    // Convert trajectory to Rerun positions
    std::vector<rerun::Position3D> positions;
    positions.reserve(trajectory_.size());
    for(const auto& pos : trajectory_) {
        positions.push_back(rerun::Position3D(pos.x(), pos.y(), pos.z()));
    }

    // Log trajectory as continuous line strip
    std::vector<rerun::LineStrip3D> strips;
    strips.push_back(rerun::LineStrip3D(positions));
    rec_->log("world/trajectory", rerun::LineStrips3D(std::move(strips)));

    // Log current position as dot
    std::vector<rerun::Position3D> current_pos;
    current_pos.push_back(rerun::Position3D(t.x(), t.y(), t.z()));
    rec_->log("world/camera", rerun::Points3D(std::move(current_pos))
        .with_radii({0.1f})
        .with_colors({rerun::Color(255, 0, 0)}));
#endif
}

void RerunVisualizer::logMapPoints(
    const std::unordered_map<int, std::shared_ptr<MapPoint>>& map_points,
    const double time
) {
#ifdef ENABLE_RERUN
    if(!enabled_ || !rec_) return;
    if(map_log_freq_ <= 0) return;  // Disabled

    frame_count_++;
    if(frame_count_ % map_log_freq_ != 0) return;  // Throttle

    // Set timeline for proper synchronization
    rec_->set_time_duration_secs("frame", time);

    std::vector<rerun::Position3D> positions;
    std::vector<rerun::Color> colors;

    for(const auto& [lmid, plm] : map_points) {
        if(!plm || !plm->is3d_) continue;

        Eigen::Vector3d pt = plm->ptxyz_;
        positions.push_back(rerun::Position3D(pt.x(), pt.y(), pt.z()));

        // Color from MapPoint (cv::Scalar to RGB)
        cv::Scalar c = plm->color_;
        colors.push_back(rerun::Color(uint8_t(c[0]), uint8_t(c[1]), uint8_t(c[2])));
    }

    rec_->log("world/map_points", rerun::Points3D(std::move(positions)).with_colors(std::move(colors)));
#endif
}

void RerunVisualizer::logKeyframe(const Sophus::SE3d& Twc, const cv::Mat& image, int kfid, const double time) {
#ifdef ENABLE_RERUN
    if(!enabled_ || !rec_) return;

    // Set timeline for proper synchronization
    rec_->set_time_duration_secs("frame", time);

    // Extract transform components
    Eigen::Vector3d t = Twc.translation();
    Eigen::Matrix3d R = Twc.rotationMatrix();

    // Create rotation matrix as 3 column vectors for Rerun
    rerun::datatypes::Vec3D columns[3] = {
        rerun::datatypes::Vec3D(R(0,0), R(1,0), R(2,0)), // Column 0
        rerun::datatypes::Vec3D(R(0,1), R(1,1), R(2,1)), // Column 1
        rerun::datatypes::Vec3D(R(0,2), R(1,2), R(2,2))  // Column 2
    };

    // Log camera transform
    rec_->log(
        "world/keyframes/" + std::to_string(kfid),
        rerun::Transform3D::from_translation_mat3x3(
            rerun::components::Translation3D(t.x(), t.y(), t.z()),
            columns
        )
    );

    // Log image if available
    if(!image.empty()) {
        // Convert to RGB if grayscale
        cv::Mat rgb;
        if(image.channels() == 1) {
            cv::cvtColor(image, rgb, cv::COLOR_GRAY2RGB);
        } else {
            rgb = image;
        }

        // Use Rerun's native image API with zero-copy borrow
        // Cast to uint32_t to avoid narrowing conversion warning
        rec_->log(
            "camera/keyframe_" + std::to_string(kfid),
            rerun::Image(
                rerun::borrow(rgb.data, rgb.total() * rgb.channels()),
                {static_cast<uint32_t>(rgb.cols), static_cast<uint32_t>(rgb.rows)},
                rerun::datatypes::ColorModel::RGB
            )
        );
    }
#endif
}

void RerunVisualizer::logGTTrajectory() {
#ifdef ENABLE_RERUN
    if(!enabled_ || !rec_ || !gt_loader_) return;

    const auto& gt_positions = gt_loader_->getTrajectory();
    if(gt_positions.empty()) {
        std::cerr << "[Rerun] No GT positions available" << std::endl;
        return;
    }

    // Convert GT positions to Rerun format
    std::vector<rerun::Position3D> positions;
    positions.reserve(gt_positions.size());
    for(const auto& pos : gt_positions) {
        positions.push_back(rerun::Position3D(pos.x(), pos.y(), pos.z()));
    }

    // Log GT trajectory as green line strip
    std::vector<rerun::LineStrip3D> gt_strip;
    gt_strip.push_back(rerun::LineStrip3D(positions));
    rec_->log("world/gt_trajectory", rerun::LineStrips3D(std::move(gt_strip))
        .with_colors({rerun::Color(0, 255, 0)})  // Green
        .with_radii({0.05f}));

    std::cout << "[Rerun] Logged GT trajectory with " << positions.size() << " points" << std::endl;
#endif
}
