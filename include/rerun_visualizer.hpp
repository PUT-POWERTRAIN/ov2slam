#pragma once
#include <sophus/se3.hpp>
#include <opencv2/opencv.hpp>
#include <memory>
#include <vector>
#include <unordered_map>
#include <string>

// Forward declarations to avoid including full headers
class MapPoint;
class Frame;
class GTLoader;

#ifdef ENABLE_RERUN
#include <rerun.hpp>
#endif

class RerunVisualizer {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // output_file: if empty, spawn live viewer; if non-empty, save to file (.rrd)
    RerunVisualizer(const std::string& output_file = "");
    ~RerunVisualizer();

    // Main logging interface
    void logPose(const Sophus::SE3d& Twc, const double time);
    void logMapPoints(const std::unordered_map<int, std::shared_ptr<MapPoint>>& map_points, const double time);
    void logKeyframe(const Sophus::SE3d& Twc, const cv::Mat& image, int kfid, const double time);

#ifdef ENABLE_RERUN
    // Ground truth
    void setGTLoader(std::shared_ptr<GTLoader> gt_loader) { gt_loader_ = gt_loader; }
    void logGTTrajectory();
#endif

    // Configuration
    void setMapLogFrequency(int freq) { map_log_freq_ = freq; }

private:
#ifdef ENABLE_RERUN
    std::unique_ptr<rerun::RecordingStream> rec_;
    std::vector<Eigen::Vector3d> trajectory_;
    std::shared_ptr<GTLoader> gt_loader_;
#endif
    bool enabled_ = false;
    int map_log_freq_ = 10;
    int frame_count_ = 0;
};
