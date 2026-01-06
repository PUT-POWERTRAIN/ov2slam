/**
* Visualizer for OV2SLAM - Saves trajectories to files
* This replaces ROS visualization with file-based output
*/

#ifndef ROS_VISUALIZER_HPP
#define ROS_VISUALIZER_HPP

#pragma once

#include <sophus/se3.hpp>
#include <opencv2/opencv.hpp>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <vector>
#include <memory>
#include <mutex>

// /////////////////////////////////////////////////////////////////////////////
// ROS STUBS - Minimal replacements for ROS types
// /////////////////////////////////////////////////////////////////////////////

namespace ros {

struct NodeHandle {
    NodeHandle() {}
};

struct Publisher {
    int getNumSubscribers() const { return 0; }
    template<typename T> void publish(const T&) {}
};

struct Time {
    double time_;
    Time(double t) : time_(t) {}
    double toSec() const { return time_; }
    static Time now() { return Time(0.0); }
};

inline void requestShutdown() {
    // Stub - does nothing in OV2SLAM
}

} // namespace ros

namespace sensor_msgs {
struct Image {
    std::string header;
};
struct Imu {};
} // namespace sensor_msgs

namespace geometry_msgs {
struct Point { double x, y, z; };
struct Quaternion { double x, y, z, w; };
struct PoseStamped {
    std::string header;
    Point pose;
    Quaternion orientation;
};
} // namespace geometry_msgs

namespace visualization_msgs {
struct Marker {
    std::string header;
    int type;
    struct Color { double a, r, g, b; } color;
    struct Scale { double x; } scale;
    std::vector<geometry_msgs::Point> points;
};
struct MarkerArray {
    std::vector<Marker> markers;
};
} // namespace visualization_msgs

namespace nav_msgs {
struct Path {};
struct Odometry {};
} // namespace nav_msgs

namespace std_msgs {
struct Header {
    std::string frame_id;
    double stamp;
};
struct ColorRGBA {
    float r, g, b, a;
    ColorRGBA() : r(0), g(0), b(0), a(1) {}
};
struct Float32 {};
struct Bool {};
} // namespace std_msgs

namespace pcl {
template<typename T>
struct PointCloud {
    std::string header;
    std::vector<T> points;
    // Ptr alias
    using Ptr = std::shared_ptr<PointCloud<T>>;
    using ConstPtr = std::shared_ptr<const PointCloud<T>>;
};

// Stub for PointXYZRGB
struct PointXYZRGB {
    float x, y, z;
    uint8_t r, g, b;
    PointXYZRGB() : x(0), y(0), z(0), r(255), g(255), b(255) {}
    PointXYZRGB(float _x, float _y, float _z) : x(_x), y(_y), z(_z), r(255), g(255), b(255) {}
    PointXYZRGB(float _x, float _y, float _z, uint8_t _r, uint8_t _g, uint8_t _b)
        : x(_x), y(_y), z(_z), r(_r), g(_g), b(_b) {}
};

// Specialization for PointXYZRGB
template<>
struct PointCloud<PointXYZRGB> {
    std::string header;
    std::vector<PointXYZRGB> points;
    using Ptr = std::shared_ptr<PointCloud<PointXYZRGB>>;
    using ConstPtr = std::shared_ptr<const PointCloud<PointXYZRGB>>;
};

} // namespace pcl

// CameraPoseVisualization stub
struct CameraPoseVisualization {
    CameraPoseVisualization() {}
    CameraPoseVisualization(double r, double g, double b, double a) {}
    void reset() {}
    void add_pose(const Eigen::Vector3d& t, const Eigen::Quaterniond& q) {}
    void setImageBoundaryColor(double r, double g, double b) {}
    void setOpticalCenterConnectorColor(double r, double g, double b) {}
    void setScale(double s) {}
    void setLineWidth(double w) {}
    void publish_by(ros::Publisher& pub, const std::string& header) {}
};

// /////////////////////////////////////////////////////////////////////////////
// STANDALONE VISUALIZER - Mimics RosVisualizer interface without ROS
// /////////////////////////////////////////////////////////////////////////////

class RosVisualizer {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    RosVisualizer(ros::NodeHandle& n) {
        std::cout << "\n[OV2SLAM] Visualizer created\n";

        // Open output files
        traj_file_.open("ov2slam_trajectory.txt");
        if (traj_file_.is_open()) {
            traj_file_ << "# timestamp tx ty tz qx qy qz qw\n";
        }

        kfs_traj_file_.open("ov2slam_keyframes.txt");
        if (kfs_traj_file_.is_open()) {
            kfs_traj_file_ << "# keyframe tx ty tz qx qy qz qw\n";
        }

        full_traj_file_.open("ov2slam_full_trajectory.txt");
        if (full_traj_file_.is_open()) {
            full_traj_file_ << "# timestamp tx ty tz qx qy qz qw\n";
        }

        std::cout << "[OV2SLAM] Results will be saved to:\n";
        std::cout << "  - ov2slam_trajectory.txt (VO trajectory)\n";
        std::cout << "  - ov2slam_keyframes.txt (keyframe poses)\n";
        std::cout << "  - ov2slam_full_trajectory.txt (optimized trajectory)\n";
    }

    ~RosVisualizer() {
        if (traj_file_.is_open()) traj_file_.close();
        if (kfs_traj_file_.is_open()) kfs_traj_file_.close();
        if (full_traj_file_.is_open()) full_traj_file_.close();
        std::cout << "\n[OV2SLAM] Visualizer closed, trajectories saved\n";
    }

    void pubTrackImage(const cv::Mat& imgTrack, const double time) {
        // Optionally save debug images if needed
    }

    void pubVO(const Sophus::SE3d& Twc, const double time) {
        std::lock_guard<std::mutex> lock(traj_mutex_);
        if (traj_file_.is_open()) {
            const Eigen::Vector3d t = Twc.translation();
            const Eigen::Quaterniond q = Twc.unit_quaternion();
            traj_file_ << std::fixed << std::setprecision(9)
                       << time << " "
                       << t.x() << " " << t.y() << " " << t.z() << " "
                       << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << "\n";
            traj_file_.flush();
        }
    }

    void addVisualKF(const Sophus::SE3d& Twc) {
        keyframes_.push_back(Twc);
    }

    void pubVisualKFs(const double time) {
        // Keyframes are tracked but not published here
    }

    void pubPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcloud, const double time) {
        // Optionally save point cloud
    }

    void addKFsTraj(const Sophus::SE3d& Twc) {
        std::lock_guard<std::mutex> lock(kfs_traj_mutex_);
        if (kfs_traj_file_.is_open()) {
            const Eigen::Vector3d t = Twc.translation();
            const Eigen::Quaterniond q = Twc.unit_quaternion();
            kfs_traj_file_ << std::fixed << std::setprecision(9)
                          << t.x() << " " << t.y() << " " << t.z() << " "
                          << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << "\n";
        }
    }

    void clearKFsTraj() {
        keyframes_.clear();
    }

    void pubKFsTraj(const double time) {
        std::lock_guard<std::mutex> lock(kfs_traj_mutex_);
        kfs_traj_file_.flush();
    }

    void pubFinalKFsTraj(const Sophus::SE3d& Twc, const double time) {
        std::lock_guard<std::mutex> lock(full_traj_mutex_);
        if (full_traj_file_.is_open()) {
            const Eigen::Vector3d t = Twc.translation();
            const Eigen::Quaterniond q = Twc.unit_quaternion();
            full_traj_file_ << std::fixed << std::setprecision(9)
                           << time << " "
                           << t.x() << " " << t.y() << " " << t.z() << " "
                           << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << "\n";
            full_traj_file_.flush();
        }
    }

    // Public members required by SLAM code
    ros::Publisher pub_image_track_;
    ros::Publisher pub_vo_traj_;
    ros::Publisher pub_vo_pose_;
    ros::Publisher camera_pose_visual_pub_;
    CameraPoseVisualization cameraposevisual_;
    ros::Publisher pub_point_cloud_;
    ros::Publisher pub_kfs_pose_;
    std::vector<CameraPoseVisualization> vkeyframesposevisual_;
    ros::Publisher pub_kfs_traj_;
    ros::Publisher pub_final_kfs_traj_;
    visualization_msgs::Marker vo_traj_msg_;
    visualization_msgs::Marker kfs_traj_msg_;
    visualization_msgs::Marker final_kfs_traj_msg_;

private:
    std::vector<Sophus::SE3d> keyframes_;
    std::ofstream traj_file_;
    std::ofstream kfs_traj_file_;
    std::ofstream full_traj_file_;
    std::mutex traj_mutex_;       // Protects traj_file_ writes
    std::mutex kfs_traj_mutex_;   // Protects kfs_traj_file_ writes
    std::mutex full_traj_mutex_;  // Protects full_traj_file_ writes
};
#endif // ROS_VISUALIZER_HPP
