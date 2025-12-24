/**
* Standalone OV2SLAM - Minimal version that doesn't require ROS
* This reads stereo images from disk and runs SLAM core directly
*/

#include <iostream>
#include <string>
#include <thread>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>

// Forward declarations to avoid including ROS headers
namespace Sophus {
class SE3d;
}

// Minimal stub for ROS-dependent stuff
struct DummyMsg {
    int getNumSubscribers() const { return 0; }
};

// Simple visualizer that writes to text files
class SimpleVisualizer {
public:
    SimpleVisualizer() {
        traj_file_.open("ov2slam_trajectory.txt");
        if (traj_file_.is_open()) {
            traj_file_ << "# timestamp tx ty tz qx qy qz qw\n";
        }
        std::cout << "SimpleVisualizer created - results will be saved to ov2slam_trajectory.txt\n";
    }

    ~SimpleVisualizer() {
        if (traj_file_.is_open()) traj_file_.close();
    }

    void savePose(double time, const Eigen::Vector3d& t, const Eigen::Quaterniond& q) {
        if (traj_file_.is_open()) {
            traj_file_ << std::fixed << std::setprecision(9)
                       << time << " "
                       << t.x() << " " << t.y() << " " << t.z() << " "
                       << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << "\n";
            traj_file_.flush();
        }
    }

    DummyMsg pub_image_track_;
    DummyMsg pub_kfs_pose_;
    DummyMsg pub_kfs_traj_;
    DummyMsg pub_final_kfs_traj_;
    DummyMsg pub_point_cloud_;

private:
    std::ofstream traj_file_;
};

// Since SlamManager requires RosVisualizer, we need to work around this.
// The easiest approach is to build OV2SLAM library first (with ROS disabled),
// then link it with our standalone executable.

// Actually, let's try a different approach - modify the build to be ROS-free

int main(int argc, char** argv) {
    std::cout << "\n===== OV2SLAM Standalone =====\n";
    std::cout << "This version requires building OV2SLAM without ROS dependency.\n";
    std::cout << "Please use the build_standalone.sh script to compile.\n";

    if (argc < 3) {
        std::cout << "\nUsage: " << argv[0] << " <parameters_file.yaml> <dataset_path>\n";
        std::cout << "Example: " << argv[0] << " parameters_files/pohang00.yaml ~/datasets/pohang00\n";
        return 1;
    }

    // For now, this is a stub - we need to modify the build system
    std::cerr << "\nError: OV2SLAM core is currently tied to ROS.\n";
    std::cerr << "To run standalone, you need to:\n";
    std::cerr << "1. Remove ROS dependencies from CMakeLists.txt\n";
    std::cerr << "2. Create a stub ros_visualizer.hpp\n";
    std::cerr << "3. Rebuild the library\n";

    return 1;
}
