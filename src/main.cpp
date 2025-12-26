/**
* OV2SLAM - Visual SLAM System
* Reads stereo images from disk and runs SLAM
*
* BUILD INSTRUCTIONS:
* 1. Build with: ./build.sh
* 2. Run: ./build/ov2slam parameters_files/pohang00.yaml ~/datasets/pohang00
*/

#include <iostream>
#include <string>
#include <thread>
#include <fstream>
#include <chrono>
#include <opencv2/opencv.hpp>

#include "ov2slam.hpp"
#include "slam_params.hpp"
#include "rerun_visualizer.hpp"
#include "gt_loader.hpp"

// Use stub ros_visualizer instead of real ROS
#define USE_STUB_VISUALIZER

// Read timestamps from file
std::vector<std::pair<double, std::string>> readTimestamps(const std::string& filepath) {
    std::vector<std::pair<double, std::string>> timestamps;
    std::ifstream file(filepath);

    if (!file.is_open()) {
        std::cerr << "Failed to open timestamp file: " << filepath << std::endl;
        return timestamps;
    }

    std::string line;
    while (std::getline(file, line)) {
        double ts;
        std::string img_name;
        size_t tab_pos = line.find('\t');

        if (tab_pos != std::string::npos) {
            ts = std::stod(line.substr(0, tab_pos));
            img_name = line.substr(tab_pos + 1);
            timestamps.push_back({ts, img_name});
        }
    }

    std::cout << "Read " << timestamps.size() << " timestamps from " << filepath << std::endl;
    return timestamps;
}


int main(int argc, char** argv) {
    if (argc < 3) {
        std::cout << "\n===== OV2SLAM =====\n";
        std::cout << "Usage: " << argv[0] << " <parameters_file.yaml> <dataset_path>\n";
        std::cout << "\nExample:\n";
        std::cout << "  " << argv[0] << " parameters_files/pohang00.yaml ~/datasets/pohang00\n";
        std::cout << "\nOutput files:\n";
        std::cout << "  - ov2slam_trajectory.txt (VO trajectory)\n";
        std::cout << "  - ov2slam_keyframes.txt (keyframe poses)\n";
        std::cout << "  - ov2slam_full_trajectory.txt (optimized trajectory)\n";
        return 1;
    }

    std::string parameters_file = argv[1];
    std::string dataset_path = argv[2];

    std::cout << "\n========================================\n";
    std::cout << "     OV2SLAM\n";
    std::cout << "========================================\n";
    std::cout << "Parameters: " << parameters_file << "\n";
    std::cout << "Dataset:    " << dataset_path << "\n\n";

    // Load parameters
    std::cout << "Loading parameters..." << std::endl;
    cv::FileStorage fsSettings(parameters_file.c_str(), cv::FileStorage::READ);

    if (!fsSettings.isOpened()) {
        std::cerr << "Failed to open parameters file: " << parameters_file << std::endl;
        return 1;
    }

    std::shared_ptr<SlamParams> pparams;
    pparams.reset(new SlamParams(fsSettings));

    // Create visualizer (using stub, no ROS required)
    ros::NodeHandle nh;  // Stub node handle
    std::shared_ptr<RosVisualizer> prosviz;
    prosviz.reset(new RosVisualizer(nh));

#ifdef ENABLE_RERUN
    std::shared_ptr<RerunVisualizer> prviz;
    prviz.reset(new RerunVisualizer(pparams->rerun_output_file_));
#endif

    // Load Ground Truth if available
    std::shared_ptr<GTLoader> gt_loader;
    std::string gps_file = dataset_path + "/navigation/gps.txt";
    std::string ahrs_file = dataset_path + "/navigation/ahrs.txt";

    gt_loader.reset(new GTLoader());
    if(gt_loader->loadFromGPS(gps_file)) {
        gt_loader->loadFromAHRS(ahrs_file);  // Merge orientation data
        std::cout << "[GT] Ground truth loaded successfully" << std::endl;

#ifdef ENABLE_RERUN
        if(prviz) {
            prviz->setGTLoader(gt_loader);
            prviz->logGTTrajectory();  // Log GT trajectory to Rerun
        }
#endif
    } else {
        std::cout << "[GT] No ground truth available, skipping GT visualization" << std::endl;
    }

    // Setup SLAM Manager
    std::cout << "Initializing SLAM system..." << std::endl;
    SlamManager slam(pparams, prosviz);

#if defined(ENABLE_GPS_INIT) || defined(ENABLE_AHRS_INIT)
    if(gt_loader && (pparams->use_gps_init_ || pparams->use_ahrs_init_)) {
        slam.setGTLoader(gt_loader);
        if(pparams->use_gps_init_)
            std::cout << "GPS+AHRS initialization enabled" << std::endl;
        else
            std::cout << "AHRS-only initialization enabled" << std::endl;
    }
#endif

#ifdef ENABLE_RERUN
    if(prviz) {
        slam.setRerunVisualizer(prviz);
        slam.setMapLogFrequency(pparams->rerun_map_log_frequency_);
        std::cout << "Rerun visualization enabled (map freq: " << pparams->rerun_map_log_frequency_ << ")" << std::endl;
    }
#else
    std::cout << "Rerun visualization disabled (compile with -DENABLE_RERUN=ON)" << std::endl;
#endif

    // Start SLAM thread
    std::thread slamthread(&SlamManager::run, &slam);

    // Read timestamps
    std::string timestamp_file = dataset_path + "/stereo/timestamp.txt";
    auto timestamps = readTimestamps(timestamp_file);

    if (timestamps.empty()) {
        std::cerr << "No timestamps found!" << std::endl;
        return 1;
    }

    // Image directories
    std::string left_dir = dataset_path + "/stereo/left_images/";
    std::string right_dir = dataset_path + "/stereo/right_images/";

    // Process images
    std::cout << "\nProcessing " << timestamps.size() << " image pairs..." << std::endl;
    std::cout << "Press Ctrl+C to stop early\n" << std::endl;

    auto start_time = std::chrono::steady_clock::now();

    for (size_t i = 0; i < timestamps.size(); i++) {
        double ts = timestamps[i].first;
        std::string img_name = timestamps[i].second;

        // Load images
        std::string left_path = left_dir + img_name + ".png";
        std::string right_path = right_dir + img_name + ".png";

        cv::Mat left_img = cv::imread(left_path, cv::IMREAD_GRAYSCALE);
        cv::Mat right_img = cv::imread(right_path, cv::IMREAD_GRAYSCALE);

        if (left_img.empty() || right_img.empty()) {
            std::cerr << "Failed to read image pair: " << img_name << std::endl;
            continue;
        }

        // Add to SLAM
        slam.addNewStereoImages(ts, left_img, right_img);

        if ((i + 1) % 100 == 0) {
            auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
                std::chrono::steady_clock::now() - start_time).count();
            std::cout << "Processed " << (i + 1) << "/" << timestamps.size()
                     << " images (elapsed: " << elapsed << "s)" << std::endl;
        }
    }

    auto total_time = std::chrono::duration_cast<std::chrono::seconds>(
        std::chrono::steady_clock::now() - start_time).count();

    std::cout << "\n========================================\n";
    std::cout << "Finished processing all images!\n";
    std::cout << "Total time: " << total_time << "s\n";
    std::cout << "Average: " << (timestamps.size() / std::max(1.0, (double)total_time)) << " fps\n";
    std::cout << "========================================\n\n";

    std::cout << "Shutting down SLAM..." << std::endl;

    // Request SLAM to exit
    slam.bexit_required_ = true;

    // Wait for SLAM thread to finish
    int wait_count = 0;
    while (slam.bis_on_ && wait_count < 10) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
        wait_count++;
    }

    if (slamthread.joinable()) {
        slamthread.join();
    }

    std::cout << "\n===== OV2SLAM Finished =====\n";
    std::cout << "Results saved to:\n";
    std::cout << "  - ov2slam_trajectory.txt\n";
    std::cout << "  - ov2slam_keyframes.txt\n";
    std::cout << "  - ov2slam_full_trajectory.txt\n";

    return 0;
}
