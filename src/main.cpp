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
#include <iomanip>
#include <climits>
#include <opencv2/opencv.hpp>

#include "ov2slam.hpp"
#include "slam_params.hpp"
#include "rerun_visualizer.hpp"
#include "gt_loader.hpp"
#include "../async_image_loader_parallel.hpp"

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
        std::cout << "Usage: " << argv[0] << " <parameters_file.yaml> <dataset_path> [start_frame] [end_frame]\n";
        std::cout << "\nExample:\n";
        std::cout << "  " << argv[0] << " parameters_files/pohang00.yaml ~/datasets/pohang00\n";
        std::cout << "  " << argv[0] << " parameters_files/pohang00.yaml ~/datasets/pohang00 240 280\n";
        std::cout << "\nOutput files:\n";
        std::cout << "  - ov2slam_trajectory.txt (VO trajectory)\n";
        std::cout << "  - ov2slam_keyframes.txt (keyframe poses)\n";
        std::cout << "  - ov2slam_full_trajectory.txt (optimized trajectory)\n";
        return 1;
    }

    std::string parameters_file = argv[1];
    std::string dataset_path = argv[2];

    // Parse optional frame range arguments
    int start_frame = 0;
    int end_frame = INT_MAX;
    if (argc >= 4) {
        start_frame = std::stoi(argv[3]);
        if (argc >= 5) {
            end_frame = std::stoi(argv[4]);
        }
        std::cout << "Frame range: " << start_frame << " to " << end_frame << std::endl;
    }

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

    // Process images with async I/O (prefetch for speedup)
    std::cout << "\nProcessing " << timestamps.size() << " image pairs..." << std::endl;
    std::cout << "Using PARALLEL async I/O with 16-frame prefetch (12-thread PNG decode: 6L + 6R)" << std::endl;
    std::cout << "Press Ctrl+C to stop early\n" << std::endl;

    auto start_time = std::chrono::steady_clock::now();

    // Apply frame range
    size_t start_idx = (start_frame > 0) ? start_frame : 0;
    size_t end_idx = (end_frame < (int)timestamps.size()) ? end_frame : timestamps.size();

    // Create async loader with 16-frame prefetch (12-thread parallel decode: 6L + 6R)
    AsyncImageLoaderParallel loader(left_dir, right_dir, 16, 6, 6);

    // CRITICAL: Set start index to match frame range (prevents deadlock)
    if (start_idx > 0) {
        loader.setStartIndex(start_idx);
    }

    // Prime the pump: add first 16 frames to load queue
    size_t prefetch_count = std::min((size_t)16, end_idx - start_idx);
    for (size_t i = 0; i < prefetch_count; i++) {
        double ts = timestamps[start_idx + i].first;
        std::string img_name = timestamps[start_idx + i].second;
        loader.addFrame(ts, img_name, start_idx + i);
    }

    // Process frames with async loading
    size_t next_to_queue = start_idx + prefetch_count;

    // Timing accumulators
    double total_io_us = 0.0;
    double total_slam_us = 0.0;
    double total_wait_us = 0.0;

    for (size_t i = start_idx; i < end_idx; i++) {
        AsyncImageLoaderParallel::ImagePair img_pair;

        // Get next loaded frame (blocks if not ready, but should be ready due to prefetch)
        if (!loader.getNext(img_pair)) {
            break;  // End of queue or error
        }

        // Log I/O timing
        total_io_us += img_pair.load_us;
        total_wait_us += img_pair.wait_us;

        // Queue up MULTIPLE frames in background to maintain prefetch buffer
        // (not just 1 frame - that causes wait time)
        while (next_to_queue < end_idx && (next_to_queue - i) < 8) {
            double ts = timestamps[next_to_queue].first;
            std::string img_name = timestamps[next_to_queue].second;
            loader.addFrame(ts, img_name, next_to_queue);
            next_to_queue++;
        }

        // Add to SLAM with timing
        auto slam_start = std::chrono::high_resolution_clock::now();
        slam.addNewStereoImages(img_pair.timestamp, img_pair.left, img_pair.right);
        auto slam_end = std::chrono::high_resolution_clock::now();

        double slam_us = std::chrono::duration_cast<std::chrono::microseconds>(
            slam_end - slam_start).count();
        total_slam_us += slam_us;

        // Print timing breakdown every 10 frames
        if ((i + 1) % 10 == 0 || i == end_idx - 1) {
            double avg_io = total_io_us / (i - start_idx + 1);
            double avg_slam = total_slam_us / (i - start_idx + 1);
            double avg_wait = total_wait_us / (i - start_idx + 1);
            double avg_total = avg_io + avg_slam + avg_wait;

            std::cout << "Frame " << (i + 1) << " timings: "
                      << "I/O=" << std::fixed << std::setprecision(2) << (avg_io/1000.0) << "ms, "
                      << "SLAM=" << (avg_slam/1000.0) << "ms, "
                      << "Wait=" << (avg_wait/1000.0) << "ms, "
                      << "Total=" << (avg_total/1000.0) << "ms "
                      << "(" << std::fixed << std::setprecision(1) << (1000.0/avg_total) << " fps)" << std::endl;
        }

        if ((i + 1) % 100 == 0) {
            auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
                std::chrono::steady_clock::now() - start_time).count();
            std::cout << "Processed " << (i + 1) << "/" << timestamps.size()
                     << " images (elapsed: " << elapsed << "s)" << std::endl;
        }
        // Progress update for frame range
        if (start_frame > 0 || end_frame < INT_MAX) {
            if ((i + 1) % 10 == 0 || i == end_idx - 1) {
                auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
                    std::chrono::steady_clock::now() - start_time).count();
                // std::cout << "Processed frame " << (i + 1) << " (range: " << start_idx
                //          << "-" << end_idx << ", elapsed: " << elapsed << "s)" << std::endl;
            }
        }
    }

    // Print final timing summary
    std::cout << "\n=== Timing Summary ===" << std::endl;
    std::cout << "Total frames: " << (end_idx - start_idx) << std::endl;
    std::cout << "Avg I/O (PNG decode):    " << (total_io_us / (end_idx - start_idx) / 1000.0) << " ms/frame" << std::endl;
    std::cout << "Avg SLAM processing:     " << (total_slam_us / (end_idx - start_idx) / 1000.0) << " ms/frame" << std::endl;
    std::cout << "Avg wait (prefetch):     " << (total_wait_us / (end_idx - start_idx) / 1000.0) << " ms/frame" << std::endl;
    std::cout << "Total per frame:         " << ((total_io_us + total_slam_us + total_wait_us) / (end_idx - start_idx) / 1000.0) << " ms/frame" << std::endl;
    std::cout << "Bottleneck: ";
    if (total_io_us > total_slam_us) {
        std::cout << "I/O (" << (100.0 * total_io_us / (total_io_us + total_slam_us)) << "%)" << std::endl;
    } else {
        std::cout << "SLAM (" << (100.0 * total_slam_us / (total_io_us + total_slam_us)) << "%)" << std::endl;
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
