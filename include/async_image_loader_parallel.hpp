/**
* AsyncImageLoaderParallel - Parallel PNG decompression for OV2SLAM
* Uses thread pool to decompress multiple PNG images simultaneously
*
* Usage:
*   AsyncImageLoaderParallel loader(left_dir, right_dir, timestamps, 4, 8);
*   loader.start();  // 4 = buffer_size, 8 = num_threads
*   // In main loop:
*   double ts;
*   cv::Mat left, right;
*   while(loader.getNext(ts, left, right)) {
*       slam.addNewStereoImages(ts, left, right);
*   }
*/

#pragma once

#include <string>
#include <queue>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <vector>
#include <opencv2/opencv.hpp>

struct ImagePair {
    double timestamp;
    cv::Mat left_img;
    cv::Mat right_img;

    ImagePair() : timestamp(0.0) {}

    ImagePair(double ts, const cv::Mat& left, const cv::Mat& right)
        : timestamp(ts), left_img(left), right_img(right) {}
};

class AsyncImageLoaderParallel {
public:
    AsyncImageLoaderParallel(const std::string& left_dir,
                             const std::string& right_dir,
                             const std::vector<std::pair<double, std::string>>& timestamps,
                             size_t buffer_size = 4,
                             size_t num_threads = 8)
        : left_dir_(left_dir)
        , right_dir_(right_dir)
        , timestamps_(timestamps)
        , buffer_size_(buffer_size)
        , num_threads_(num_threads)
        , next_to_load_(0)
        , next_to_deliver_(0)
        , running_(false)
        , finished_(false)
    {
        loaded_images_.resize(timestamps.size());
    }

    ~AsyncImageLoaderParallel() {
        stop();
    }

    // Start background loading threads
    void start() {
        running_ = true;
        finished_ = false;
        next_to_load_ = 0;
        next_to_deliver_ = 0;

        // Launch worker threads
        for (size_t i = 0; i < num_threads_; i++) {
            workers_.emplace_back(&AsyncImageLoaderParallel::workerLoop, this);
        }

        std::cout << "[AsyncImageLoaderParallel] Started with " << num_threads_
                  << " threads, buffer size " << buffer_size_ << "\n";
    }

    // Stop background threads
    void stop() {
        running_ = false;
        cv_.notify_all();

        for (auto& w : workers_) {
            if (w.joinable()) {
                w.join();
            }
        }
        workers_.clear();
    }

    // Get next pre-loaded image pair (blocks until available, in order)
    bool getNext(double& timestamp, cv::Mat& left_img, cv::Mat& right_img) {
        std::unique_lock<std::mutex> lock(queue_mutex_);

        // Wait for next image in sequence to be ready
        cv_.wait(lock, [this] {
            return (next_to_deliver_ < loaded_images_.size() && loaded_images_[next_to_deliver_].loaded) || finished_;
        });

        if (next_to_deliver_ >= loaded_images_.size() || finished_) {
            return false;  // No more images
        }

        // Get next image in sequence
        ImagePair& pair = loaded_images_[next_to_deliver_].pair;
        timestamp = pair.timestamp;
        left_img = pair.left_img;
        right_img = pair.right_img;

        next_to_deliver_++;

        // Notify workers that they can load more
        cv_.notify_all();

        return true;
    }

    // Non-blocking version
    bool tryGet(double& timestamp, cv::Mat& left_img, cv::Mat& right_img) {
        std::lock_guard<std::mutex> lock(queue_mutex_);

        if (next_to_deliver_ >= loaded_images_.size() || finished_) {
            return false;
        }

        if (!loaded_images_[next_to_deliver_].loaded) {
            return false;  // Next image not ready yet
        }

        ImagePair& pair = loaded_images_[next_to_deliver_].pair;
        timestamp = pair.timestamp;
        left_img = pair.left_img;
        right_img = pair.right_img;

        next_to_deliver_++;
        cv_.notify_all();

        return true;
    }

    // Check if more images are available
    bool hasMore() const {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        return next_to_deliver_ < loaded_images_.size() || !finished_;
    }

    // Get total number of images
    size_t totalImages() const {
        return timestamps_.size();
    }

private:
    struct LoadedImage {
        bool loaded;
        ImagePair pair;
        LoadedImage() : loaded(false) {}
    };

    // Worker thread - loads images in parallel
    void workerLoop() {
        while (running_) {
            size_t idx;

            // Get next index to load
            {
                std::unique_lock<std::mutex> lock(queue_mutex_);

                // Wait for work to do
                cv_.wait(lock, [this] {
                    // Can load if:
                    // 1. There are images left to load
                    // 2. We haven't pre-filled too many ahead (buffer_size_)
                    size_t max_preload = next_to_deliver_ + buffer_size_;
                    bool can_load = (next_to_load_ < loaded_images_.size()) &&
                                   (next_to_load_ < max_preload);

                    return can_load || !running_;
                });

                if (!running_) break;

                if (next_to_load_ >= loaded_images_.size()) {
                    continue;
                }

                idx = next_to_load_++;
            }

            // Load image WITHOUT holding lock (parallel decompression happens here!)
            const auto& ts = timestamps_[idx];
            std::string left_path = left_dir_ + ts.second + ".png";
            std::string right_path = right_dir_ + ts.second + ".png";

            cv::Mat left_img = cv::imread(left_path, cv::IMREAD_GRAYSCALE);
            cv::Mat right_img = cv::imread(right_path, cv::IMREAD_GRAYSCALE);

            if (left_img.empty() || right_img.empty()) {
                std::cerr << "[AsyncImageLoaderParallel] Failed: " << ts.second << "\n";
                // Mark as loaded anyway (with empty images) to avoid deadlock
                std::lock_guard<std::mutex> lock(queue_mutex_);
                loaded_images_[idx].loaded = true;
                cv_.notify_all();
                continue;
            }

            // Store result
            {
                std::lock_guard<std::mutex> lock(queue_mutex_);
                loaded_images_[idx].loaded = true;
                loaded_images_[idx].pair = ImagePair(ts.first, left_img, right_img);

                // Notify main thread if this is the next one it's waiting for
                if (idx == next_to_deliver_) {
                    cv_.notify_one();
                }
            }
        }

        // Check if this was the last worker
        {
            std::lock_guard<std::mutex> lock(queue_mutex_);
            if (next_to_load_ >= loaded_images_.size()) {
                finished_ = true;
                cv_.notify_all();
            }
        }
    }

    std::string left_dir_;
    std::string right_dir_;
    std::vector<std::pair<double, std::string>> timestamps_;
    size_t buffer_size_;
    size_t num_threads_;

    std::atomic<size_t> next_to_load_;      // Next index to load
    std::atomic<size_t> next_to_deliver_;   // Next index to deliver to user
    std::atomic<bool> running_;
    std::atomic<bool> finished_;

    std::vector<LoadedImage> loaded_images_;  // Pre-allocated array

    mutable std::mutex queue_mutex_;
    std::condition_variable cv_;

    std::vector<std::thread> workers_;
};
