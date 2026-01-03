#ifndef ASYNC_IMAGE_LOADER_HPP
#define ASYNC_IMAGE_LOADER_HPP

#include <string>
#include <queue>
#include <mutex>
#include <thread>
#include <condition_variable>
#include <opencv2/opencv.hpp>

// Async image loader with prefetch for faster I/O
// Loads next N frames in background while processing current frame
class AsyncImageLoader {
public:
    struct ImagePair {
        double timestamp;
        cv::Mat left;
        cv::Mat right;
        size_t frame_idx;
        long load_us;  // Time spent loading (decode)
        long wait_us;  // Time spent waiting for prefetch
    };

    AsyncImageLoader(const std::string& left_dir,
                     const std::string& right_dir,
                     size_t prefetch_size = 4)
        : left_dir_(left_dir)
        , right_dir_(right_dir)
        , prefetch_size_(prefetch_size)
        , stop_requested_(false)
        , next_idx_(0)
    {
        // Start worker thread
        worker_thread_ = std::thread(&AsyncImageLoader::workerLoop, this);
    }

    ~AsyncImageLoader() {
        stop_requested_ = true;
        cv_.notify_all();
        if (worker_thread_.joinable()) {
            worker_thread_.join();
        }
    }

    // Add timestamp/imagename pair to load queue
    void addFrame(double timestamp, const std::string& img_name, size_t idx) {
        std::unique_lock<std::mutex> lock(queue_mutex_);
        load_queue_.push({timestamp, img_name, idx});
        cv_.notify_one();
    }

    // Get next loaded image pair (blocks if not ready)
    bool getNext(ImagePair& output) {
        auto wait_start = std::chrono::high_resolution_clock::now();

        std::unique_lock<std::mutex> lock(ready_mutex_);

        // Wait for image to be ready
        cv_ready_.wait(lock, [this] {
            return !ready_queue_.empty() || stop_requested_;
        });

        if (stop_requested_ && ready_queue_.empty()) {
            return false;
        }

        output = ready_queue_.front();
        ready_queue_.pop();

        auto wait_end = std::chrono::high_resolution_clock::now();
        output.wait_us = std::chrono::duration_cast<std::chrono::microseconds>(
            wait_end - wait_start).count();

        // Notify worker to load more
        cv_.notify_one();

        return true;
    }

private:
    struct LoadTask {
        double timestamp;
        std::string img_name;
        size_t idx;
    };

    void workerLoop() {
        while (!stop_requested_) {
            LoadTask task;

            // Get next task to load
            {
                std::unique_lock<std::mutex> lock(queue_mutex_);
                cv_.wait(lock, [this] {
                    return !load_queue_.empty() || stop_requested_;
                });

                if (stop_requested_) break;

                if (load_queue_.empty()) continue;

                task = load_queue_.front();
                load_queue_.pop();
            }

            // Load images (this is the slow I/O part)
            auto load_start = std::chrono::high_resolution_clock::now();

            ImagePair pair;
            pair.timestamp = task.timestamp;
            pair.frame_idx = task.idx;

            std::string left_path = left_dir_ + task.img_name + ".png";
            std::string right_path = right_dir_ + task.img_name + ".png";

            pair.left = cv::imread(left_path, cv::IMREAD_GRAYSCALE);
            pair.right = cv::imread(right_path, cv::IMREAD_GRAYSCALE);

            auto load_end = std::chrono::high_resolution_clock::now();
            pair.load_us = std::chrono::duration_cast<std::chrono::microseconds>(
                load_end - load_start).count();

            if (pair.left.empty() || pair.right.empty()) {
                std::cerr << "Failed to read: " << task.img_name << std::endl;
                continue;
            }

            // Add to ready queue
            {
                std::unique_lock<std::mutex> lock(ready_mutex_);
                ready_queue_.push(pair);
            }
            cv_ready_.notify_one();
        }
    }

    std::string left_dir_;
    std::string right_dir_;
    size_t prefetch_size_;

    std::thread worker_thread_;
    std::queue<LoadTask> load_queue_;
    std::queue<ImagePair> ready_queue_;

    std::mutex queue_mutex_;
    std::mutex ready_mutex_;
    std::condition_variable cv_;
    std::condition_variable cv_ready_;

    bool stop_requested_;
    size_t next_idx_;
};

#endif // ASYNC_IMAGE_LOADER_HPP
