#ifndef ASYNC_IMAGE_LOADER_4THREAD_HPP
#define ASYNC_IMAGE_LOADER_4THREAD_HPP

#include <string>
#include <queue>
#include <mutex>
#include <thread>
#include <condition_variable>
#include <opencv2/opencv.hpp>

// Parallel async image loader with 4-thread decode
// 2 threads for left images, 2 threads for right images
class AsyncImageLoader4Thread {
public:
    struct ImagePair {
        double timestamp;
        cv::Mat left;
        cv::Mat right;
        size_t frame_idx;
        long load_us;
        long wait_us;
    };

    AsyncImageLoader4Thread(const std::string& left_dir,
                            const std::string& right_dir,
                            size_t prefetch_size = 16)
        : left_dir_(left_dir)
        , right_dir_(right_dir)
        , stop_requested_(false)
    {
        // Start worker threads
        left_thread1_ = std::thread(&AsyncImageLoader4Thread::leftWorker1, this);
        left_thread2_ = std::thread(&AsyncImageLoader4Thread::leftWorker2, this);
        right_thread1_ = std::thread(&AsyncImageLoader4Thread::rightWorker1, this);
        right_thread2_ = std::thread(&AsyncImageLoader4Thread::rightWorker2, this);
    }

    ~AsyncImageLoader4Thread() {
        stop_requested_ = true;
        cv_left1_.notify_all();
        cv_left2_.notify_all();
        cv_right1_.notify_all();
        cv_right2_.notify_all();
        if (left_thread1_.joinable()) left_thread1_.join();
        if (left_thread2_.joinable()) left_thread2_.join();
        if (right_thread1_.joinable()) right_thread1_.join();
        if (right_thread2_.joinable()) right_thread2_.join();
    }

    void addFrame(double timestamp, const std::string& img_name, size_t idx) {
        LoadTask task{timestamp, img_name, idx};

        // Add to ALL left queues (first to grab wins)
        {
            std::lock_guard<std::mutex> lock(mtx_left1_);
            left_queue1_.push(task);
        }
        cv_left1_.notify_one();
        {
            std::lock_guard<std::mutex> lock(mtx_left2_);
            left_queue2_.push(task);
        }
        cv_left2_.notify_one();

        // Add to ALL right queues (first to grab wins)
        {
            std::lock_guard<std::mutex> lock(mtx_right1_);
            right_queue1_.push(task);
        }
        cv_right1_.notify_one();
        {
            std::lock_guard<std::mutex> lock(mtx_right2_);
            right_queue2_.push(task);
        }
        cv_right2_.notify_one();
    }

    bool getNext(ImagePair& output) {
        auto wait_start = std::chrono::high_resolution_clock::now();

        std::unique_lock<std::mutex> lock(mtx_ready_);

        // Wait for frame to be ready
        cv_ready_.wait(lock, [this] {
            return !ready_queue_.empty() || stop_requested_;
        });

        if (stop_requested_ && ready_queue_.empty()) return false;

        output = ready_queue_.front();
        ready_queue_.pop();

        auto wait_end = std::chrono::high_resolution_clock::now();
        output.wait_us = std::chrono::duration_cast<std::chrono::microseconds>(
            wait_end - wait_start).count();

        return true;
    }

private:
    struct LoadTask {
        double timestamp;
        std::string img_name;
        size_t idx;
    };

    struct FramePair {
        cv::Mat left;
        cv::Mat right;
        double timestamp;
        size_t idx;
        long left_load_us = 0;
        long right_load_us = 0;
        bool left_ready = false;
        bool right_ready = false;
    };

    void leftWorker1() {
        while (!stop_requested_) {
            LoadTask task;

            {
                std::unique_lock<std::mutex> lock(mtx_left1_);
                cv_left1_.wait(lock, [this] {
                    return !left_queue1_.empty() || stop_requested_;
                });

                if (stop_requested_) break;
                if (left_queue1_.empty()) continue;

                task = left_queue1_.front();
                left_queue1_.pop();
            }

            // Get or create frame pair
            FramePair* frame = nullptr;
            {
                std::unique_lock<std::mutex> lock(mtx_ready_);
                if (frames_.find(task.idx) == frames_.end()) {
                    frames_[task.idx].idx = task.idx;
                    frames_[task.idx].timestamp = task.timestamp;
                }
                frame = &frames_[task.idx];
            }

            // Decode LEFT
            auto t1 = std::chrono::high_resolution_clock::now();
            std::string left_path = left_dir_ + task.img_name + ".png";
            frame->left = cv::imread(left_path, cv::IMREAD_GRAYSCALE);
            auto t2 = std::chrono::high_resolution_clock::now();
            frame->left_load_us = std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count();

            if (frame->left.empty()) {
                std::cerr << "Left1 failed: " << task.img_name << std::endl;
                continue;
            }

            frame->left_ready = true;

            // Check if complete
            {
                std::unique_lock<std::mutex> lock(mtx_ready_);
                if (frame->right_ready) {
                    // Both ready
                    ImagePair pair;
                    pair.timestamp = frame->timestamp;
                    pair.left = frame->left;
                    pair.right = frame->right;
                    pair.frame_idx = frame->idx;
                    pair.load_us = std::max(frame->left_load_us, frame->right_load_us);

                    ready_queue_.push(pair);
                    cv_ready_.notify_one();

                    frames_.erase(task.idx);
                }
            }
        }
    }

    void leftWorker2() {
        while (!stop_requested_) {
            LoadTask task;

            {
                std::unique_lock<std::mutex> lock(mtx_left2_);
                cv_left2_.wait(lock, [this] {
                    return !left_queue2_.empty() || stop_requested_;
                });

                if (stop_requested_) break;
                if (left_queue2_.empty()) continue;

                task = left_queue2_.front();
                left_queue2_.pop();
            }

            // Get or create frame pair
            FramePair* frame = nullptr;
            {
                std::unique_lock<std::mutex> lock(mtx_ready_);
                if (frames_.find(task.idx) == frames_.end()) {
                    frames_[task.idx].idx = task.idx;
                    frames_[task.idx].timestamp = task.timestamp;
                }
                frame = &frames_[task.idx];
            }

            // Decode LEFT
            auto t1 = std::chrono::high_resolution_clock::now();
            std::string left_path = left_dir_ + task.img_name + ".png";
            frame->left = cv::imread(left_path, cv::IMREAD_GRAYSCALE);
            auto t2 = std::chrono::high_resolution_clock::now();
            frame->left_load_us = std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count();

            if (frame->left.empty()) {
                std::cerr << "Left2 failed: " << task.img_name << std::endl;
                continue;
            }

            frame->left_ready = true;

            // Check if complete
            {
                std::unique_lock<std::mutex> lock(mtx_ready_);
                if (frame->right_ready) {
                    // Both ready
                    ImagePair pair;
                    pair.timestamp = frame->timestamp;
                    pair.left = frame->left;
                    pair.right = frame->right;
                    pair.frame_idx = frame->idx;
                    pair.load_us = std::max(frame->left_load_us, frame->right_load_us);

                    ready_queue_.push(pair);
                    cv_ready_.notify_one();

                    frames_.erase(task.idx);
                }
            }
        }
    }

    void rightWorker1() {
        while (!stop_requested_) {
            LoadTask task;

            {
                std::unique_lock<std::mutex> lock(mtx_right1_);
                cv_right1_.wait(lock, [this] {
                    return !right_queue1_.empty() || stop_requested_;
                });

                if (stop_requested_) break;
                if (right_queue1_.empty()) continue;

                task = right_queue1_.front();
                right_queue1_.pop();
            }

            // Get or create frame pair
            FramePair* frame = nullptr;
            {
                std::unique_lock<std::mutex> lock(mtx_ready_);
                if (frames_.find(task.idx) == frames_.end()) {
                    frames_[task.idx].idx = task.idx;
                    frames_[task.idx].timestamp = task.timestamp;
                }
                frame = &frames_[task.idx];
            }

            // Decode RIGHT
            auto t1 = std::chrono::high_resolution_clock::now();
            std::string right_path = right_dir_ + task.img_name + ".png";
            frame->right = cv::imread(right_path, cv::IMREAD_GRAYSCALE);
            auto t2 = std::chrono::high_resolution_clock::now();
            frame->right_load_us = std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count();

            if (frame->right.empty()) {
                std::cerr << "Right1 failed: " << task.img_name << std::endl;
                continue;
            }

            frame->right_ready = true;

            // Check if complete
            {
                std::unique_lock<std::mutex> lock(mtx_ready_);
                if (frame->left_ready) {
                    // Both ready
                    ImagePair pair;
                    pair.timestamp = frame->timestamp;
                    pair.left = frame->left;
                    pair.right = frame->right;
                    pair.frame_idx = frame->idx;
                    pair.load_us = std::max(frame->left_load_us, frame->right_load_us);

                    ready_queue_.push(pair);
                    cv_ready_.notify_one();

                    frames_.erase(task.idx);
                }
            }
        }
    }

    void rightWorker2() {
        while (!stop_requested_) {
            LoadTask task;

            {
                std::unique_lock<std::mutex> lock(mtx_right2_);
                cv_right2_.wait(lock, [this] {
                    return !right_queue2_.empty() || stop_requested_;
                });

                if (stop_requested_) break;
                if (right_queue2_.empty()) continue;

                task = right_queue2_.front();
                right_queue2_.pop();
            }

            // Get or create frame pair
            FramePair* frame = nullptr;
            {
                std::unique_lock<std::mutex> lock(mtx_ready_);
                if (frames_.find(task.idx) == frames_.end()) {
                    frames_[task.idx].idx = task.idx;
                    frames_[task.idx].timestamp = task.timestamp;
                }
                frame = &frames_[task.idx];
            }

            // Decode RIGHT
            auto t1 = std::chrono::high_resolution_clock::now();
            std::string right_path = right_dir_ + task.img_name + ".png";
            frame->right = cv::imread(right_path, cv::IMREAD_GRAYSCALE);
            auto t2 = std::chrono::high_resolution_clock::now();
            frame->right_load_us = std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count();

            if (frame->right.empty()) {
                std::cerr << "Right2 failed: " << task.img_name << std::endl;
                continue;
            }

            frame->right_ready = true;

            // Check if complete
            {
                std::unique_lock<std::mutex> lock(mtx_ready_);
                if (frame->left_ready) {
                    // Both ready
                    ImagePair pair;
                    pair.timestamp = frame->timestamp;
                    pair.left = frame->left;
                    pair.right = frame->right;
                    pair.frame_idx = frame->idx;
                    pair.load_us = std::max(frame->left_load_us, frame->right_load_us);

                    ready_queue_.push(pair);
                    cv_ready_.notify_one();

                    frames_.erase(task.idx);
                }
            }
        }
    }

    std::string left_dir_, right_dir_;

    // Left threads
    std::thread left_thread1_;
    std::queue<LoadTask> left_queue1_;
    std::mutex mtx_left1_;
    std::condition_variable cv_left1_;

    std::thread left_thread2_;
    std::queue<LoadTask> left_queue2_;
    std::mutex mtx_left2_;
    std::condition_variable cv_left2_;

    // Right threads
    std::thread right_thread1_;
    std::queue<LoadTask> right_queue1_;
    std::mutex mtx_right1_;
    std::condition_variable cv_right1_;

    std::thread right_thread2_;
    std::queue<LoadTask> right_queue2_;
    std::mutex mtx_right2_;
    std::condition_variable cv_right2_;

    // Shared data
    std::map<size_t, FramePair> frames_;
    std::mutex mtx_ready_;
    std::condition_variable cv_ready_;

    // Output queue
    std::queue<ImagePair> ready_queue_;

    bool stop_requested_;
};

#endif // ASYNC_IMAGE_LOADER_4THREAD_HPP
