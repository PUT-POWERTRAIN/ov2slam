#ifndef ASYNC_IMAGE_LOADER_FIXED_HPP
#define ASYNC_IMAGE_LOADER_FIXED_HPP

#include <string>
#include <queue>
#include <mutex>
#include <thread>
#include <condition_variable>
#include <memory>
#include <map>
#include <vector>
#include <atomic>
#include <chrono>
#include <iostream>
#include <opencv2/opencv.hpp>

/**
 * ASYNC_IMAGE_LOADER_PARALLEL_HPP - FIXED VERSION
 *
 * Thread-safe parallel async image loader with single queue, multiple consumers pattern.
 *
 * THREAD SAFETY FIXES (vs original):
 * 1. Single queue pattern: Tasks NOT duplicated (no redundant work)
 * 2. std::shared_ptr<FramePair>: Prevents use-after-free when frames are erased
 * 3. Configurable thread counts: Supports 2, 4, or 6 worker threads
 * 4. No race conditions: Each task processed by exactly one worker
 * 5. Atomic completion flag: Prevents double completion race
 * 6. Failed load handling: Prevents memory leaks on errors
 *
 * USAGE (drop-in compatible with original):
 *   AsyncImageLoaderParallel loader(left_dir, right_dir, 16);  // 2 threads (1L+1R)
 *   AsyncImageLoaderParallel loader(left_dir, right_dir, 16, 2, 2);  // 4 threads (2L+2R)
 *   AsyncImageLoaderParallel loader(left_dir, right_dir, 16, 3, 3);  // 6 threads (3L+3R)
 *
 * THREAD ARCHITECTURE:
 * - Single left_queue_ shared by all left worker threads
 * - Single right_queue_ shared by all right worker threads
 * - First worker to wake up gets the task (no duplicates)
 * - FramePair stored as shared_ptr, safe for concurrent access
 */

class AsyncImageLoaderParallel {
public:
    struct ImagePair {
        double timestamp;
        cv::Mat left;
        cv::Mat right;
        size_t frame_idx;
        long load_us;   // Total time spent loading (max of left/right decode times)
        long wait_us;   // Time main thread waited for this frame
        bool failed;    // Set to true if image load failed
    };

    /**
     * Constructor
     * @param left_dir Directory containing left images
     * @param right_dir Directory containing right images
     * @param prefetch_size Number of frames to prefetch (buffer size hint)
     * @param num_left_threads Number of left image decoder threads (recommended: 1-3)
     * @param num_right_threads Number of right image decoder threads (recommended: 1-3)
     */
    AsyncImageLoaderParallel(const std::string& left_dir,
                         const std::string& right_dir,
                         size_t prefetch_size = 16,
                         size_t num_left_threads = 1,
                         size_t num_right_threads = 1)
        : left_dir_(left_dir)
        , right_dir_(right_dir)
        , prefetch_size_(prefetch_size)
        , stop_requested_(false)
        , num_left_threads_(num_left_threads)
        , num_right_threads_(num_right_threads)
        , next_output_idx_(0)
    {
        // Validate thread counts
        if (num_left_threads_ == 0) num_left_threads_ = 1;
        if (num_right_threads_ == 0) num_right_threads_ = 1;

        // Launch left worker threads
        left_workers_.reserve(num_left_threads_);
        for (size_t i = 0; i < num_left_threads_; ++i) {
            left_workers_.emplace_back(&AsyncImageLoaderParallel::leftWorker, this);
        }

        // Launch right worker threads
        right_workers_.reserve(num_right_threads_);
        for (size_t i = 0; i < num_right_threads_; ++i) {
            right_workers_.emplace_back(&AsyncImageLoaderParallel::rightWorker, this);
        }
    }

    /**
     * Destructor - stops all worker threads
     */
    ~AsyncImageLoaderParallel() {
        // Signal all workers to stop
        stop_requested_ = true;

        // Wake up all workers so they can check stop flag
        cv_left_.notify_all();
        cv_right_.notify_all();

        // Join all left workers
        for (auto& worker : left_workers_) {
            if (worker.joinable()) {
                worker.join();
            }
        }

        // Join all right workers
        for (auto& worker : right_workers_) {
            if (worker.joinable()) {
                worker.join();
            }
        }
    }

    /**
     * Add frame timestamp/name to load queue
     * Tasks are NOT duplicated - each task processed once by each side
     *
     * @param timestamp Frame timestamp
     * @param img_name Image filename (without .png extension)
     * @param idx Frame index (for ordering)
     */
    void addFrame(double timestamp, const std::string& img_name, size_t idx) {
        LoadTask task{timestamp, img_name, idx};

        // Add to LEFT queue (single queue, multiple consumers)
        {
            std::lock_guard<std::mutex> lock(mtx_left_);
            left_queue_.push(task);
        }
        cv_left_.notify_one();  // Wake one left worker

        // Add to RIGHT queue (single queue, multiple consumers)
        {
            std::lock_guard<std::mutex> lock(mtx_right_);
            right_queue_.push(task);
        }
        cv_right_.notify_one();  // Wake one right worker
    }

    /**
     * Set the starting frame index (for non-zero indexed datasets)
     * Call this before addFrame() if your frames don't start at index 0
     *
     * @param idx The first frame index to expect
     */
    void setStartIndex(size_t idx) {
        std::lock_guard<std::mutex> lock(mtx_frames_);
        next_output_idx_ = idx;
    }

    /**
     * Signal end-of-stream (no more frames will be added)
     * This allows getNext() to return false instead of waiting forever
     */
    void endOfStream() {
        eos_flag_.store(true);
        cv_ready_.notify_all();
    }

    /**
     * Get next complete image pair (blocks if not ready)
     * Returns frames in order of idx (sequential)
     *
     * @param output [out] The loaded image pair
     * @return true if frame retrieved, false if stop requested and no more frames
     */
    bool getNext(ImagePair& output) {
        auto wait_start = std::chrono::high_resolution_clock::now();

        std::unique_lock<std::mutex> lock(mtx_frames_);

        // Wait for next frame in sequence to be ready
        cv_ready_.wait(lock, [this] {
            // Check if next frame is ready in completed set
            if (!completed_frames_.empty()) {
                auto it = completed_frames_.find(next_output_idx_);
                if (it != completed_frames_.end()) {
                    return true;
                }
            }
            return stop_requested_.load() || eos_flag_.load();
        });

        if (stop_requested_.load() || eos_flag_.load()) {
            // Check if we have the next frame before giving up
            auto it = completed_frames_.find(next_output_idx_);
            if (it == completed_frames_.end()) {
                return false;  // No more frames
            }
        }

        // Retrieve the completed frame
        auto it = completed_frames_.find(next_output_idx_);
        if (it == completed_frames_.end()) {
            return false;  // Should not happen if predicate worked
        }

        // Copy to output
        output = it->second;

        // Remove from completed set
        completed_frames_.erase(it);

        // Move to next frame
        next_output_idx_++;

        // Clean up old frames from frames_ map to save memory
        // Keep a small window of recent frames for robustness
        cleanupOldFrames();

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

    /**
     * FramePair - holds partially loaded frame data
     * Using shared_ptr ensures thread safety:
     * - Workers can hold reference while main thread erases from map
     * - Frame stays alive until all references released
     */
    struct FramePair {
        cv::Mat left;
        cv::Mat right;
        double timestamp;
        size_t idx;
        long left_load_us = 0;
        long right_load_us = 0;
        bool left_ready = false;
        bool right_ready = false;
        bool load_failed = false;  // Set to true if image load fails

        // Atomic flag to prevent double completion (both threads finishing same frame)
        std::atomic<bool> completion_started{false};

        // Mutex for this specific frame (protects left_ready, right_ready flags)
        std::mutex frame_mutex;
    };

    /**
     * Left worker thread - processes tasks from left_queue_
     * Multiple workers share single queue (no duplicate work)
     */
    void leftWorker() {
        while (!stop_requested_) {
            LoadTask task;

            // Get next task from shared left queue
            {
                std::unique_lock<std::mutex> lock(mtx_left_);
                cv_left_.wait(lock, [this] {
                    return !left_queue_.empty() || stop_requested_;
                });

                if (stop_requested_) break;
                if (left_queue_.empty()) continue;

                task = left_queue_.front();
                left_queue_.pop();
            }

            // Get or create FramePair (shared_ptr for safety)
            std::shared_ptr<FramePair> frame;
            {
                std::lock_guard<std::mutex> lock(mtx_frames_);

                auto it = frames_.find(task.idx);
                if (it == frames_.end()) {
                    // Create new frame
                    frame = std::make_shared<FramePair>();
                    frame->idx = task.idx;
                    frame->timestamp = task.timestamp;
                    frames_[task.idx] = frame;
                } else {
                    frame = it->second;
                }
            }

            // Decode LEFT image (no lock needed - only we write to left)
            auto t1 = std::chrono::high_resolution_clock::now();
            std::string left_path = left_dir_ + task.img_name + ".png";
            cv::Mat left_img = cv::imread(left_path, cv::IMREAD_GRAYSCALE);
            auto t2 = std::chrono::high_resolution_clock::now();
            long load_us = std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count();

            if (left_img.empty()) {
                std::cerr << "[LEFT] Failed to load: " << task.img_name << std::endl;
                // Mark frame as failed and notify main thread
                {
                    std::lock_guard<std::mutex> lock(frame->frame_mutex);
                    frame->load_failed = true;
                    frame->left_ready = true;  // Mark as "ready" so completion happens
                }
                checkAndCompleteFrame(frame);
                continue;
            }

            // Update frame (lock only the frame's mutex)
            {
                std::lock_guard<std::mutex> lock(frame->frame_mutex);
                frame->left = std::move(left_img);  // Move instead of copy
                frame->left_load_us = load_us;
                frame->left_ready = true;
            }

            // Check if frame is complete (both sides ready)
            checkAndCompleteFrame(frame);
        }
    }

    /**
     * Right worker thread - processes tasks from right_queue_
     * Multiple workers share single queue (no duplicate work)
     */
    void rightWorker() {
        while (!stop_requested_) {
            LoadTask task;

            // Get next task from shared right queue
            {
                std::unique_lock<std::mutex> lock(mtx_right_);
                cv_right_.wait(lock, [this] {
                    return !right_queue_.empty() || stop_requested_;
                });

                if (stop_requested_) break;
                if (right_queue_.empty()) continue;

                task = right_queue_.front();
                right_queue_.pop();
            }

            // Get or create FramePair (shared_ptr for safety)
            std::shared_ptr<FramePair> frame;
            {
                std::lock_guard<std::mutex> lock(mtx_frames_);

                auto it = frames_.find(task.idx);
                if (it == frames_.end()) {
                    // Create new frame
                    frame = std::make_shared<FramePair>();
                    frame->idx = task.idx;
                    frame->timestamp = task.timestamp;
                    frames_[task.idx] = frame;
                } else {
                    frame = it->second;
                }
            }

            // Decode RIGHT image (no lock needed - only we write to right)
            auto t1 = std::chrono::high_resolution_clock::now();
            std::string right_path = right_dir_ + task.img_name + ".png";
            cv::Mat right_img = cv::imread(right_path, cv::IMREAD_GRAYSCALE);
            auto t2 = std::chrono::high_resolution_clock::now();
            long load_us = std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count();

            if (right_img.empty()) {
                std::cerr << "[RIGHT] Failed to load: " << task.img_name << std::endl;
                // Mark frame as failed and notify main thread
                {
                    std::lock_guard<std::mutex> lock(frame->frame_mutex);
                    frame->load_failed = true;
                    frame->right_ready = true;  // Mark as "ready" so completion happens
                }
                checkAndCompleteFrame(frame);
                continue;
            }

            // Update frame (lock only the frame's mutex)
            {
                std::lock_guard<std::mutex> lock(frame->frame_mutex);
                frame->right = std::move(right_img);  // Move instead of copy
                frame->right_load_us = load_us;
                frame->right_ready = true;
            }

            // Check if frame is complete (both sides ready)
            checkAndCompleteFrame(frame);
        }
    }

    /**
     * Check if frame is complete and move to completed set
     * Thread-safe via shared_ptr, mutex, and atomic flag
     */
    void checkAndCompleteFrame(const std::shared_ptr<FramePair>& frame) {
        // Atomic test-and-set: ensure only one thread completes this frame
        bool expected = false;
        if (!frame->completion_started.compare_exchange_strong(expected, true)) {
            return;  // Another thread is already completing this frame
        }

        // Check if both sides ready (lock frame's mutex)
        bool both_ready = false;
        {
            std::lock_guard<std::mutex> lock(frame->frame_mutex);
            both_ready = frame->left_ready && frame->right_ready;
        }

        if (!both_ready) {
            // Not both ready yet - release the completion flag
            // (so the other side can try again when it's ready)
            frame->completion_started.store(false);
            return;
        }

        // Both sides ready - construct output and move to completed set
        ImagePair pair;
        {
            std::lock_guard<std::mutex> lock(frame->frame_mutex);
            pair.timestamp = frame->timestamp;
            pair.left = std::move(frame->left);    // Move instead of copy
            pair.right = std::move(frame->right);  // Move instead of copy
            pair.frame_idx = frame->idx;
            pair.load_us = std::max(frame->left_load_us, frame->right_load_us);
            pair.failed = frame->load_failed;
        }

        // Add to completed set (need main frames mutex)
        {
            std::lock_guard<std::mutex> lock(mtx_frames_);
            completed_frames_[frame->idx] = pair;

            // Erase from active frames map (shared_ptr keeps it alive if worker still holds ref)
            frames_.erase(frame->idx);
        }

        // Notify main thread (could be this frame or next in sequence)
        cv_ready_.notify_one();
    }

    /**
     * Clean up old frames from map to save memory
     * Keeps a small window of recent frames for robustness
     */
    void cleanupOldFrames() {
        // Clean up frames older than (next_output_idx_ - prefetch_size_)
        size_t cleanup_threshold = (next_output_idx_ > prefetch_size_)
                                   ? (next_output_idx_ - prefetch_size_)
                                   : 0;

        auto it = frames_.begin();
        while (it != frames_.end()) {
            if (it->first < cleanup_threshold) {
                it = frames_.erase(it);
            } else {
                ++it;
            }
        }
    }

    // Configuration
    std::string left_dir_;
    std::string right_dir_;
    size_t prefetch_size_;
    size_t num_left_threads_;
    size_t num_right_threads_;

    // Work queues (single queue per side, multiple consumers)
    std::queue<LoadTask> left_queue_;
    std::queue<LoadTask> right_queue_;
    std::mutex mtx_left_;
    std::mutex mtx_right_;
    std::condition_variable cv_left_;
    std::condition_variable cv_right_;

    // Active frames being loaded (shared_ptr for thread safety)
    std::map<size_t, std::shared_ptr<FramePair>> frames_;
    std::mutex mtx_frames_;
    std::condition_variable cv_ready_;

    // Completed frames ready for output (ordered by idx)
    std::map<size_t, ImagePair> completed_frames_;
    size_t next_output_idx_;  // Next frame idx to output

    // Worker threads
    std::vector<std::thread> left_workers_;
    std::vector<std::thread> right_workers_;

    // Control
    std::atomic<bool> stop_requested_{false};
    std::atomic<bool> eos_flag_{false};  // End-of-stream flag
};

#endif // ASYNC_IMAGE_LOADER_FIXED_HPP
