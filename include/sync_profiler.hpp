/**
* SyncProfiler - Low-overhead synchronization profiler for OV2SLAM
* Measures mutex contention, function timing, and thread states
*
* Usage:
*   1. Build with: cmake -DENABLE_PROFILING=ON ..
*   2. Add PROFILE_FUNCTION() to functions you want to time
*   3. Replace std::mutex with ProfiledMutex for mutex profiling
*   4. Run your program - report printed at exit
*/

#pragma once

#include <chrono>
#include <mutex>
#include <string>
#include <map>
#include <atomic>
#include <iostream>
#include <iomanip>
#include <algorithm>
#include <thread>
#include <vector>
#include <limits>

#ifdef ENABLE_PROFILING

// High-resolution clock typedef
using Clock = std::chrono::high_resolution_clock;
using TimePoint = Clock::time_point;
using Duration = std::chrono::duration<double, std::milli>;

/**
* ProfiledMutex - Drop-in replacement for std::mutex with timing
* Tracks wait time and hold time for each lock acquisition
*/
class ProfiledMutex {
public:
    explicit ProfiledMutex(const std::string& name)
        : name_(name), wait_count_(0), hold_count_(0), total_wait_ms_(0.0), total_hold_ms_(0.0) {}

    void lock() {
        TimePoint wait_start = Clock::now();

        // Actually acquire the lock
        mtx_.lock();

        // Record wait time
        TimePoint wait_end = Clock::now();
        Duration wait_duration = std::chrono::duration_cast<Duration>(wait_end - wait_start);

        // Update statistics (lock stats mutex for thread safety)
        {
            std::lock_guard<std::mutex> lock(stats_mutex_);
            wait_count_++;
            total_wait_ms_ += wait_duration.count();
        }

        // Record when lock was acquired (for hold time tracking)
        lock_acquired_time_ = Clock::now();
    }

    void unlock() {
        // Calculate hold time
        TimePoint now = Clock::now();
        Duration hold_duration = std::chrono::duration_cast<Duration>(now - lock_acquired_time_);

        // Update hold time statistics
        {
            std::lock_guard<std::mutex> lock(stats_mutex_);
            hold_count_++;
            total_hold_ms_ += hold_duration.count();
        }

        // Actually release the lock
        mtx_.unlock();
    }

    bool try_lock() {
        bool acquired = mtx_.try_lock();

        if (acquired) {
            // Don't record "wait" time for successful try_lock - it's non-blocking
            // (would just measure try_lock() overhead, not actual waiting)
            lock_acquired_time_ = Clock::now();
        } else {
            // Track failed attempts (indicates contention)
            std::lock_guard<std::mutex> lock(stats_mutex_);
            wait_count_++;  // Track contention attempts
        }

        return acquired;
    }

    // Get statistics (for report generation)
    std::string getName() const { return name_; }
    size_t getWaitCount() const {
        std::lock_guard<std::mutex> lock(stats_mutex_);
        return wait_count_;
    }
    size_t getHoldCount() const {
        std::lock_guard<std::mutex> lock(stats_mutex_);
        return hold_count_;
    }
    double getTotalWaitMs() const {
        std::lock_guard<std::mutex> lock(stats_mutex_);
        return total_wait_ms_;
    }
    double getTotalHoldMs() const {
        std::lock_guard<std::mutex> lock(stats_mutex_);
        return total_hold_ms_;
    }

private:
    std::mutex mtx_;  // The actual mutex
    std::string name_;

    // Statistics mutex
    mutable std::mutex stats_mutex_;

    // Timing statistics (protected by stats_mutex_)
    size_t wait_count_;
    size_t hold_count_;
    double total_wait_ms_;
    double total_hold_ms_;

    // Per-lock acquisition timing
    // NOTE: Protected by mtx_ - only accessed by thread holding mtx_
    // Safety guaranteed because:
    //   1. lock_acquired_time_ is written AFTER acquiring mtx_ (in lock())
    //   2. lock_acquired_time_ is read BEFORE releasing mtx_ (in unlock())
    //   3. Only one thread can hold mtx_ at a time (mutex property)
    TimePoint lock_acquired_time_;
};

/**
* RAII lock guard for ProfiledMutex
*/
class ProfiledLockGuard {
public:
    explicit ProfiledLockGuard(ProfiledMutex& mutex) : mutex_(mutex) {
        mutex_.lock();
    }

    ~ProfiledLockGuard() {
        mutex_.unlock();
    }

    ProfiledLockGuard(const ProfiledLockGuard&) = delete;
    ProfiledLockGuard& operator=(const ProfiledLockGuard&) = delete;

private:
    ProfiledMutex& mutex_;
};

/**
* RAII unique lock for ProfiledMutex (supports lock/unlock)
*/
class ProfiledUniqueLock {
public:
    explicit ProfiledUniqueLock(ProfiledMutex& mutex) : mutex_(mutex), owns_(false) {
        mutex_.lock();
        owns_ = true;
    }

    ~ProfiledUniqueLock() {
        if (owns_) {
            mutex_.unlock();
        }
    }

    void lock() {
        if (!owns_) {
            mutex_.lock();
            owns_ = true;
        }
    }

    void unlock() {
        if (owns_) {
            mutex_.unlock();
            owns_ = false;
        }
    }

    bool owns_lock() const { return owns_; }  // Check if lock is held

    ProfiledUniqueLock(const ProfiledUniqueLock&) = delete;
    ProfiledUniqueLock& operator=(const ProfiledUniqueLock&) = delete;

private:
    ProfiledMutex& mutex_;
    bool owns_;
};

/**
* Function statistics
*/
struct FunctionStats {
    size_t call_count{0};
    double total_time_ms{0.0};
    double min_time_ms{std::numeric_limits<double>::infinity()};
    double max_time_ms{0.0};

    void update(double elapsed_ms) {
        call_count++;
        total_time_ms += elapsed_ms;
        min_time_ms = std::min(min_time_ms, elapsed_ms);
        max_time_ms = std::max(max_time_ms, elapsed_ms);
    }

    double getAvg() const {
        return call_count > 0 ? total_time_ms / call_count : 0.0;
    }
};

/**
* Mutex statistics
*/
struct MutexStats {
    size_t wait_count{0};
    size_t hold_count{0};
    double total_wait_ms{0.0};
    double total_hold_ms{0.0};

    double getAvgWaitMs() const {
        return wait_count > 0 ? total_wait_ms / wait_count : 0.0;
    }

    double getAvgHoldMs() const {
        return hold_count > 0 ? total_hold_ms / hold_count : 0.0;
    }

    double getContentionRatio() const {
        return total_hold_ms > 0 ? (total_wait_ms / total_hold_ms) * 100.0 : 0.0;
    }
};

/**
* SyncProfiler - Singleton profiler
* Collects and reports synchronization statistics
*/
class SyncProfiler {
public:
    static SyncProfiler& getInstance() {
        static SyncProfiler instance;
        return instance;
    }

    // Function timing
    void recordFunctionCall(const std::string& func_name, double elapsed_ms) {
        std::lock_guard<std::mutex> lock(mutex_);
        function_stats_[func_name].update(elapsed_ms);
    }

    // Mutex registration
    void registerMutex(const std::string& name, ProfiledMutex* mutex) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (mutexes_.count(name)) {
            std::cerr << "[SyncProfiler] Warning: Duplicate mutex registration: " << name << "\n";
        }
        mutexes_[name] = mutex;
    }

    // Mutex unregistration (call before mutex is destroyed)
    void unregisterMutex(const std::string& name) {
        std::lock_guard<std::mutex> lock(mutex_);
        mutexes_.erase(name);
    }

    // Generate report
    void generateReport() {
        std::lock_guard<std::mutex> lock(mutex_);

        std::cout << "\n";
        std::cout << "================================================================================\n";
        std::cout << "                    OV2SLAM Synchronization Profiler Report\n";
        std::cout << "================================================================================\n\n";

        // Mutex statistics
        if (!mutexes_.empty()) {
            std::cout << "--- Mutex Lock Statistics (Sorted by Contention Ratio) ---\n";
            std::cout << std::left << std::setw(20) << "Mutex Name"
                      << std::right << std::setw(10) << "Locks"
                      << std::setw(15) << "Wait (ms)"
                      << std::setw(15) << "Hold (ms)"
                      << std::setw(15) << "Avg Wait"
                      << std::setw(15) << "Avg Hold"
                      << std::setw(15) << "Contention %"
                      << "\n";
            std::cout << std::string(105, '-') << "\n";

            // Sort by contention ratio
            std::vector<std::pair<std::string, double>> contention_ratios;
            for (const auto& kv : mutexes_) {
                double wait_ms = kv.second->getTotalWaitMs();
                double hold_ms = kv.second->getTotalHoldMs();
                double ratio = hold_ms > 0 ? (wait_ms / hold_ms) * 100.0 : 0.0;
                contention_ratios.push_back({kv.first, ratio});
            }

            std::sort(contention_ratios.begin(), contention_ratios.end(),
                [](const auto& a, const auto& b) { return a.second > b.second; });

            for (const auto& kv : contention_ratios) {
                const auto& mutex = mutexes_[kv.first];
                size_t locks = mutex->getWaitCount();
                double wait_ms = mutex->getTotalWaitMs();
                double hold_ms = mutex->getTotalHoldMs();
                double avg_wait = mutex->getWaitCount() > 0 ? wait_ms / mutex->getWaitCount() : 0.0;
                double avg_hold = mutex->getHoldCount() > 0 ? hold_ms / mutex->getHoldCount() : 0.0;

                std::cout << std::left << std::setw(20) << kv.first
                          << std::right << std::setw(10) << locks
                          << std::setw(15) << std::fixed << std::setprecision(1) << wait_ms
                          << std::setw(15) << std::fixed << std::setprecision(1) << hold_ms
                          << std::setw(15) << std::fixed << std::setprecision(3) << avg_wait
                          << std::setw(15) << std::fixed << std::setprecision(3) << avg_hold
                          << std::setw(14) << std::fixed << std::setprecision(1) << kv.second << "%"
                          << "\n";
            }
            std::cout << "\n";
        }

        // Function timing
        if (!function_stats_.empty()) {
            std::cout << "--- Function Execution Times (Sorted by Total Time) ---\n";
            std::cout << std::left << std::setw(40) << "Function Name"
                      << std::right << std::setw(10) << "Calls"
                      << std::setw(15) << "Total (ms)"
                      << std::setw(15) << "Avg (ms)"
                      << std::setw(15) << "Max (ms)"
                      << "\n";
            std::cout << std::string(95, '-') << "\n";

            // Sort by total time
            std::vector<std::pair<std::string, FunctionStats>> sorted_funcs(
                function_stats_.begin(), function_stats_.end());

            std::sort(sorted_funcs.begin(), sorted_funcs.end(),
                [](const auto& a, const auto& b) {
                    return a.second.total_time_ms > b.second.total_time_ms;
                });

            for (const auto& kv : sorted_funcs) {
                const auto& stats = kv.second;
                std::cout << std::left << std::setw(40) << kv.first
                          << std::right << std::setw(10) << stats.call_count
                          << std::setw(15) << std::fixed << std::setprecision(1) << stats.total_time_ms
                          << std::setw(15) << std::fixed << std::setprecision(3) << stats.getAvg()
                          << std::setw(15) << std::fixed << std::setprecision(1) << stats.max_time_ms
                          << "\n";
            }
            std::cout << "\n";
        }

        std::cout << "================================================================================\n\n";
    }

private:
    SyncProfiler() = default;
    ~SyncProfiler() {
        // Generate report on destruction
        generateReport();
    }
    SyncProfiler(const SyncProfiler&) = delete;
    SyncProfiler& operator=(const SyncProfiler&) = delete;

    std::mutex mutex_;
    std::map<std::string, FunctionStats> function_stats_;
    std::map<std::string, ProfiledMutex*> mutexes_;
};

/**
* ScopeTimer - RAII timer for functions
* Automatically records function execution time on destruction
*
* Uses std::string_view to avoid string copy on construction (C++17)
*/
class ScopeTimer {
public:
    explicit ScopeTimer(std::string_view func_name)
        : func_name_(func_name), start_(Clock::now()) {}

    ~ScopeTimer() {
        TimePoint end = Clock::now();
        Duration elapsed = std::chrono::duration_cast<Duration>(end - start_);
        SyncProfiler::getInstance().recordFunctionCall(std::string(func_name_), elapsed.count());
    }

    ScopeTimer(const ScopeTimer&) = delete;
    ScopeTimer& operator=(const ScopeTimer&) = delete;

private:
    std::string_view func_name_;  // View into original string (no copy)
    TimePoint start_;
};

// Convenience macros
#define PROFILE_FUNCTION() ScopeTimer __timer__(__PRETTY_FUNCTION__)
#define PROFILE_SCOPE(name) ScopeTimer __timer__(name)

#else // ENABLE_PROFILING not defined

// Empty stubs - zero overhead when profiling is disabled
#define PROFILE_FUNCTION()
#define PROFILE_SCOPE(name)

// When profiling is disabled, ProfiledMutex is just std::mutex
using ProfiledMutex = std::mutex;
using ProfiledLockGuard = std::lock_guard<std::mutex>;

#endif // ENABLE_PROFILING
