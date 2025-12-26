# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

OV²SLAM is a real-time Visual SLAM system for stereo/monocular cameras with a multi-threaded architecture (Tracking, Mapping, Bundle Adjustment, Loop Closing). This version operates without ROS dependencies.

## Build and Run Commands

### Standard Build (without Rerun)
```bash
./build.sh
```

### Build with Rerun Visualization
```bash
ENABLE_RERUN=ON ./build.sh
```

### Run OV2SLAM
```bash
./build/ov2slam parameters_files/pohang00.yaml ~/datasets/pohang00
```

## Architecture Overview

**Multi-threaded Pipeline:**
- **Main Thread**: Image loading from disk, queue management
- **SLAM Manager Thread** (`SlamManager::run()`): Core orchestration
- **Mapper Thread**: Map building, local map tracking
- **Estimator Thread**: Bundle adjustment, optimization
- **LoopCloser Thread**: Loop closure detection (optional with iBoW-LCD)

**Data Flow:**
```
Images → VisualFrontEnd → Mapper/Estimator → Pose Output
         (KLT tracking)  (BA/optimization)
```

## Key Components

### Core Classes
- **SlamManager** (`include/ov2slam.hpp`, `src/ov2slam.cpp`): Central orchestrator, initializes all subsystems
- **VisualFrontEnd**: Feature extraction (FAST/Shi-Tomasi), KLT tracking, keyframe selection
- **Mapper**: Map point triangulation, local map management
- **Estimator**: Bundle adjustment with Ceres, anchored inverse depth parametrization
- **Frame**: Camera frames with keypoints/descriptors
- **SlamParams** (`include/slam_params.hpp`): YAML-based configuration system

### Threading Model
- Producer-consumer between main thread and SLAM manager
- Mutex-protected queues for frame passing
- Each major component runs in its own thread

## Build System

### CMake Configuration
- Uses `CMakeLists.txt`
- **Build Options:**
  - `-DENABLE_PROFILING=ON`: Performance instrumentation (enabled by default)
  - `-DENABLE_RERUN=ON`: Optional 3D visualization via Rerun
  - `-DWITH_IBOW_LCD=OFF`: Loop closure support (auto-detected)

### Build Script (`build.sh`)
- Creates clean build directory
- Configures with profiling by default
- Builds with `-O3 -march=native`

### Dependencies
- OpenCV (computer vision)
- Eigen3 (linear algebra)
- Sophus (SE3 Lie groups)
- Ceres Solver (non-linear optimization)
- OpenGV (optional, geometric vision)

## Dataset Format

**Directory Structure:**
```
~/datasets/pohang00/
├── stereo/
│   ├── left_images/      # Left camera images
│   ├── right_images/     # Right camera images
│   └── timestamp.txt     # Timestamps: "timestamp image_name"
└── calibration/          # Camera calibration files
```

**YAML Configuration:**
- Camera intrinsics/extrinsics in `parameters_files/*.yaml`
- SLAM parameters (feature thresholds, BA settings, etc.)
- Rerun parameters (e.g., `rerun_map_log_frequency: 10`)

## Optional Features

### Rerun Visualization
- **Enabled**: `ENABLE_RERUN=ON` during build
- **Installation**: `snap install rerun` (or download binary)
- **Logs**: Camera trajectory, map points, keyframes, images
- **Note**: Requires `rerun` binary in PATH (snap installs to `/snap/bin/rerun`)
- **API**: `rec_->spawn()` launches viewer process automatically
- **Implementation**: `src/rerun_visualizer.cpp`, guarded by `#ifdef ENABLE_RERUN`

### Profiling
- Always enabled in builds
- `ProfiledMutex` wrapper for timing locks
- Instrumentation in core SLAM components
- **Thread Safety**: When profiling is enabled, mutexes become `ProfiledMutex` instead of `std::mutex`
  - Use `std::lock_guard<ProfiledMutex>` (NOT `std::lock_guard<std::mutex>`)
  - Include `#include "sync_profiler.hpp"`

## Known Issues and Solutions

### Matrix Orthogonality Errors
If you see `Sophus ensure failed... R is not orthogonal`, the calibration matrices in YAML need orthogonalization. The code includes SVD-based orthogonalization in `slam_params.cpp` for the computed relative transform.

### OpenCV Essential Matrix Crash
```
OpenCV(4.6.0) error: (-215:Assertion failed) E.cols == 3 && E.rows == 3
```
This is a bug in OV2SLAM's multi-view geometry, not related to visualization. Occurs during epipolar filtering in difficult sequences.

## Memory Management

- Extensive use of `std::shared_ptr` for components
- `EIGEN_MAKE_ALIGNED_OPERATOR_NEW` macro required for Eigen-using classes
- RAII patterns throughout

## Code Organization

### Entry Points
- `src/main.cpp`: Main entry point (disk images)

### Key Directories
- `include/`: Headers
- `src/`: Implementations
- `parameters_files/`: YAML configurations
- `Thirdparty/`: Dependencies (Sophus, iBoW-LCD, backward-cpp)

### Integration Points
- **Rerun**: `src/rerun_visualizer.cpp` - optional 3D visualization
- **Profiling**: `include/sync_profiler.hpp` - performance instrumentation

## Output Files

Generated in working directory:
- `ov2slam_trajectory.txt`: Visual odometry trajectory
- `ov2slam_keyframes.txt`: Keyframe poses
- `ov2slam_full_trajectory.txt`: Loop-closure optimized trajectory

## Additional instructions

Remeber to use subagents and tools, use Write when you want to add somehting to file(don't use echo), use Read when you want to read from file(don't use sed).
When running long running commands you can use background Command/Task.
