cmake_minimum_required(VERSION 2.8.3)
project(ov2slam)

if(POLICY CMP0022)
  cmake_policy(SET CMP0022 NEW)
endif()

# Set C++ standard
if( ${CMAKE_VERSION} VERSION_LESS "3.8.2" )
    set(CMAKE_CXX_STANDARD 14)
else()
    set(CMAKE_CXX_STANDARD 17)
endif()

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -march=native -g")

# OpenCV contrib flag (set to OFF if not available)
set(WITH_OPENCV_CONTRIB OFF)

if(WITH_OPENCV_CONTRIB)
    message("\nGoing to use OpenCV contrib! (WITH_OPENCV_CONTRIB is set to : ${WITH_OPENCV_CONTRIB}.\n")
    add_definitions(-DOPENCV_CONTRIB)
endif()

# OV2SLAM - No ROS dependencies

# Profiling support
option(ENABLE_PROFILING "Enable synchronization profiling" OFF)
if(ENABLE_PROFILING)
    add_definitions(-DENABLE_PROFILING)
    message(STATUS "Profiling enabled")
endif()

# Rerun visualization support
option(ENABLE_RERUN "Enable Rerun visualization" OFF)
if(ENABLE_RERUN)
  message(STATUS "Rerun visualization enabled - fetching SDK via FetchContent...")

  include(FetchContent)

  # Download and configure Rerun C++ SDK
  FetchContent_Declare(
    rerun_sdk
    URL https://github.com/rerun-io/rerun/releases/latest/download/rerun_cpp_sdk.zip
    DOWNLOAD_EXTRACT_TIMESTAMP true
  )

  # This will download prebuilt libraries, C++ sources, headers, and CMake config
  # We need to override the project() call in Rerun's CMakeLists to avoid recursive CXX enable
  set(CMAKE_PROJECT_rerun_sdk_INCLUDE)
  FetchContent_MakeAvailable(rerun_sdk)

  add_definitions(-DENABLE_RERUN)
  message(STATUS "Rerun SDK: ${rerun_sdk_SOURCE_DIR}")
endif()

# OpenCV 2.x to 4.x compatibility
add_definitions(-DCV_GRAY2RGB=cv::COLOR_GRAY2BGR)
add_definitions(-DCV_RGB2GRAY=cv::COLOR_RGB2GRAY)

# iBOW-LCD
option(WITH_IBOW_LCD "Build with iBoW-LCD loop closure support" ON)
if( EXISTS "${PROJECT_SOURCE_DIR}/Thirdparty/ibow_lcd/build/liblcdetector.so" )
  if(WITH_IBOW_LCD)
    add_definitions(-DIBOW_LCD)
    message(STATUS "iBoW-LCD found! Going to use Loop Closer!")
  else()
    message(STATUS "iBoW-LCD found but disabled by user!")
    set(WITH_IBOW_LCD OFF)
  endif()
else()
  if(WITH_IBOW_LCD)
    message(WARNING "iBoW-LCD requested but not found! Disabling...")
  endif()
  set(WITH_IBOW_LCD OFF)
  message(STATUS "iBoW-LCD NOT found! Loop Closer will not be enabled!")
endif()

# Find OpenCV
find_package(OpenCV REQUIRED)
if(OpenCV_VERSION_MAJOR LESS 3)
  message(FATAL_ERROR "OpenCV 3 or 4 is required! Current version: ${OpenCV_VERSION}")
endif()

if(OpenCV_VERSION VERSION_LESS "3.3.0")
  message("ENABLE Internal ParallelLoopBodyLambdaWrapper Class definition (OpenCV < 3.3.0)")
  add_definitions(-DOPENCV_LAMBDA_MISSING)
endif()

# Find Eigen3
find_package(Eigen3 3.3.0 REQUIRED NO_MODULE)
if (Eigen3_FOUND)
  message(STATUS "Found Eigen3 lib: ${Eigen3_VERSION} - ${EIGEN3_INCLUDE_DIRS}")
endif()

# Find OpenGV (optional)
find_package(opengv QUIET)
get_target_property(opengv_INCLUDE_DIR opengv INTERFACE_INCLUDE_DIRECTORIES)
if (opengv_FOUND)
  message(STATUS "Found Opengv lib: ${opengv_INCLUDE_DIR} - ${opengv_DIR}")
  # DISABLED: OpenGV causes double-free crash in P3P RANSAC
  # add_definitions(-DUSE_OPENGV)
  message(STATUS "OpenGV DISABLED - using OpenCV fallback instead")
endif()

# Find Ceres
find_package(Ceres NO_DEFAULT_PATH PATHS ${PROJECT_SOURCE_DIR}/Thirdparty/ceres-solver/install/)
find_package(Ceres REQUIRED)
if (Ceres_FOUND)
  message(STATUS "Found Ceres lib: ${CERES_VERSION} - ${Ceres_DIR}")
endif()

# Find Sophus
find_package(Sophus NO_DEFAULT_PATH PATHS ${PROJECT_SOURCE_DIR}/Thirdparty/Sophus/build/)
find_package(Sophus REQUIRED)
if (Sophus_FOUND)
  message(STATUS "Found Sophus lib: ${Sophus_VERSION} - ${Sophus_DIR}")
endif()

# Find GeographicLib (try both GeographicLib and geographiclib)
find_package(GeographicLib QUIET)
if(NOT GeographicLib_FOUND)
  find_package(geographiclib QUIET)
  if(geographiclib_FOUND)
    set(GeographicLib_FOUND TRUE)
    # Create alias for compatibility
    if(NOT TARGET GeographicLib::GeographicLib AND TARGET geographiclib::geographiclib)
      add_library(GeographicLib::GeographicLib ALIAS geographiclib::geographiclib)
    endif()
  endif()
endif()

# Manual search for Debian/Ubuntu systems
if(NOT GeographicLib_FOUND)
  find_path(GeographicLib_INCLUDE_DIRS
    NAMES GeographicLib/Config.h
    PATHS /usr/include /usr/local/include
  )
  find_library(GeographicLib_LIBRARIES
    NAMES GeographicLib
    PATHS /usr/lib /usr/lib/x86_64-linux-gnu /usr/local/lib
  )
  if(GeographicLib_INCLUDE_DIRS AND GeographicLib_LIBRARIES)
    set(GeographicLib_FOUND TRUE)
    # Create imported target
    if(NOT TARGET GeographicLib::GeographicLib)
      add_library(GeographicLib::GeographicLib UNKNOWN IMPORTED)
      set_target_properties(GeographicLib::GeographicLib PROPERTIES
        IMPORTED_LOCATION "${GeographicLib_LIBRARIES}"
        INTERFACE_INCLUDE_DIRECTORIES "${GeographicLib_INCLUDE_DIRS}"
      )
    endif()
  endif()
endif()

if(GeographicLib_FOUND)
  message(STATUS "Found GeographicLib: ${GeographicLib_VERSION}")
  # Note: GPS initialization now uses runtime checks (via GTLoader), not compile-time flags
  # GeographicLib is only used for GPSConverter class (optional advanced features)
else()
  message(STATUS "GeographicLib not found - GPS init will use GTLoader's built-in WGS84->ENU conversion")
endif()

# AHRS initialization (always available if GTLoader is built)
add_definitions(-DENABLE_AHRS_INIT)
message(STATUS "AHRS initialization enabled")

# Add backward
add_subdirectory(Thirdparty/backward-cpp)

# Build OV2SLAM library
add_library(${PROJECT_NAME}_lib
  src/ov2slam.cpp
  src/visual_front_end.cpp
  src/frame.cpp
  src/slam_params.cpp
  src/camera_calibration.cpp
  src/feature_extractor.cpp
  src/feature_tracker.cpp
  src/map_manager.cpp
  src/map_point.cpp
  src/mapper.cpp
  src/multi_view_geometry.cpp
  src/ceres_parametrization.cpp
  src/optimizer.cpp
  src/estimator.cpp
  src/loop_closer.cpp
  src/gt_loader.cpp
  src/gps_converter.cpp
  src/trajectory_evaluator.cpp
  src/imu_preintegration.cpp
)

target_include_directories(${PROJECT_NAME}_lib
  PUBLIC
    ${PROJECT_SOURCE_DIR}/include
    ${PROJECT_SOURCE_DIR}/include/ceres_parametrization
    ${Eigen3_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}_lib
    PUBLIC
      Eigen3::Eigen
      Sophus::Sophus
    PRIVATE
      ${OpenCV_LIBS}
      Ceres::ceres
)

# Add optional dependencies
if (WITH_IBOW_LCD)
  target_include_directories(
    ${PROJECT_NAME}_lib
    PUBLIC
      ${PROJECT_SOURCE_DIR}/Thirdparty/obindex2/lib/include/
      ${PROJECT_SOURCE_DIR}/Thirdparty/ibow_lcd/include/
  )
  target_link_libraries(
    ${PROJECT_NAME}_lib
    PRIVATE
      ${PROJECT_SOURCE_DIR}/Thirdparty/ibow_lcd/build/liblcdetector.so
  )
endif()

if (opengv_FOUND)
  target_link_libraries(
    ${PROJECT_NAME}_lib
    PRIVATE
      opengv
  )
endif()

if(GeographicLib_FOUND)
  target_link_libraries(${PROJECT_NAME}_lib PUBLIC GeographicLib::GeographicLib)
endif()

# Rerun visualization
if(ENABLE_RERUN)
  target_sources(${PROJECT_NAME}_lib PRIVATE
    src/rerun_visualizer.cpp
  )
  target_link_libraries(${PROJECT_NAME}_lib PRIVATE rerun_sdk)
endif()

# Build executable
add_executable(${PROJECT_NAME} src/main.cpp ${BACKWARD_ENABLE})

# Add Rerun definitions to executable too
if(ENABLE_RERUN)
  target_compile_definitions(${PROJECT_NAME} PRIVATE ENABLE_RERUN)
  target_link_libraries(${PROJECT_NAME} PRIVATE rerun_sdk)
endif()

add_backward(${PROJECT_NAME})

target_include_directories(${PROJECT_NAME}
  PRIVATE
    ${PROJECT_SOURCE_DIR}/include
    ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(
    ${PROJECT_NAME}
    PRIVATE
      ${PROJECT_NAME}_lib
)

message(STATUS "Building OV2SLAM")
message(STATUS "OpenCV: ${OpenCV_VERSION}")
message(STATUS "Eigen3: ${Eigen3_VERSION}")
message(STATUS "Sophus: ${Sophus_VERSION}")
message(STATUS "Ceres: ${CERES_VERSION}")
message(STATUS "iBoW-LCD: ${WITH_IBOW_LCD}")
message(STATUS "OpenGV: ${opengv_FOUND}")

# Testing support
option(ENABLE_TESTS "Build unit tests" OFF)
if(ENABLE_TESTS)
    enable_testing()

    # Download GoogleTest
    include(FetchContent)
    FetchContent_Declare(
        googletest
        GIT_REPOSITORY https://github.com/google/googletest.git
        GIT_TAG v1.14.0
    )
    set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(googletest)

    # Test executable
    add_executable(gps_tests
        test/test_gps_converter.cpp
        test/test_direction.cpp
    )

    target_include_directories(gps_tests
        PRIVATE
            ${PROJECT_SOURCE_DIR}/include
            ${Eigen3_INCLUDE_DIRS}
    )

    target_link_libraries(gps_tests
        PRIVATE
            ${PROJECT_NAME}_lib
            gtest
            gtest_main
    )

    include(GoogleTest)
    gtest_discover_tests(gps_tests)

    message(STATUS "Tests enabled")
endif()

# GT extractor (for extracting ground truth trajectory)
if(ENABLE_GPS_TESTS)
    add_executable(gt_extractor
        test/gt_extractor.cpp
    )
    target_link_libraries(gt_extractor ${PROJECT_NAME}_lib ${GeographicLib_LIBRARIES})
endif()
