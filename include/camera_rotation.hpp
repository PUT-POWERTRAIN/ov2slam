#pragma once

#include <sophus/so3.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <iostream>
#include <string>

class CameraRotation {
public:
    CameraRotation(const std::string& yaml_file);
    Sophus::SO3d get_rotation();
    bool done;
private:
    Sophus::SO3d rotation_;
};

inline CameraRotation::CameraRotation(const std::string& yaml_file) {
    cv::FileStorage fs(yaml_file, cv::FileStorage::READ);
    
    if (!fs.isOpened()) {
        std::cerr << "ERROR: Cannot open YAML file: " << yaml_file << std::endl;
        rotation_ = Sophus::SO3d();
        return;
    }
    
    cv::Mat rotation_mat;
    fs["Camera.rotation_matrix"] >> rotation_mat;
    
    if (rotation_mat.empty()) {
        std::cerr << "ERROR: Camera.rotation_matrix not found in YAML!" << std::endl;
        rotation_ = Sophus::SO3d();
        fs.release();
        return;
    }
    
    Eigen::Matrix3d eigen_rot;
    cv::cv2eigen(rotation_mat, eigen_rot);
    rotation_ = Sophus::SO3d(eigen_rot);
    fs.release();
}

inline Sophus::SO3d CameraRotation::get_rotation() {
    return rotation_;
}