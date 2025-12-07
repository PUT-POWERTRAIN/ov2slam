#include "rclcpp/rclcpp.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "builtin_interfaces/msg/time.hpp"
#include <sensor_msgs/msg/imu.hpp>
#include <vector>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cmath>

class TfStaticPublisher : public rclcpp::Node
{
public:
  TfStaticPublisher();
private:
  void tf_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);
  void calculate_orientation_from_imu();
  Eigen::Quaterniond calculateRotationQuaternion(
    const Eigen::Vector3d& imu_vec_raw,
    const Eigen::Vector3d& vo_pose_raw);

  Eigen::Vector3d imu_acceleration_;
  Eigen::Vector3d vo_acceleration_;
  Eigen::Vector3d vo_velocity_;
  Eigen::Vector3d vo_pose_;
  Eigen::Vector3d prev_vo_pose_;
  Eigen::Vector3d prev_vo_velocity_;
  rclcpp::Time last_time_;

  bool static_tf_sent_;
  bool has_initial_data_;
  Eigen::Quaterniond q_orientation_;
  rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr publisher_tf_static_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscriber_pose_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscriber_imu_;
};

TfStaticPublisher::TfStaticPublisher() : Node("tf_static_publisher")
{
  static_tf_sent_ = false;
  has_initial_data_ = false;
  vo_pose_ = Eigen::Vector3d::Zero();
  prev_vo_pose_ = Eigen::Vector3d::Zero();
  vo_velocity_ = Eigen::Vector3d::Zero();
  prev_vo_velocity_ = Eigen::Vector3d::Zero();
  vo_acceleration_ = Eigen::Vector3d::Zero();
  imu_acceleration_ = Eigen::Vector3d::Zero();
  q_orientation_ = Eigen::Quaterniond::Identity();
  
  // Ustawienia QoS dla /tf_static (TRANSIENT_LOCAL)
  auto qos_static = rclcpp::QoS(rclcpp::KeepLast(10));
  qos_static.transient_local();
  qos_static.reliable();
    
  publisher_tf_static_ = this->create_publisher<tf2_msgs::msg::TFMessage>("/tf_static", qos_static);
    
  // Subskrybuj /tf z domyślnym QoS
  auto qos_tf = rclcpp::QoS(rclcpp::KeepLast(100));
  qos_tf.best_effort();
    
  subscriber_pose_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/vo_pose", 
    qos_tf,
    std::bind(&TfStaticPublisher::tf_callback, this, std::placeholders::_1));
  
  subscriber_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(
    "/imu_data", 
    qos_tf,
    std::bind(&TfStaticPublisher::imu_callback, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "TF Static Publisher started - listening to /vo_pose and /imu_data");
}

void TfStaticPublisher::tf_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{  
  // Jeśli już wysłano static_tf, nie rób nic więcej
  if (static_tf_sent_) {
    return;
  }

  // Nowa pozycja z VO
  Eigen::Vector3d new_pose = Eigen::Vector3d(msg->pose.position.x,
                                              msg->pose.position.y,
                                              msg->pose.position.z);
  
  // Pierwsza iteracja - inicjalizacja
  if (!has_initial_data_) {
    vo_pose_ = new_pose;
    last_time_ = msg->header.stamp;
    has_initial_data_ = true;
    RCLCPP_INFO(this->get_logger(), "Initial pose received");
    return;
  }
  
  // Obliczanie delta czasu
  rclcpp::Time current_time = msg->header.stamp;
  double dt = (current_time - last_time_).seconds();
  
  if (dt <= 0.0 || dt > 1.0) {
    RCLCPP_WARN(this->get_logger(), "Invalid dt: %.6f, skipping", dt);
    last_time_ = current_time;
    return;
  }
  
  // Obliczanie prędkości: v = (pose - prev_pose) / dt
  Eigen::Vector3d new_velocity = (new_pose - vo_pose_) / dt;
  
  // Obliczanie przyspieszenia: a = (velocity - prev_velocity) / dt
  Eigen::Vector3d new_acceleration = (new_velocity - vo_velocity_) / dt;
  
  // Sprawdzenie czy mamy wystarczające przyspieszenie do obliczeń
  if (new_acceleration.norm() > 0.1 && imu_acceleration_.norm() > 0.1) {
    vo_acceleration_ = new_acceleration;
    
    RCLCPP_INFO(this->get_logger(), 
                "VO accel: [%.3f, %.3f, %.3f], IMU accel: [%.3f, %.3f, %.3f]",
                vo_acceleration_.x(), vo_acceleration_.y(), vo_acceleration_.z(),
                imu_acceleration_.x(), imu_acceleration_.y(), imu_acceleration_.z());
    
    // Wywołanie funkcji obliczającej orientację
    calculate_orientation_from_imu();
    
    // Jeśli obliczono orientację, publikuj tf_static
    if (!static_tf_sent_) {
      // Przygotuj wiadomość do publikacji
      tf2_msgs::msg::TFMessage tf_msg;
      geometry_msgs::msg::TransformStamped transform;
        
      // Użyj timestamp z /vo_pose
      transform.header.stamp = current_time;
      transform.header.frame_id = "world";
      transform.child_frame_id = "dworld";
        
      // Translacja (brak)
      transform.transform.translation.x = 0.0;
      transform.transform.translation.y = 0.0;
      transform.transform.translation.z = 0.0;
        
      // Rotacja (quaternion)
      transform.transform.rotation.x = q_orientation_.x();
      transform.transform.rotation.y = q_orientation_.y();
      transform.transform.rotation.z = q_orientation_.z();
      transform.transform.rotation.w = q_orientation_.w();
        
      tf_msg.transforms.push_back(transform);
        
      publisher_tf_static_->publish(tf_msg);
      static_tf_sent_ = false;
      
      RCLCPP_INFO(this->get_logger(), 
                  "Published static TF with quaternion: [x=%.6f, y=%.6f, z=%.6f, w=%.6f]",
                  q_orientation_.x(), q_orientation_.y(), 
                  q_orientation_.z(), q_orientation_.w());
    }
  }
  
  // Aktualizacja stanu dla następnej iteracji
  prev_vo_velocity_ = vo_velocity_;
  vo_velocity_ = new_velocity;
  prev_vo_pose_ = vo_pose_;
  vo_pose_ = new_pose;
  last_time_ = current_time;
}

void TfStaticPublisher::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) 
{
  imu_acceleration_ = Eigen::Vector3d(
    msg->linear_acceleration.x,
    msg->linear_acceleration.y,
    msg->linear_acceleration.z);
}

Eigen::Quaterniond TfStaticPublisher::calculateRotationQuaternion(
    const Eigen::Vector3d& imu_vec_raw,
    const Eigen::Vector3d& vo_pose_raw)
{
    // Normalizacja wektorów
    Eigen::Vector3d imu_vec = imu_vec_raw.normalized();
    Eigen::Vector3d vo_pose = vo_pose_raw.normalized();
    
    // Obliczanie kąta między wektorami
    double dot_product = vo_pose.dot(imu_vec);
    dot_product = std::clamp(dot_product, -1.0, 1.0);
    double angle = std::acos(dot_product);
    
    // Obliczanie osi rotacji (iloczyn wektorowy)
    Eigen::Vector3d rotation_axis = vo_pose.cross(imu_vec);
    double axis_norm = rotation_axis.norm();
    
    // Obsługa przypadków współliniowych
    if (axis_norm < 1e-6) {
        if (dot_product > 0) {
            // Wektory równoległe - brak rotacji
            return Eigen::Quaterniond::Identity();
        } else {
            // Wektory antyrównoległe - rotacja o 180 stopni
            // Znajdź oś prostopadłą do vo_pose
            rotation_axis = vo_pose.cross(Eigen::Vector3d(1, 0, 0));
            if (rotation_axis.norm() < 1e-6) {
                rotation_axis = vo_pose.cross(Eigen::Vector3d(0, 1, 0));
            }
            rotation_axis.normalize();
            angle = M_PI;
        }
    } else {
        rotation_axis.normalize();
    }
    
    // Tworzenie kwaterniona z osi i kąta
    Eigen::AngleAxisd angle_axis(angle, rotation_axis);
    Eigen::Quaterniond quaternion(angle_axis);
    
    return quaternion;
}

void TfStaticPublisher::calculate_orientation_from_imu()
{
  RCLCPP_INFO(this->get_logger(), "Calculating orientation from IMU and VO data");
  
  // Sprawdzenie czy mamy dane
  if (imu_acceleration_.norm() < 1e-6 || vo_acceleration_.norm() < 1e-6) {
    RCLCPP_WARN(this->get_logger(), "Not enough acceleration data for orientation calculation");
    return;
  }

  // Obliczenie kwaternionu rotacji
  q_orientation_ = calculateRotationQuaternion(imu_acceleration_, vo_acceleration_);
  
  RCLCPP_INFO(this->get_logger(), 
              "Orientation calculated - Quaternion: x=%.6f, y=%.6f, z=%.6f, w=%.6f",
              q_orientation_.x(), q_orientation_.y(), 
              q_orientation_.z(), q_orientation_.w());
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TfStaticPublisher>());
  rclcpp::shutdown();
  return 0;
}