#include "rclcpp/rclcpp.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "builtin_interfaces/msg/time.hpp"
#include <sensor_msgs/msg/imu.hpp>
#include <vector>
#include <ceres/ceres.h>
#include <Eigen/Dense>
#include <cmath>

// Struktura funkcji kosztu dla Ceres
struct OrientationCostFunctor {
  OrientationCostFunctor(const Eigen::Vector3d& imu_vec,
                        const Eigen::Vector3d& prev_imu_vec,
                        const Eigen::Vector3d& vo_pose,
                        const Eigen::Vector3d& prev_vo_pose)
      : imu_vec_(imu_vec.normalized()),
        prev_imu_vec_(prev_imu_vec.normalized()),
        vo_pose_(vo_pose.normalized()),
        prev_vo_pose_(prev_vo_pose.normalized()) {
    d_imu_ = (imu_vec_ - prev_imu_vec_).normalized();
    d_vo_ = (vo_pose_ - prev_vo_pose_).normalized();
  }

  template <typename T>
  bool operator()(const T* const angles, T* residual) const {
    // angles = [roll, pitch, yaw]
    T c_phi = ceres::cos(angles[0]);
    T s_phi = ceres::sin(angles[0]);
    T c_theta = ceres::cos(angles[1]);
    T s_theta = ceres::sin(angles[1]);
    T c_psi = ceres::cos(angles[2]);
    T s_psi = ceres::sin(angles[2]);

    // Macierz rotacji z kątów Eulera (ZYX convention)
    Eigen::Matrix<T, 3, 3> R;
    R(0, 0) = c_theta * c_psi;
    R(0, 1) = c_theta * s_psi;
    R(0, 2) = -s_theta;

    R(1, 0) = s_phi * s_theta * c_psi - c_phi * s_psi;
    R(1, 1) = s_phi * s_theta * s_psi + c_phi * c_psi;
    R(1, 2) = s_phi * c_theta;

    R(2, 0) = c_phi * s_theta * c_psi + s_phi * s_psi;
    R(2, 1) = c_phi * s_theta * s_psi - s_phi * c_psi;
    R(2, 2) = c_phi * c_theta;

    // Predykcje
    Eigen::Matrix<T, 3, 1> vo_pose_T = vo_pose_.cast<T>();
    Eigen::Matrix<T, 3, 1> d_vo_T = d_vo_.cast<T>();
    
    Eigen::Matrix<T, 3, 1> pred_pose = R * vo_pose_T;
    Eigen::Matrix<T, 3, 1> pred_growth = R * d_vo_T;

    // Błąd pozycji
    Eigen::Matrix<T, 3, 1> e_pose = imu_vec_.cast<T>() - pred_pose;
    
    // Błąd przyrostu
    Eigen::Matrix<T, 3, 1> e_growth = d_imu_.cast<T>() - pred_growth;

    // Suma kwadratów błędów
    residual[0] = e_pose.squaredNorm() + e_growth.squaredNorm();

    return true;
  }

private:
  Eigen::Vector3d imu_vec_;
  Eigen::Vector3d prev_imu_vec_;
  Eigen::Vector3d vo_pose_;
  Eigen::Vector3d prev_vo_pose_;
  Eigen::Vector3d d_imu_;
  Eigen::Vector3d d_vo_;
};

class TfStaticPublisher : public rclcpp::Node
{
public:
  TfStaticPublisher();
private:
  void tf_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);
  void calculate_orientation_from_imu();

  std::vector<double> imu;
  std::vector<double> prev_imu;
  std::vector<double> pose; 
  std::vector<double> prev_pose;
  std::vector<double> prev_iter_imu;  // Dane z poprzedniego wywołania calculate_orientation
  double quaternion_x_;
  double quaternion_y_;
  double quaternion_z_;
  double quaternion_w_;
  rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr publisher_tf_static_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscriber_pose_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscriber_imu_;
};

TfStaticPublisher::TfStaticPublisher() : Node("tf_static_publisher")
{
  // Inicjalizacja kwaternionu
  quaternion_x_ = 0.0;
  quaternion_y_ = 0.0;
  quaternion_z_ = 0.0;
  quaternion_w_ = 1.0;

  // Inicjalizacja wektorów
  imu = {0.0, 0.0, 0.0};
  prev_imu = {0.0, 0.0, 0.0};
  pose = {0.0, 0.0, 0.0};
  prev_pose = {0.0, 0.0, 0.0};
  prev_iter_imu = {0.0, 0.0, 0.0};

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
  // Zapisz poprzednią pozycję
  prev_pose = pose;
  
  // Pobierz nową pozycję z PoseStamped
  pose = {msg->pose.position.x,
          msg->pose.position.y,
          msg->pose.position.z};
  
  RCLCPP_INFO(this->get_logger(), "pose: x=%.3f, y=%.3f, z=%.3f", 
               pose[0], pose[1], pose[2]);
  // Jeśli mamy wystarczające dane, oblicz orientację
  RCLCPP_INFO(this->get_logger(), "imu: x=%.3f, y=%.3f, z=%.3f", 
               imu[0], imu[1], imu[2]);
  if (prev_pose != std::vector<double>{0.0, 0.0, 0.0} && 
      imu != std::vector<double>{0.0, 0.0, 0.0}) {
    calculate_orientation_from_imu();
    // Aktualizuj prev_iter_imu po obliczeniu orientacji
    prev_iter_imu = imu;
  }

  // Pobierz timestamp z wiadomości
  auto received_stamp = msg->header.stamp;
  
  // Przygotuj wiadomość do publikacji
  tf2_msgs::msg::TFMessage tf_msg;
  geometry_msgs::msg::TransformStamped transform;
    
  // Użyj timestamp z /vo_pose
  transform.header.stamp = received_stamp;
  transform.header.frame_id = "world";
  transform.child_frame_id = "dworld";
    
  // Translacja
  transform.transform.translation.x = 0.0;
  transform.transform.translation.y = 0.0;
  transform.transform.translation.z = 0.0;
    
  // Rotacja (quaternion)
  transform.transform.rotation.x = quaternion_x_;
  transform.transform.rotation.y = quaternion_y_;
  transform.transform.rotation.z = quaternion_z_;
  transform.transform.rotation.w = quaternion_w_;
    
  tf_msg.transforms.push_back(transform);
    
  publisher_tf_static_->publish(tf_msg);
    
  RCLCPP_DEBUG(this->get_logger(), "Published /tf_static with timestamp: %d.%09d", 
               received_stamp.sec, received_stamp.nanosec);
}

void TfStaticPublisher::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) 
{
  // Pobranie aktualnego czasu
  static rclcpp::Time prev_time = this->now();
  rclcpp::Time current_time = this->now();
  
  // Obliczenie delty czasu w sekundach
  double dt = (current_time - prev_time).seconds();
  
  // Pomiń pierwszą iterację (dt = 0)
  if (dt <= 0.0) {
    prev_time = current_time;
    return;
  }
  
  // Inicjalizacja wektorów jeśli są puste (statyczne zmienne zachowują stan)
  static std::vector<double> velocity = {0.0, 0.0, 0.0};
  static std::vector<double> prev_acceleration = {0.0, 0.0, 0.0};
  static std::vector<double> prev_velocity = {0.0, 0.0, 0.0};
  
  // Pobranie przyspieszenia liniowego z wiadomości IMU
  double ax = msg->linear_acceleration.x;
  double ay = msg->linear_acceleration.y;
  double az = msg->linear_acceleration.z - 9.81;  //WAŻNE: Odejmij grawitację od osi Z!
  
  // Krok 1: Całkowanie przyspieszenia do prędkości metodą trapezów
  // v(t) = v(t-1) + (a(t) + a(t-1)) * dt / 2
  velocity[0] += (ax + prev_acceleration[0]) * dt / 2.0;
  velocity[1] += (ay + prev_acceleration[1]) * dt / 2.0;
  velocity[2] += (az + prev_acceleration[2]) * dt / 2.0;
  
  // Krok 2: Całkowanie prędkości do pozycji metodą trapezów
  // s(t) = s(t-1) + (v(t) + v(t-1)) * dt / 2
  // TYLKO X i Y dla łódki (porusza się po płaszczyźnie XY)
  imu[0] = prev_imu[0] + (velocity[0] + prev_velocity[0]) * dt / 2.0;
  imu[1] = prev_imu[1] + (velocity[1] + prev_velocity[1]) * dt / 2.0;
  imu[2] = 0.0;  // Z zawsze = 0 dla łódki na wodzie!
  
  // Aktualizacja poprzednich wartości
  prev_imu = imu;
  prev_acceleration = {ax, ay, az};
  prev_velocity = velocity;
  
  // Aktualizacja czasu
  prev_time = current_time;
  
  RCLCPP_DEBUG(this->get_logger(), "IMU position: x=%.3f, y=%.3f, z=%.3f", 
               imu[0], imu[1], imu[2]);
}

void TfStaticPublisher::calculate_orientation_from_imu()
{
  RCLCPP_INFO(this->get_logger(), "Calculating orientation from IMU and VO data");
  // Sprawdzenie czy mamy dane
  if (imu.size() < 3 || prev_iter_imu.size() < 3 || 
      pose.size() < 3 || prev_pose.size() < 3) {
    RCLCPP_WARN(this->get_logger(), "Not enough data for orientation calculation");
    return;
  }

  // Konwersja danych do Eigen
  Eigen::Vector3d imu_vec(imu[0], imu[1], imu[2]);
  Eigen::Vector3d prev_iter_imu_vec(prev_iter_imu[0], prev_iter_imu[1], prev_iter_imu[2]);
  Eigen::Vector3d vo_pose(pose[0], pose[1], pose[2]);
  Eigen::Vector3d prev_vo_pose(prev_pose[0], prev_pose[1], prev_pose[2]);

  // Sprawdzenie czy dane się zmieniły
  if ((imu_vec - prev_iter_imu_vec).norm() < 1e-6 || 
      (vo_pose - prev_vo_pose).norm() < 1e-6) {
    RCLCPP_DEBUG(this->get_logger(), "Skipping: no significant change in data");
    return;
  }

  // Normalizacja i obliczenie przyrostów
  Eigen::Vector3d imu_norm = imu_vec.normalized();
  Eigen::Vector3d prev_iter_imu_norm = prev_iter_imu_vec.normalized();
  Eigen::Vector3d vo_norm = vo_pose.normalized();
  Eigen::Vector3d prev_vo_norm = prev_vo_pose.normalized();

  Eigen::Vector3d d_imu = (imu_norm - prev_iter_imu_norm);
  Eigen::Vector3d d_vo = (vo_norm - prev_vo_norm);

  // Sprawdzenie czy przyrosty nie są zbyt małe
  if (d_imu.norm() < 1e-6 || d_vo.norm() < 1e-6) {
    RCLCPP_DEBUG(this->get_logger(), "Skipping orientation calculation: increments too small");
    return;
  }

  d_imu.normalize();
  d_vo.normalize();

  // Sprawdzenie czy kierunki się nie pokrywają (iloczyn skalarny bliski 1 lub -1)
  double dot_product_imu = imu_norm.dot(d_imu);
  double dot_product_vo = vo_norm.dot(d_vo);
  
  const double threshold = 0.95; // Próg podobieństwa (cosinus ~18 stopni)
  
  if (std::abs(dot_product_imu) > threshold || std::abs(dot_product_vo) > threshold) {
    RCLCPP_DEBUG(this->get_logger(), 
                 "Skipping orientation calculation: directions too similar (IMU: %.3f, VO: %.3f)",
                 dot_product_imu, dot_product_vo);
    return;
  }

  // Parametry do optymalizacji: [roll, pitch, yaw]
  double angles[3] = {0.0, 0.0, 0.0};

  // Konfiguracja problemu Ceres
  ceres::Problem problem;
  
  ceres::CostFunction* cost_function =
      new ceres::AutoDiffCostFunction<OrientationCostFunctor, 1, 3>(
          new OrientationCostFunctor(imu_vec, prev_iter_imu_vec, vo_pose, prev_vo_pose));

  problem.AddResidualBlock(cost_function, nullptr, angles);

  // Ograniczenia na kąty
  problem.SetParameterLowerBound(angles, 0, -M_PI);      // roll
  problem.SetParameterUpperBound(angles, 0, M_PI);
  problem.SetParameterLowerBound(angles, 1, -M_PI/2);    // pitch
  problem.SetParameterUpperBound(angles, 1, M_PI/2);
  problem.SetParameterLowerBound(angles, 2, -M_PI);      // yaw
  problem.SetParameterUpperBound(angles, 2, M_PI);

  // Opcje solvera
  ceres::Solver::Options options;
  options.linear_solver_type = ceres::DENSE_QR;
  options.minimizer_progress_to_stdout = false;
  options.max_num_iterations = 100;

  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

  if (summary.termination_type == ceres::CONVERGENCE) {
    // Obliczenie macierzy rotacji z kątów Eulera
    double c_phi = cos(angles[0]);
    double s_phi = sin(angles[0]);
    double c_theta = cos(angles[1]);
    double s_theta = sin(angles[1]);
    double c_psi = cos(angles[2]);
    double s_psi = sin(angles[2]);

    Eigen::Matrix3d R;
    R(0, 0) = c_theta * c_psi;
    R(0, 1) = c_theta * s_psi;
    R(0, 2) = -s_theta;

    R(1, 0) = s_phi * s_theta * c_psi - c_phi * s_psi;
    R(1, 1) = s_phi * s_theta * s_psi + c_phi * c_psi;
    R(1, 2) = s_phi * c_theta;

    R(2, 0) = c_phi * s_theta * c_psi + s_phi * s_psi;
    R(2, 1) = c_phi * s_theta * s_psi - s_phi * c_psi;
    R(2, 2) = c_phi * c_theta;

    // Konwersja macierzy rotacji na kwaternion
    Eigen::Quaterniond q(R);

    // Zapisanie kwaternionu
    quaternion_x_ = q.x();
    quaternion_y_ = q.y();
    quaternion_z_ = q.z();
    quaternion_w_ = q.w();

    RCLCPP_INFO(this->get_logger(), 
                "Orientation calculated - Quaternion: x=%.6f, y=%.6f, z=%.6f, w=%.6f",
                quaternion_x_, quaternion_y_, quaternion_z_, quaternion_w_);
  } else {
    RCLCPP_WARN(this->get_logger(), "Orientation optimization failed");
  }
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TfStaticPublisher>());
  rclcpp::shutdown();
  return 0;
}