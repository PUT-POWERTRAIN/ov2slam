#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <algorithm>
#include <deque>
#include <vector>
#include <memory>
#include <optional>
#include <string>

#include <Eigen/Dense>

namespace {

tf2::Quaternion to_tf2(const geometry_msgs::msg::Quaternion& q) {
  return tf2::Quaternion(q.x, q.y, q.z, q.w);
}

tf2::Quaternion normalized(tf2::Quaternion q) {
  if (q.length2() <= 0.0) return tf2::Quaternion(0, 0, 0, 1);
  q.normalize();
  return q;
}

struct ImuQuatSample {
  rclcpp::Time stamp{0, 0, RCL_SYSTEM_TIME};
  tf2::Quaternion q{0, 0, 0, 1};
};

struct PoseSample {
  rclcpp::Time stamp{0, 0, RCL_SYSTEM_TIME};
  Eigen::Vector3d p{0.0, 0.0, 0.0};
  tf2::Quaternion q{0, 0, 0, 1};
};

// Returns IMU orientation at target time using nearest-neighbor / linear-time slerp interpolation.
// If we cannot bracket the timestamp, falls back to nearest available sample.
std::optional<tf2::Quaternion> interpolate_imu_quat(const std::deque<ImuQuatSample>& buf, const rclcpp::Time& target) {
  if (buf.empty()) return std::nullopt;

  if (target <= buf.front().stamp) return buf.front().q;
  if (target >= buf.back().stamp) return buf.back().q;

  // Find the first element with stamp >= target.
  auto it = std::lower_bound(
      buf.begin(),
      buf.end(),
      target,
      [](const ImuQuatSample& s, const rclcpp::Time& t) { return s.stamp < t; });

  if (it == buf.begin()) return it->q;
  if (it == buf.end()) return buf.back().q;

  const auto& b = *it;
  const auto& a = *(it - 1);
  const double dt = (b.stamp - a.stamp).seconds();
  if (dt <= 1e-9) return b.q;
  const double alpha = std::clamp((target - a.stamp).seconds() / dt, 0.0, 1.0);
  return normalized(a.q.slerp(b.q, alpha));
}

std::optional<Eigen::Matrix3d> kabsch_rotation(
    const std::vector<Eigen::Vector3d>& a,
    const std::vector<Eigen::Vector3d>& b) {
  if (a.size() != b.size() || a.size() < 3) return std::nullopt;

  Eigen::Matrix3d H = Eigen::Matrix3d::Zero();
  for (size_t i = 0; i < a.size(); ++i) {
    H += a[i] * b[i].transpose();
  }

  Eigen::JacobiSVD<Eigen::Matrix3d> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
  const Eigen::Matrix3d U = svd.matrixU();
  Eigen::Matrix3d V = svd.matrixV();

  Eigen::Matrix3d R = V * U.transpose();
  if (R.determinant() < 0.0) {
    V.col(2) *= -1.0;
    R = V * U.transpose();
  }
  return R;
}

tf2::Quaternion eigen_to_tf2(const Eigen::Quaterniond& q) {
  return tf2::Quaternion(q.x(), q.y(), q.z(), q.w());
}

}  // namespace

class ahrs_align_tf : public rclcpp::Node {
public:
  ahrs_align_tf();

private:
  void on_imu(const sensor_msgs::msg::Imu::SharedPtr msg);
  void on_vo(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void on_gt(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void maybe_publish();

  std::string imu_topic_;
  std::string vo_topic_;
  std::string gt_topic_;
  std::string parent_frame_;
  std::string child_frame_;
  bool published_ = false;

  std::deque<ImuQuatSample> imu_buf_;
  size_t imu_buf_max_ = 20000;  // ~200s at 100Hz (bounded to avoid unbounded growth if VO starts late)
  std::deque<PoseSample> vo_buf_;
  std::deque<PoseSample> gt_buf_;
  size_t pose_buf_max_ = 20000;
  std::optional<PoseSample> first_vo_;
  std::optional<PoseSample> first_gt_;
  int min_pairs_ = 30;
  int max_pairs_ = 200;
  double match_tolerance_sec_ = 0.05;

  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> broadcaster_;

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr vo_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr gt_sub_;
};

ahrs_align_tf::ahrs_align_tf() : Node("ahrs_align_tf") {
  this->declare_parameter("imu_topic", "/imu_data");
  this->declare_parameter("vo_topic", "/vo_pose");
  this->declare_parameter("gt_topic", "/gt_pose");
  this->declare_parameter("parent_frame", "world_ahrs");
  this->declare_parameter("child_frame", "world");
  this->declare_parameter("min_pairs", 10);
  this->declare_parameter("max_pairs", 200);
  this->declare_parameter("match_tolerance_sec", 0.05);

  imu_topic_ = this->get_parameter("imu_topic").as_string();
  vo_topic_ = this->get_parameter("vo_topic").as_string();
  gt_topic_ = this->get_parameter("gt_topic").as_string();
  parent_frame_ = this->get_parameter("parent_frame").as_string();
  child_frame_ = this->get_parameter("child_frame").as_string();
  min_pairs_ = this->get_parameter("min_pairs").as_int();
  max_pairs_ = this->get_parameter("max_pairs").as_int();
  match_tolerance_sec_ = this->get_parameter("match_tolerance_sec").as_double();

  if (min_pairs_ < 3) min_pairs_ = 3;
  if (max_pairs_ < min_pairs_) max_pairs_ = min_pairs_;
  if (match_tolerance_sec_ <= 0.0) match_tolerance_sec_ = 0.05;

  broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

  imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      imu_topic_, rclcpp::SensorDataQoS(), std::bind(&ahrs_align_tf::on_imu, this, std::placeholders::_1));
  vo_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      vo_topic_, 10, std::bind(&ahrs_align_tf::on_vo, this, std::placeholders::_1));
  gt_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      gt_topic_, 10, std::bind(&ahrs_align_tf::on_gt, this, std::placeholders::_1));

  RCLCPP_INFO(
      this->get_logger(),
      "Waiting to publish static TF: parent=%s child=%s (imu=%s vo=%s gt=%s)",
      parent_frame_.c_str(),
      child_frame_.c_str(),
      imu_topic_.c_str(),
      vo_topic_.c_str(),
      gt_topic_.c_str());
}

void ahrs_align_tf::on_imu(const sensor_msgs::msg::Imu::SharedPtr msg) {
  if (published_) return;

  ImuQuatSample s;
  s.stamp = msg->header.stamp;
  s.q = normalized(to_tf2(msg->orientation));

  // IMU data for this dataset is timestamped in playback-time already. Keep a bounded buffer.
  imu_buf_.push_back(s);
  if (imu_buf_.size() > imu_buf_max_) imu_buf_.pop_front();

  maybe_publish();
}

void ahrs_align_tf::on_vo(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
  if (published_) return;
  PoseSample s;
  s.stamp = msg->header.stamp;
  s.p = Eigen::Vector3d(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
  s.q = normalized(to_tf2(msg->pose.orientation));

  vo_buf_.push_back(s);
  if (vo_buf_.size() > pose_buf_max_) vo_buf_.pop_front();

  if (!first_vo_) first_vo_ = s;
  maybe_publish();
}

void ahrs_align_tf::on_gt(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
  if (published_) return;
  PoseSample s;
  s.stamp = msg->header.stamp;
  s.p = Eigen::Vector3d(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
  s.q = normalized(to_tf2(msg->pose.orientation));

  gt_buf_.push_back(s);
  if (gt_buf_.size() > pose_buf_max_) gt_buf_.pop_front();

  if (!first_gt_) first_gt_ = s;
  maybe_publish();
}

void ahrs_align_tf::maybe_publish() {
  if (published_) return;
  if (!first_vo_) return;

  // Why this node exists:
  // OV2SLAM publishes its outputs in a fixed "world" frame that is arbitrary (VO world).
  // For quick debugging/measurement (e.g. checking if the trajectory is mirrored/rotated vs AHRS/GT),
  // it’s useful to visualize the VO output in an AHRS-aligned frame without touching OV2SLAM internals.
  //
  // We do that by publishing ONE static TF (rotation only, no translation):
  //   parent_frame (world_ahrs) -> child_frame (world)
  // so RViz can use `world_ahrs` as Fixed Frame and automatically rotate all `frame_id=world` data.
  //
  // This is intentionally not a full IMU-camera calibration or a proper fusion solution.
  // It is a "measurement tool" to make coordinate-frame mismatches visible.

  tf2::Quaternion q_align{0, 0, 0, 1};
  std::string method = "gt_kabsch";
  rclcpp::Time vo_t = first_vo_->stamp;

  // Goal for RViz overlay:
  // - GT stays "as-is" in world_ahrs (Fixed Frame)
  // - OV2SLAM stays publishing in world (unchanged)
  // - We publish exactly one static TF world_ahrs->world so RViz rotates ONLY the VO outputs into world_ahrs
  //
  // The most robust way to do that (without guessing AHRS conventions/extrinsics) is to fit a pure rotation
  // that best aligns the two trajectories using their positions (Kabsch).
  if (first_gt_ && !gt_buf_.empty() && !vo_buf_.empty()) {
    std::vector<Eigen::Vector3d> a_vo;
    std::vector<Eigen::Vector3d> b_gt;
    a_vo.reserve(static_cast<size_t>(max_pairs_));
    b_gt.reserve(static_cast<size_t>(max_pairs_));

    const Eigen::Vector3d vo0 = first_vo_->p;
    const Eigen::Vector3d gt0 = first_gt_->p;

    // Match samples in time-order to avoid pairing many GT samples to the same VO sample (degenerate fit).
    size_t i = 0;
    size_t j = 0;
    while (i < vo_buf_.size() && j < gt_buf_.size() && static_cast<int>(a_vo.size()) < max_pairs_) {
      const auto& vo = vo_buf_[i];
      const auto& gt = gt_buf_[j];
      const double dt = (vo.stamp - gt.stamp).seconds();

      if (dt < -match_tolerance_sec_) {
        ++i;
        continue;
      }
      if (dt > match_tolerance_sec_) {
        ++j;
        continue;
      }

      a_vo.push_back(vo.p - vo0);
      b_gt.push_back(gt.p - gt0);
      vo_t = vo.stamp;
      ++i;
      ++j;
    }

    if (static_cast<int>(a_vo.size()) < min_pairs_) {
      // GT exists, but we don't have enough matched pairs yet -> wait (do NOT publish AHRS fallback).
      return;
    }

    const auto R_opt = kabsch_rotation(a_vo, b_gt);
    if (!R_opt) return;
    const Eigen::Quaterniond q_eig(*R_opt);
    q_align = normalized(eigen_to_tf2(q_eig));
  } else {
    // Fallback (only when GT isn't available): align using AHRS orientation at VO start time.
    method = "ahrs";
    if (imu_buf_.empty()) return;
    const auto q_ahrs_opt = interpolate_imu_quat(imu_buf_, vo_t);
    if (!q_ahrs_opt) return;
    const tf2::Quaternion q_ahrs = *q_ahrs_opt;
    const tf2::Quaternion q_vo = first_vo_->q;
    q_align = normalized(q_ahrs * q_vo.inverse());
  }

  geometry_msgs::msg::TransformStamped t;
  t.header.stamp = this->get_clock()->now();
  t.header.frame_id = parent_frame_;
  t.child_frame_id = child_frame_;
  t.transform.translation.x = 0.0;
  t.transform.translation.y = 0.0;
  t.transform.translation.z = 0.0;
  t.transform.rotation.x = q_align.x();
  t.transform.rotation.y = q_align.y();
  t.transform.rotation.z = q_align.z();
  t.transform.rotation.w = q_align.w();

  broadcaster_->sendTransform(t);
  published_ = true;

  const double imu_dt = (imu_buf_.empty()) ? 0.0 : (imu_buf_.back().stamp - vo_t).seconds();

  RCLCPP_INFO(
      this->get_logger(),
      "Published static TF %s -> %s method=%s at vo_t=%.9f (imu_buf_dt_to_last=%.6fs) q=[%.6f %.6f %.6f %.6f]",
      parent_frame_.c_str(),
      child_frame_.c_str(),
      method.c_str(),
      vo_t.seconds(),
      imu_dt,
      t.transform.rotation.x,
      t.transform.rotation.y,
      t.transform.rotation.z,
      t.transform.rotation.w);
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ahrs_align_tf>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
