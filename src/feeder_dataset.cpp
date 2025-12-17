#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <opencv2/opencv.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <fstream>
#include <limits>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace {

using SteadyClock = std::chrono::steady_clock;

class PlaybackClock {
public:
  void start(
      double dataset_t0_sec,
      double speed,
      const rclcpp::Time& ros_t0,
      const SteadyClock::time_point& steady_t0) {
    dataset_t0_sec_ = dataset_t0_sec;
    speed_ = speed;
    steady_t0_ = steady_t0;
    ros_t0_ = ros_t0;
  }

  void sleep_until_dataset_time(double dataset_ts_sec) const {
    const auto target = steady_time_for(dataset_ts_sec);
    while (rclcpp::ok()) {
      const auto now = SteadyClock::now();
      if (target <= now) return;
      const auto remaining = target - now;
      const auto step = std::min<SteadyClock::duration>(remaining, std::chrono::milliseconds(2));
      std::this_thread::sleep_for(step);
    }
  }

  rclcpp::Time ros_time_for(double dataset_ts_sec) const {
    const double scaled_offset = (dataset_ts_sec - dataset_t0_sec_) / speed_;
    return ros_t0_ + rclcpp::Duration::from_seconds(scaled_offset);
  }

  int64_t lateness_ns(double dataset_ts_sec) const {
    const auto target = steady_time_for(dataset_ts_sec);
    const auto now = SteadyClock::now();
    return std::chrono::duration_cast<std::chrono::nanoseconds>(now - target).count();
  }

private:
  SteadyClock::time_point steady_time_for(double dataset_ts_sec) const {
    const double scaled_offset = (dataset_ts_sec - dataset_t0_sec_) / speed_;
    const auto offset = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(scaled_offset));
    return steady_t0_ + offset;
  }

  double dataset_t0_sec_ = 0.0;
  double speed_ = 1.0;
  SteadyClock::time_point steady_t0_{};
  rclcpp::Time ros_t0_{0, 0, RCL_SYSTEM_TIME};
};

struct ImageSample {
  double ts_sec = 0.0;
  std::string name;
};

struct ImuSample {
  double ts_sec = 0.0;
  std::vector<double> data;
};

struct GtSample {
  double ts_sec = 0.0;
  // baseline.txt format: timestamp qx qy qz qw x y z
  std::array<double, 7> values{};
};

bool read_next_image(std::ifstream& file, ImageSample& out) {
  std::string line;
  while (std::getline(file, line)) {
    if (line.empty()) continue;
    std::istringstream iss(line);
    ImageSample sample;
    if (!(iss >> sample.ts_sec >> sample.name)) continue;
    out = std::move(sample);
    return true;
  }
  return false;
}

bool read_next_imu(std::ifstream& file, ImuSample& out) {
  std::string line;
  while (std::getline(file, line)) {
    if (line.empty()) continue;
    std::istringstream iss(line);
    ImuSample sample;
    double value = 0.0;
    while (iss >> value) sample.data.push_back(value);
    if (sample.data.empty()) continue;
    sample.ts_sec = sample.data[0];
    out = std::move(sample);
    return true;
  }
  return false;
}

bool read_next_gt(std::ifstream& file, GtSample& out, double& last_ts) {
  std::string line;
  while (std::getline(file, line)) {
    if (line.empty()) continue;
    std::istringstream iss(line);
    std::vector<double> values;
    double v = 0.0;
    while (iss >> v) values.push_back(v);
    if (values.size() < 8) continue;

    const double ts_sec = values[0];
    if (last_ts >= 0.0 && ts_sec < last_ts) continue;
    last_ts = ts_sec;

    GtSample sample;
    sample.ts_sec = ts_sec;
    sample.values = {values[1], values[2], values[3], values[4], values[5], values[6], values[7]};
    out = sample;
    return true;
  }
  return false;
}

template <typename SampleT, typename ReadFn>
std::optional<SampleT> read_first_at_or_after(std::ifstream& file, double start_ts, ReadFn read_next) {
  if (start_ts < 0.0) {
    SampleT s;
    if (read_next(file, s)) return s;
    return std::nullopt;
  }

  SampleT s;
  while (read_next(file, s)) {
    if (s.ts_sec >= start_ts) return s;
  }
  return std::nullopt;
}

}  // namespace

class feeder_dataset : public rclcpp::Node {
public:
  feeder_dataset();
  void run_once();
  bool loop() const { return loop_; }

private:
  sensor_msgs::msg::Image create_image_msg(const cv::Mat& img, const rclcpp::Time& stamp) const;
  sensor_msgs::msg::Imu create_imu_msg(const ImuSample& sample, const rclcpp::Time& stamp) const;

  void publish_image(const ImageSample& sample, const rclcpp::Time& stamp);
  void publish_imu(const ImuSample& sample, const rclcpp::Time& stamp);
  void publish_gt(const GtSample& sample, const rclcpp::Time& stamp);

  void reset_gt_path_state();

  bool enable_images_ = true;
  bool enable_stereo_ = true;
  bool enable_imu_ = true;
  bool enable_gt_ = true;
  bool loop_ = true;
  bool zero_origin_ = true;
  double playback_speed_ = 1.0;
  double start_timestamp_ = -1.0;  // dataset unix time (s); <0 disables
  int max_images_ = -1;            // max number of images to publish; <0 disables
  bool timing_debug_ = false;
  int timing_log_every_ = 1000;

  std::string images_folder_left_;
  std::string images_folder_right_;
  std::string timestamp_path_;
  std::string imu_path_;
  std::string gt_path_;
  std::string gt_frame_id_ = "world";

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_left_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_right_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_publisher_;

  visualization_msgs::msg::Marker marker_msg_;
  bool origin_set_ = false;
  double origin_x_ = 0.0;
  double origin_y_ = 0.0;
  double origin_z_ = 0.0;

  cv::Mat buffer_left_;
  cv::Mat buffer_right_;

  PlaybackClock clock_;
};

feeder_dataset::feeder_dataset() : Node("feeder_dataset") {
  this->declare_parameter("enable_images", true);
  this->declare_parameter("images_folder_left", "/datasets/left_images");
  this->declare_parameter("images_folder_right", "/datasets/right_images");
  this->declare_parameter("enable_stereo", true);
  this->declare_parameter("timestamp_path", "/datasets/timestamp.txt");

  this->declare_parameter("enable_imu", true);
  this->declare_parameter("imu_path", "/datasets/ahrs.txt");

  this->declare_parameter("enable_gt", true);
  this->declare_parameter("gt_path", "/datasets/baseline.txt");
  this->declare_parameter("gt_frame_id", "world");
  this->declare_parameter("zero_origin", true);

  this->declare_parameter("loop", true);
  this->declare_parameter("playback_speed", 1.0);
  this->declare_parameter("start_timestamp", -1.0);
  this->declare_parameter("max_images", -1);
  this->declare_parameter("timing_debug", false);
  this->declare_parameter("timing_log_every", 1000);

  enable_images_ = this->get_parameter("enable_images").as_bool();
  images_folder_left_ = this->get_parameter("images_folder_left").as_string();
  images_folder_right_ = this->get_parameter("images_folder_right").as_string();
  enable_stereo_ = this->get_parameter("enable_stereo").as_bool();
  timestamp_path_ = this->get_parameter("timestamp_path").as_string();

  enable_imu_ = this->get_parameter("enable_imu").as_bool();
  imu_path_ = this->get_parameter("imu_path").as_string();

  enable_gt_ = this->get_parameter("enable_gt").as_bool();
  gt_path_ = this->get_parameter("gt_path").as_string();
  gt_frame_id_ = this->get_parameter("gt_frame_id").as_string();
  zero_origin_ = this->get_parameter("zero_origin").as_bool();

  loop_ = this->get_parameter("loop").as_bool();
  playback_speed_ = this->get_parameter("playback_speed").as_double();
  start_timestamp_ = this->get_parameter("start_timestamp").as_double();
  max_images_ = this->get_parameter("max_images").as_int();
  timing_debug_ = this->get_parameter("timing_debug").as_bool();
  timing_log_every_ = this->get_parameter("timing_log_every").as_int();

  if (timing_log_every_ <= 0) timing_log_every_ = 1000;

  if (playback_speed_ <= 0.0) {
    RCLCPP_WARN(this->get_logger(), "Invalid playback_speed=%.3f, forcing 1.0", playback_speed_);
    playback_speed_ = 1.0;
  }

  if (max_images_ == 0) {
    RCLCPP_WARN(this->get_logger(), "max_images=0 means no images will be published; did you mean -1 (disable limit)?");
  }

  if (enable_images_) {
    image_publisher_left_ = this->create_publisher<sensor_msgs::msg::Image>("image_left_raw_data", 10);
    if (enable_stereo_) {
      image_publisher_right_ = this->create_publisher<sensor_msgs::msg::Image>("image_right_raw_data", 10);
    }
  }
  if (enable_imu_) {
    imu_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("imu_data", 50);
  }
  if (enable_gt_) {
    pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("gt_pose", 10);
    marker_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("gt_traj", 10);
  }

  marker_msg_.header.frame_id = gt_frame_id_;
  marker_msg_.ns = "gt";
  marker_msg_.id = 0;
  marker_msg_.type = visualization_msgs::msg::Marker::LINE_STRIP;
  marker_msg_.action = visualization_msgs::msg::Marker::ADD;
  marker_msg_.scale.x = 0.02;
  marker_msg_.color.a = 1.0;
  marker_msg_.color.r = 1.0;
  marker_msg_.color.g = 0.2;
  marker_msg_.color.b = 0.2;
  marker_msg_.pose.orientation.w = 1.0;

  RCLCPP_INFO(
      this->get_logger(),
      "feeder_dataset: images=%d stereo=%d imu=%d gt=%d loop=%d speed=%.3f start_ts=%.9f max_images=%d",
      enable_images_,
      enable_stereo_,
      enable_imu_,
      enable_gt_,
      loop_,
      playback_speed_,
      start_timestamp_,
      max_images_);
}

sensor_msgs::msg::Image feeder_dataset::create_image_msg(const cv::Mat& img, const rclcpp::Time& stamp) const {
  sensor_msgs::msg::Image msg;
  msg.header.stamp = stamp;
  msg.height = img.rows;
  msg.width = img.cols;
  msg.encoding = "bgr8";
  msg.is_bigendian = false;
  msg.step = static_cast<sensor_msgs::msg::Image::_step_type>(img.cols * img.elemSize());
  msg.data.assign(img.datastart, img.dataend);
  return msg;
}

sensor_msgs::msg::Imu feeder_dataset::create_imu_msg(const ImuSample& sample, const rclcpp::Time& stamp) const {
  sensor_msgs::msg::Imu msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = "world";

  const auto& d = sample.data;
  if (d.size() >= 11) {
    msg.orientation.x = d[1];
    msg.orientation.y = d[2];
    msg.orientation.z = d[3];
    msg.orientation.w = d[4];

    msg.angular_velocity.x = d[5];
    msg.angular_velocity.y = d[6];
    msg.angular_velocity.z = d[7];

    msg.linear_acceleration.x = d[8];
    msg.linear_acceleration.y = d[9];
    msg.linear_acceleration.z = d[10];
  }

  const std::array<double, 9> covariance_array = {-1.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, -1.0};
  std::copy(covariance_array.begin(), covariance_array.end(), msg.orientation_covariance.begin());
  std::copy(covariance_array.begin(), covariance_array.end(), msg.angular_velocity_covariance.begin());
  std::copy(covariance_array.begin(), covariance_array.end(), msg.linear_acceleration_covariance.begin());

  return msg;
}

void feeder_dataset::publish_image(const ImageSample& sample, const rclcpp::Time& stamp) {
  if (!enable_images_) return;

  const std::string left_path = images_folder_left_ + "/" + sample.name + ".png";
  buffer_left_ = cv::imread(left_path, cv::IMREAD_COLOR);
  if (buffer_left_.empty()) {
    RCLCPP_ERROR(this->get_logger(), "Cannot load image %s", left_path.c_str());
    return;
  }

  auto left_msg = create_image_msg(buffer_left_, stamp);
  left_msg.header.frame_id = "cam0";

  if (enable_stereo_) {
    const std::string right_path = images_folder_right_ + "/" + sample.name + ".png";
    buffer_right_ = cv::imread(right_path, cv::IMREAD_COLOR);
    if (buffer_right_.empty()) {
      RCLCPP_ERROR(this->get_logger(), "Cannot load stereo image %s", right_path.c_str());
      return;
    }
    auto right_msg = create_image_msg(buffer_right_, stamp);
    right_msg.header.frame_id = "cam1";

    image_publisher_left_->publish(left_msg);
    image_publisher_right_->publish(right_msg);
  } else {
    image_publisher_left_->publish(left_msg);
  }
}

void feeder_dataset::publish_imu(const ImuSample& sample, const rclcpp::Time& stamp) {
  if (!enable_imu_) return;
  imu_publisher_->publish(create_imu_msg(sample, stamp));
}

void feeder_dataset::reset_gt_path_state() {
  marker_msg_.points.clear();
  origin_set_ = false;
  origin_x_ = origin_y_ = origin_z_ = 0.0;
}

void feeder_dataset::publish_gt(const GtSample& sample, const rclcpp::Time& stamp) {
  if (!enable_gt_) return;

  geometry_msgs::msg::PoseStamped pose_msg;
  pose_msg.header.stamp = stamp;
  pose_msg.header.frame_id = gt_frame_id_;

  // baseline.txt format: timestamp qx qy qz qw x y z
  pose_msg.pose.orientation.x = sample.values[0];
  pose_msg.pose.orientation.y = sample.values[1];
  pose_msg.pose.orientation.z = sample.values[2];
  pose_msg.pose.orientation.w = sample.values[3];

  pose_msg.pose.position.x = sample.values[4];
  pose_msg.pose.position.y = sample.values[5];
  pose_msg.pose.position.z = sample.values[6];

  if (zero_origin_ && !origin_set_) {
    origin_x_ = pose_msg.pose.position.x;
    origin_y_ = pose_msg.pose.position.y;
    origin_z_ = pose_msg.pose.position.z;
    origin_set_ = true;
  }

  if (zero_origin_) {
    pose_msg.pose.position.x -= origin_x_;
    pose_msg.pose.position.y -= origin_y_;
    pose_msg.pose.position.z -= origin_z_;
  }

  pose_publisher_->publish(pose_msg);

  geometry_msgs::msg::Point p;
  p.x = pose_msg.pose.position.x;
  p.y = pose_msg.pose.position.y;
  p.z = pose_msg.pose.position.z;

  marker_msg_.header.frame_id = gt_frame_id_;
  marker_msg_.header.stamp = stamp;
  marker_msg_.points.push_back(p);
  marker_publisher_->publish(marker_msg_);
}

void feeder_dataset::run_once() {
  const auto steady_program_start = SteadyClock::now();
  const rclcpp::Time ros_program_start = rclcpp::Clock(RCL_SYSTEM_TIME).now();

  std::optional<ImageSample> next_image;
  std::optional<ImuSample> next_imu;
  std::optional<GtSample> next_gt;
  std::optional<double> stop_dataset_ts;

  std::ifstream image_file;
  std::ifstream imu_file;
  std::ifstream gt_file;
  double gt_last_ts = -1.0;

  if (enable_images_) {
    image_file.open(timestamp_path_);
    if (!image_file.is_open()) {
      throw std::runtime_error("Cannot open timestamp file: " + timestamp_path_);
    }
    next_image = read_first_at_or_after<ImageSample>(
        image_file, start_timestamp_, [](std::ifstream& f, ImageSample& s) { return read_next_image(f, s); });
  }

  if (enable_imu_) {
    imu_file.open(imu_path_);
    if (!imu_file.is_open()) {
      throw std::runtime_error("Cannot open IMU file: " + imu_path_);
    }
    next_imu = read_first_at_or_after<ImuSample>(
        imu_file, start_timestamp_, [](std::ifstream& f, ImuSample& s) { return read_next_imu(f, s); });
  }

  if (enable_gt_) {
    gt_file.open(gt_path_);
    if (!gt_file.is_open()) {
      throw std::runtime_error("Cannot open GT file: " + gt_path_);
    }
    if (start_timestamp_ < 0.0) {
      GtSample s{};
      if (read_next_gt(gt_file, s, gt_last_ts)) next_gt = s;
    } else {
      GtSample s{};
      while (read_next_gt(gt_file, s, gt_last_ts)) {
        if (s.ts_sec >= start_timestamp_) {
          next_gt = s;
          break;
        }
      }
    }
  }

  // Reference t0 is the earliest available acquisition time across enabled streams.
  //
  // Rationale (Pohang Canal Dataset): different sensors can start at different times (e.g. IMU earlier than stereo).
  // By anchoring playback to the earliest timestamp, we preserve the original time axis and avoid "catch-up bursts"
  // of earlier streams (which would happen if we anchored to the first camera frame).
  // Side effect: if the camera starts later than IMU/GT, the first image will be published after that real-time delay.
  //
  // When start_timestamp is set (>= 0), the feeder first skips samples < start_timestamp for each enabled stream and
  // anchors t0 to the earliest timestamp among the first remaining samples. This lets you start from the middle of
  // a dataset without waiting for earlier frames to play out.
  double dataset_t0 = std::numeric_limits<double>::infinity();
  if (next_image) dataset_t0 = std::min(dataset_t0, next_image->ts_sec);
  if (next_imu) dataset_t0 = std::min(dataset_t0, next_imu->ts_sec);
  if (next_gt) dataset_t0 = std::min(dataset_t0, next_gt->ts_sec);

  if (!std::isfinite(dataset_t0)) {
    RCLCPP_WARN(this->get_logger(), "No samples to publish (all inputs empty?)");
    return;
  }

  reset_gt_path_state();
  clock_.start(dataset_t0, playback_speed_, ros_program_start, steady_program_start);
  const std::string first_image_ts = next_image ? std::to_string(next_image->ts_sec) : "n/a";
  const std::string first_imu_ts = next_imu ? std::to_string(next_imu->ts_sec) : "n/a";
  const std::string first_gt_ts = next_gt ? std::to_string(next_gt->ts_sec) : "n/a";
  RCLCPP_INFO(
      this->get_logger(),
      "Clock start: ds_t0=%.9f ros_t0=%.9f speed=%.3f (first: image=%s imu=%s gt=%s)",
      dataset_t0,
      ros_program_start.seconds(),
      playback_speed_,
      first_image_ts.c_str(),
      first_imu_ts.c_str(),
      first_gt_ts.c_str());

  size_t published_images = 0;
  size_t published_imu = 0;
  size_t published_gt = 0;
  bool logged_first_image = false;
  bool logged_first_imu = false;
  bool logged_first_gt = false;

  while (rclcpp::ok()) {
    double next_ts = std::numeric_limits<double>::infinity();
    enum class NextType { None, Image, Imu, Gt };
    NextType next_type = NextType::None;

    if (next_imu && (!stop_dataset_ts || next_imu->ts_sec <= *stop_dataset_ts) && next_imu->ts_sec < next_ts) {
      next_ts = next_imu->ts_sec;
      next_type = NextType::Imu;
    }
    if (next_image && (!stop_dataset_ts || next_image->ts_sec <= *stop_dataset_ts) && next_image->ts_sec < next_ts) {
      next_ts = next_image->ts_sec;
      next_type = NextType::Image;
    }
    if (next_gt && (!stop_dataset_ts || next_gt->ts_sec <= *stop_dataset_ts) && next_gt->ts_sec < next_ts) {
      next_ts = next_gt->ts_sec;
      next_type = NextType::Gt;
    }

    if (next_type == NextType::None) break;

    clock_.sleep_until_dataset_time(next_ts);
    const rclcpp::Time stamp = clock_.ros_time_for(next_ts);
    const int64_t late_ns = timing_debug_ ? clock_.lateness_ns(next_ts) : 0;

    switch (next_type) {
      case NextType::Imu: {
        publish_imu(*next_imu, stamp);
        published_imu++;
        if (!logged_first_imu) {
          logged_first_imu = true;
          RCLCPP_INFO(
              this->get_logger(),
              "First IMU: ts_ds=%.9f stamp=%.9f lateness_ns=%ld",
              next_ts,
              stamp.seconds(),
              static_cast<long>(clock_.lateness_ns(next_ts)));
        }
        ImuSample s;
        if (read_next_imu(imu_file, s)) {
          next_imu = std::move(s);
        } else {
          next_imu.reset();
        }
      } break;
      case NextType::Image: {
        publish_image(*next_image, stamp);
        published_images++;
        if (!logged_first_image) {
          logged_first_image = true;
          RCLCPP_INFO(
              this->get_logger(),
              "First Image: ts_ds=%.9f stamp=%.9f name=%s lateness_ns=%ld",
              next_ts,
              stamp.seconds(),
              next_image->name.c_str(),
              static_cast<long>(clock_.lateness_ns(next_ts)));
        }
        ImageSample s;
        if (max_images_ > 0 && static_cast<int>(published_images) >= max_images_) {
          // Stop condition to limit runtime/testing cost:
          // once we publish the N-th image we stop reading further images and stop the run at that dataset time.
          stop_dataset_ts = next_ts;
          next_image.reset();
        } else {
          if (read_next_image(image_file, s)) {
            next_image = std::move(s);
          } else {
            next_image.reset();
          }
        }
      } break;
      case NextType::Gt: {
        publish_gt(*next_gt, stamp);
        published_gt++;
        if (!logged_first_gt) {
          logged_first_gt = true;
          RCLCPP_INFO(
              this->get_logger(),
              "First GT: ts_ds=%.9f stamp=%.9f lateness_ns=%ld",
              next_ts,
              stamp.seconds(),
              static_cast<long>(clock_.lateness_ns(next_ts)));
        }
        GtSample s;
        if (read_next_gt(gt_file, s, gt_last_ts)) {
          next_gt = s;
        } else {
          next_gt.reset();
        }
      } break;
      case NextType::None:
        break;
    }

    const size_t total = published_images + published_imu + published_gt;
    if (total > 0 && total % static_cast<size_t>(timing_log_every_) == 0) {
      if (timing_debug_) {
        RCLCPP_INFO(
            this->get_logger(),
            "Published: images=%zu imu=%zu gt=%zu (lateness=%ld ns)",
            published_images,
            published_imu,
            published_gt,
            static_cast<long>(late_ns));
      } else {
        RCLCPP_INFO(this->get_logger(), "Published: images=%zu imu=%zu gt=%zu", published_images, published_imu, published_gt);
      }
    }
  }

  RCLCPP_INFO(this->get_logger(), "DONE. Published: images=%zu imu=%zu gt=%zu", published_images, published_imu, published_gt);
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  try {
    auto feeder = std::make_shared<feeder_dataset>();
    do {
      feeder->run_once();
    } while (rclcpp::ok() && feeder->loop());
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("feeder_dataset"), "Error: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
