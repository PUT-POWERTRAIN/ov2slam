#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>

#include <chrono>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <mutex>
#include <string>

namespace fs = std::filesystem;

namespace {

std::string join_path(const std::string& dir, const std::string& file) {
  fs::path p(dir);
  p /= file;
  return p.string();
}

}  // namespace

class trajectory_csv_logger : public rclcpp::Node {
public:
  trajectory_csv_logger();
  ~trajectory_csv_logger() override;

private:
  void open_files();
  void write_header_if_needed(std::ofstream& out);
  void on_vo(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void on_gt(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void write_pose(std::ofstream& out, const geometry_msgs::msg::PoseStamped& msg);

  std::string vo_topic_;
  std::string gt_topic_;
  std::string output_dir_;
  std::string vo_filename_;
  std::string gt_filename_;
  bool flush_each_row_ = false;

  std::ofstream vo_out_;
  std::ofstream gt_out_;
  std::mutex io_mutex_;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr vo_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr gt_sub_;
};

trajectory_csv_logger::trajectory_csv_logger() : Node("trajectory_csv_logger") {
  this->declare_parameter("vo_topic", "/vo_pose");
  this->declare_parameter("gt_topic", "/gt_pose");
  this->declare_parameter("output_dir", "/datasets/exports");
  this->declare_parameter("vo_filename", "vo_pose.csv");
  this->declare_parameter("gt_filename", "gt_pose.csv");
  this->declare_parameter("flush_each_row", false);

  vo_topic_ = this->get_parameter("vo_topic").as_string();
  gt_topic_ = this->get_parameter("gt_topic").as_string();
  output_dir_ = this->get_parameter("output_dir").as_string();
  vo_filename_ = this->get_parameter("vo_filename").as_string();
  gt_filename_ = this->get_parameter("gt_filename").as_string();
  flush_each_row_ = this->get_parameter("flush_each_row").as_bool();

  open_files();

  vo_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      vo_topic_, 50, std::bind(&trajectory_csv_logger::on_vo, this, std::placeholders::_1));
  gt_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      gt_topic_, 50, std::bind(&trajectory_csv_logger::on_gt, this, std::placeholders::_1));

  RCLCPP_INFO(
      this->get_logger(),
      "Logging trajectories to %s (vo=%s gt=%s) from topics (vo=%s gt=%s)",
      output_dir_.c_str(),
      vo_filename_.c_str(),
      gt_filename_.c_str(),
      vo_topic_.c_str(),
      gt_topic_.c_str());
}

trajectory_csv_logger::~trajectory_csv_logger() {
  std::lock_guard<std::mutex> lock(io_mutex_);
  if (vo_out_.is_open()) vo_out_.close();
  if (gt_out_.is_open()) gt_out_.close();
}

void trajectory_csv_logger::open_files() {
  fs::create_directories(output_dir_);

  const std::string vo_path = join_path(output_dir_, vo_filename_);
  const std::string gt_path = join_path(output_dir_, gt_filename_);

  vo_out_.open(vo_path, std::ios::out | std::ios::trunc);
  if (!vo_out_.is_open()) {
    throw std::runtime_error("Failed to open VO CSV for writing: " + vo_path);
  }
  gt_out_.open(gt_path, std::ios::out | std::ios::trunc);
  if (!gt_out_.is_open()) {
    throw std::runtime_error("Failed to open GT CSV for writing: " + gt_path);
  }

  write_header_if_needed(vo_out_);
  write_header_if_needed(gt_out_);
}

void trajectory_csv_logger::write_header_if_needed(std::ofstream& out) {
  out << "stamp_sec,stamp_nanosec,frame_id,"
         "pos_x,pos_y,pos_z,ori_x,ori_y,ori_z,ori_w\n";
}

void trajectory_csv_logger::write_pose(std::ofstream& out, const geometry_msgs::msg::PoseStamped& msg) {
  out << msg.header.stamp.sec << "," << msg.header.stamp.nanosec << "," << msg.header.frame_id << ",";
  out << msg.pose.position.x << "," << msg.pose.position.y << "," << msg.pose.position.z << ",";
  out << msg.pose.orientation.x << "," << msg.pose.orientation.y << "," << msg.pose.orientation.z << ","
      << msg.pose.orientation.w << "\n";
  if (flush_each_row_) out.flush();
}

void trajectory_csv_logger::on_vo(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(io_mutex_);
  if (!vo_out_.is_open()) return;
  write_pose(vo_out_, *msg);
}

void trajectory_csv_logger::on_gt(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(io_mutex_);
  if (!gt_out_.is_open()) return;
  write_pose(gt_out_, *msg);
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<trajectory_csv_logger>();
    rclcpp::spin(node);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("trajectory_csv_logger"), "Error: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}

