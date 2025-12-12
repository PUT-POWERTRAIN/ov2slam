#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <memory>
#include <chrono>

class feeder_gt : public rclcpp::Node {
public:
    feeder_gt();
    bool send_gt_data();
    bool loop_;

private:
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_publisher_;

    std::string gt_path_;
    std::string frame_id_;
    bool zero_origin_;
    bool origin_set_;
    double origin_x_;
    double origin_y_;
    double origin_z_;

    visualization_msgs::msg::Marker marker_msg_;
};

feeder_gt::feeder_gt() : Node("feeder_gt_node") {
    this->declare_parameter("gt_path", "/datasets/gt.txt");
    this->declare_parameter("frame_id", "world");
    this->declare_parameter("loop", true);
    this->declare_parameter("zero_origin", true);

    gt_path_ = this->get_parameter("gt_path").as_string();
    frame_id_ = this->get_parameter("frame_id").as_string();
    loop_ = this->get_parameter("loop").as_bool();
    zero_origin_ = this->get_parameter("zero_origin").as_bool();
    origin_set_ = false;
    origin_x_ = origin_y_ = origin_z_ = 0.0;

    pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("gt_pose", 10);
    marker_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("gt_traj", 10);

    marker_msg_.header.frame_id = frame_id_;
    marker_msg_.ns = "gt";
    marker_msg_.id = 0;
    marker_msg_.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker_msg_.action = visualization_msgs::msg::Marker::ADD;
    marker_msg_.scale.x = 0.02;
    marker_msg_.color.a = 1.0;
    marker_msg_.color.r = 1.0;
    marker_msg_.color.g = 0.2;
    marker_msg_.color.b = 0.2;
    marker_msg_.pose.orientation.w = 1.0; // identity rotation
}

bool feeder_gt::send_gt_data() {
    std::ifstream gt_file(gt_path_);
    if (!gt_file.is_open()) {
        RCLCPP_ERROR(this->get_logger(), "Cannot open GT file: %s", gt_path_.c_str());
        return false;
    }
    RCLCPP_INFO(this->get_logger(), "Loaded GT data from: %s", gt_path_.c_str());

    // Always reset path and origin at the start of a run to avoid stacking multiple loops.
    marker_msg_.points.clear();
    origin_set_ = false;
    double prev_timestamp = -1.0;

    std::string line;
    double first_timestamp = -1.0;
    rclcpp::Time start_time = this->now();
    size_t frame_count = 0;

    while (std::getline(gt_file, line)) {
        if (!rclcpp::ok()) return false;
        if (line.empty()) continue;

        std::istringstream iss(line);
        std::vector<double> values;
        double value;
        while (iss >> value) {
            values.push_back(value);
        }

        if (values.size() < 8) {
            RCLCPP_WARN(this->get_logger(), "Invalid GT line (expected timestamp + 7 values), skipping: %s", line.c_str());
            continue;
        }

        double timestamp_sec = values[0];

        if (prev_timestamp >= 0.0 && timestamp_sec < prev_timestamp) {
            RCLCPP_WARN(this->get_logger(),
                        "Non-monotonic GT timestamp: prev=%.9f current=%.9f (skipping)",
                        prev_timestamp, timestamp_sec);
            continue;
        }
        prev_timestamp = timestamp_sec;

        if (first_timestamp < 0) {
            first_timestamp = timestamp_sec;
            start_time = this->now();
        }

        double offset = timestamp_sec - first_timestamp;
        rclcpp::Time target_time = start_time + rclcpp::Duration::from_seconds(offset);

        rclcpp::Time now = this->now();
        if (target_time > now) {
            rclcpp::sleep_for(std::chrono::nanoseconds((target_time - now).nanoseconds()));
        }

        geometry_msgs::msg::PoseStamped pose_msg;
        pose_msg.header.stamp = target_time;
        pose_msg.header.frame_id = frame_id_;

        // Fixed format: timestamp qx qy qz qw x y z
        pose_msg.pose.orientation.x = values[1];
        pose_msg.pose.orientation.y = values[2];
        pose_msg.pose.orientation.z = values[3];
        pose_msg.pose.orientation.w = values[4];

        pose_msg.pose.position.x = values[5];
        pose_msg.pose.position.y = values[6];
        pose_msg.pose.position.z = values[7];

        if (zero_origin_ && !origin_set_) {
            origin_x_ = pose_msg.pose.position.x;
            origin_y_ = pose_msg.pose.position.y;
            origin_z_ = pose_msg.pose.position.z;
            origin_set_ = true;
            RCLCPP_INFO(this->get_logger(), "GT origin set to first pose: [%.3f, %.3f, %.3f]", origin_x_, origin_y_, origin_z_);
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
        marker_msg_.header.frame_id = frame_id_;
        marker_msg_.header.stamp = target_time;
        marker_msg_.points.push_back(p);
        marker_publisher_->publish(marker_msg_);

        frame_count++;
        if (frame_count <= 5 || frame_count % 100 == 0) {
            RCLCPP_INFO(this->get_logger(),
                        "[GT] frame=%zu ts_ds=%.9f target=%.9f now=%.9f origin_set=%d pos=(%.3f,%.3f,%.3f)",
                        frame_count, timestamp_sec, target_time.seconds(), this->now().seconds(),
                        origin_set_,
                        pose_msg.pose.position.x, pose_msg.pose.position.y, pose_msg.pose.position.z);
        }
    }

    RCLCPP_INFO(this->get_logger(), "GT feeder finished. Published %zu poses", frame_count);
    gt_file.close();
    return true;
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    try {
        auto feeder = std::make_shared<feeder_gt>();
        bool ok = feeder->send_gt_data();
        if (feeder->loop_ && ok) {
            while (rclcpp::ok()) feeder->send_gt_data();
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("feeder_gt"), "Error: %s", e.what());
        rclcpp::shutdown();
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}
