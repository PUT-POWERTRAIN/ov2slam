#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <memory>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

class feeder_imu : public rclcpp::Node {
public:
    feeder_imu();
    void send_imu_data();
    bool loop_;

private:
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
    std::string imu_data_path_;    

    sensor_msgs::msg::Imu createImuMsg(const std::vector<double>& imu_data, const rclcpp::Time& timestamp);
};

feeder_imu::feeder_imu() : Node("feeder_imu_node") {
    this->declare_parameter("imu_data", "/datasets/ahrs.txt");
    this->declare_parameter("loop", true);

    imu_data_path_ = this->get_parameter("imu_data").as_string();
    loop_ = this->get_parameter("loop").as_bool();

    if (loop_) RCLCPP_INFO(this->get_logger(), "Odtwarzanie dataset w pętli");

    imu_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("imu_data", 10);
}

sensor_msgs::msg::Imu feeder_imu::createImuMsg(const std::vector<double>& imu_data, const rclcpp::Time& timestamp)
{    
    auto msg = sensor_msgs::msg::Imu();

    msg.header.stamp = timestamp;
    msg.header.frame_id = "world";

    if (imu_data.size() >= 11) {
        // format: timestamp qx qy qz qw gx gy gz ax ay az
        msg.orientation.x = imu_data[1];
        msg.orientation.y = imu_data[2];
        msg.orientation.z = imu_data[3];
        msg.orientation.w = imu_data[4];

        msg.angular_velocity.x = imu_data[5];
        msg.angular_velocity.y = imu_data[6];
        msg.angular_velocity.z = imu_data[7];

        msg.linear_acceleration.x = imu_data[8];
        msg.linear_acceleration.y = imu_data[9];
        msg.linear_acceleration.z = imu_data[10];
    }
    
    const std::array<double, 9> covariance_array = {
        -1.0, 0.0, 0.0, 
        0.0, -1.0, 0.0, 
        0.0, 0.0, -1.0
    };
    
    std::copy(covariance_array.begin(), covariance_array.end(), msg.orientation_covariance.begin());
    std::copy(covariance_array.begin(), covariance_array.end(), msg.angular_velocity_covariance.begin());
    std::copy(covariance_array.begin(), covariance_array.end(), msg.linear_acceleration_covariance.begin());

    return msg;
}

void feeder_imu::send_imu_data() {
    std::ifstream imu_file(imu_data_path_);

    if (!imu_file.is_open()) {
        RCLCPP_ERROR(this->get_logger(), "Cannot open file: %s", imu_data_path_.c_str());
        throw std::runtime_error("Failed to open imu data");
    }
    RCLCPP_INFO(this->get_logger(), "Loaded IMU data from: %s", imu_data_path_.c_str());

    std::string line;
    double first_timestamp = -1.0;
    
    rclcpp::Time start_time = this->now();

    while (std::getline(imu_file, line)) {
        if (!rclcpp::ok()) return;
        if (line.empty()) continue;

        std::vector<double> imu_data;
        std::istringstream imu_iss(line);
        double temp_value;
        while (imu_iss >> temp_value) {
            imu_data.push_back(temp_value);
        }
        
        if (imu_data.empty()) continue;

        double timestamp_sec = imu_data[0];

        if (first_timestamp < 0) {
            first_timestamp = timestamp_sec;
            start_time = this->now();
        }

        double time_offset = timestamp_sec - first_timestamp;
        rclcpp::Time ros_timestamp = start_time + rclcpp::Duration::from_seconds(time_offset);
        rclcpp::Duration real_elapsed = this->now() - start_time;
        
        double wait_seconds = time_offset - real_elapsed.seconds();

        if (wait_seconds > 0) {
            rclcpp::sleep_for(std::chrono::nanoseconds(static_cast<int64_t>(wait_seconds * 1e9))); // POPRAWKA
        }

        imu_publisher_->publish(createImuMsg(imu_data, ros_timestamp));
    }

    imu_file.close();
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    try {
        auto feeder = std::make_shared<feeder_imu>();
        if (feeder->loop_) {
            while (rclcpp::ok()) feeder->send_imu_data();
        } else {
            feeder->send_imu_data();
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("feeder imu"), "Error: %s", e.what());
        rclcpp::shutdown();
        return 1;
    }
    
    rclcpp::shutdown();
    return 0;
}