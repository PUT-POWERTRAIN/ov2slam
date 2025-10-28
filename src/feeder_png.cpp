#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/opencv.hpp>
#include <memory>
#include <fstream>
#include <sstream>

class FEEDER_PNG : public rclcpp::Node
{
public:
    FEEDER_PNG();
    void test();
    
private:
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_left_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_right_;
    
    sensor_msgs::msg::Image createImageMsg(const cv::Mat& img, const rclcpp::Time& timestamp);
};

FEEDER_PNG::FEEDER_PNG() : Node("feeder_obrazow_png")
{
    image_publisher_left_ = this->create_publisher<sensor_msgs::msg::Image>("image_left_raw_data", 10);
    image_publisher_right_ = this->create_publisher<sensor_msgs::msg::Image>("image_right_raw_data", 10);
}

sensor_msgs::msg::Image FEEDER_PNG::createImageMsg(const cv::Mat& img, const rclcpp::Time& timestamp)
{
    auto msg = sensor_msgs::msg::Image();
    msg.header.stamp = timestamp;
    msg.header.frame_id = "camera";
    msg.height = img.rows;
    msg.width = img.cols;
    msg.encoding = "bgr8";
    msg.is_bigendian = false;
    msg.step = img.cols * img.elemSize();
    
    size_t data_size = img.total() * img.elemSize();
    msg.data.resize(data_size);
    
    if (img.isContinuous()) {
        std::memcpy(msg.data.data(), img.data, data_size);
    } else {
        size_t idx = 0;
        for(int i = 0; i < img.rows; i++) {
            std::memcpy(msg.data.data() + idx, img.ptr(i), img.cols * img.elemSize());
            idx += img.cols * img.elemSize();
        }
    }
    
    return msg;
}

void FEEDER_PNG::test() {
    std::ifstream time_stamps("/ws/png_SLAM_data/timestamp.txt");
    
    if (!time_stamps.is_open()) {
        RCLCPP_ERROR(this->get_logger(), "Nie można otworzyć timestamp.txt");
        return;
    }
    
    RCLCPP_INFO(this->get_logger(), "Czekam 2 sekundy na uruchomienie subskrybentów...");
    rclcpp::sleep_for(std::chrono::seconds(2));
    
    std::string line;
    int frame_count = 0;
    
    // Zapisz pierwszy timestamp do obliczenia różnic
    double first_timestamp = -1.0;
    rclcpp::Time start_time = this->now();
    
    while (std::getline(time_stamps, line)) {
        if (line.empty()) continue;
        
        std::istringstream iss(line);
        double timestamp_sec;
        std::string photo_name;
        
        if (!(iss >> timestamp_sec >> photo_name)) {
            RCLCPP_WARN(this->get_logger(), "Błąd parsowania linii: %s", line.c_str());
            continue;
        }
        
        // Ustaw pierwszy timestamp jako bazowy
        if (first_timestamp < 0) {
            first_timestamp = timestamp_sec;
        }
        
        // Oblicz różnicę czasu od pierwszego frame'a
        double time_offset = timestamp_sec - first_timestamp;
        
        // Użyj aktualnego czasu ROS + offset
        rclcpp::Time ros_timestamp = start_time + rclcpp::Duration::from_seconds(time_offset);
        
        std::string left_photo_name = "/ws/png_SLAM_data/left_images/" + photo_name + ".png";
        
        cv::Mat img_left = cv::imread(left_photo_name, cv::IMREAD_COLOR);
        
        if (img_left.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Nie można wczytać obrazu: %s", photo_name.c_str());
            continue;
        }
        
        // Przeskalowanie do EuRoC (752x480)
        cv::Mat img_resized;
        cv::resize(img_left, img_resized, cv::Size(752, 480), 0, 0, cv::INTER_LINEAR);
        
        auto msg_left = createImageMsg(img_resized, ros_timestamp);
        msg_left.header.frame_id = "cam0";  // WAŻNE: zmień na cam0
        
        image_publisher_left_->publish(msg_left);
        
        frame_count++;
        
        if (frame_count % 10 == 0) {
            RCLCPP_INFO(this->get_logger(), "Opublikowano %d frame'ów, subskrybenci: %zu", 
                        frame_count, image_publisher_left_->get_subscription_count());
        }
        
        // Zachowaj oryginalne odstępy czasowe między frame'ami
        if (frame_count > 1) {
            // Następny timestamp - obecny timestamp = czas oczekiwania
            auto next_line_pos = time_stamps.tellg();
            std::string next_line;
            if (std::getline(time_stamps, next_line)) {
                std::istringstream next_iss(next_line);
                double next_timestamp;
                std::string next_photo;
                if (next_iss >> next_timestamp >> next_photo) {
                    double wait_time = (next_timestamp - timestamp_sec) * 1000.0; // ms
                    if (wait_time > 0 && wait_time < 1000) {
                        rclcpp::sleep_for(std::chrono::milliseconds(static_cast<int>(wait_time)));
                    }
                }
                time_stamps.seekg(next_line_pos);
            }
        } else {
            rclcpp::sleep_for(std::chrono::milliseconds(100));
        }
    }
    
    time_stamps.close();
    RCLCPP_INFO(this->get_logger(), "DONE! Opublikowano %d obrazów", frame_count);
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto feeder_png = std::make_shared<FEEDER_PNG>();
    feeder_png->test();
    rclcpp::shutdown();
    return 0;
}