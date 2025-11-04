#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/opencv.hpp>
#include <memory>
#include <fstream>
#include <sstream>
#include <string>

class feeder_png : public rclcpp::Node
{
public:
    feeder_png();
    void send_photo_data();

private:
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_left_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_right_;
    std::string images_folder_;
    std::string timestamp_path_;
    
    sensor_msgs::msg::Image createImageMsg(const cv::Mat& img, const rclcpp::Time& timestamp);
};

feeder_png::feeder_png() : Node("feeder_obrazow_png")
{
    // Deklaracja parametrów
    this->declare_parameter("images_folder", "/ws/png_SLAM_data/left_images");
    this->declare_parameter("timestamp_path", "/ws/png_SLAM_data/timestamp.txt");
    
    // Pobranie wartości parametrów
    images_folder_ = this->get_parameter("images_folder").as_string();
    timestamp_path_ = this->get_parameter("timestamp_path").as_string();
    
    RCLCPP_INFO(this->get_logger(), "Używam ścieżki images_folder: %s", images_folder_.c_str());
    RCLCPP_INFO(this->get_logger(), "Używam ścieżki do pliku timestamp: %s", timestamp_path_.c_str());
    
    // Usunięcie końcowego '/' z folderu danych jeśli istnieje
    if (!images_folder_.empty() && images_folder_.back() == '/') {
        images_folder_.pop_back();
    }
    
    // Tworzenie publisherów
    image_publisher_left_ = this->create_publisher<sensor_msgs::msg::Image>("image_left_raw_data", 10);
    image_publisher_right_ = this->create_publisher<sensor_msgs::msg::Image>("image_right_raw_data", 10);
}

sensor_msgs::msg::Image feeder_png::createImageMsg(const cv::Mat& img, const rclcpp::Time& timestamp)
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

void feeder_png::send_photo_data() {
    std::ifstream time_stamps(timestamp_path_);
    
    if (!time_stamps.is_open()) {
        RCLCPP_ERROR(this->get_logger(), "Cannot open file: %s", timestamp_path_.c_str());
        throw std::runtime_error("Failed to open timestamp file");
    }
    
    RCLCPP_INFO(this->get_logger(), "Loaded timestamps from: %s", timestamp_path_.c_str());
    
    std::string line;
    int frame_count = 0;
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
        
        if (first_timestamp < 0) {
            first_timestamp = timestamp_sec;
        }
        
        double time_offset = timestamp_sec - first_timestamp;
        rclcpp::Time ros_timestamp = start_time + rclcpp::Duration::from_seconds(time_offset);
        
        std::string left_photo_name = images_folder_ + "/" + photo_name + ".png";
        
        cv::Mat img_left = cv::imread(left_photo_name, cv::IMREAD_COLOR);
        
        if (img_left.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Nie można wczytać obrazu: %s", left_photo_name.c_str());
            continue;
        }
        
        cv::Mat img_resized;
        cv::resize(img_left, img_resized, cv::Size(752, 480), 0, 0, cv::INTER_LINEAR);
        
        auto msg_left = createImageMsg(img_resized, ros_timestamp);
        msg_left.header.frame_id = "cam0";
        
        image_publisher_left_->publish(msg_left);
        
        frame_count++;
        
        if (frame_count % 10 == 0) {
            RCLCPP_INFO(this->get_logger(), "Opublikowano %d frame'ów, subskrybenci: %zu", 
                        frame_count, image_publisher_left_->get_subscription_count());
        }
        
        if (frame_count > 1) {
            auto next_line_pos = time_stamps.tellg();
            std::string next_line;
            if (std::getline(time_stamps, next_line)) {
                std::istringstream next_iss(next_line);
                double next_timestamp;
                std::string next_photo;
                if (next_iss >> next_timestamp >> next_photo) {
                    double wait_time = (next_timestamp - timestamp_sec) * 1000.0; // oblicza ile ma poczekać do następnego timestamp
                    if (wait_time > 0 && wait_time < 1000) { // ignoruje jesli przerwa jest wieksza niz sekunda
                        rclcpp::sleep_for(std::chrono::milliseconds(static_cast<int>(wait_time))); // czeka
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
    // WAŻNE: argc i argv są przekazywane do rclcpp::init()
    // ROS2 automatycznie parsuje argumenty w formacie --ros-args -p
    rclcpp::init(argc, argv);
    
    try {
        auto feeder = std::make_shared<feeder_png>();
        feeder->send_photo_data();
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("feeder"), "Error: %s", e.what());
        rclcpp::shutdown();
        return 1;
    }
    
    rclcpp::shutdown();
    return 0;
}