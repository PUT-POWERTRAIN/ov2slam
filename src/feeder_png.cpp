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
    bool loop_;
    
private:
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_left_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_right_;
    std::string images_folder_left_;
    std::string images_folder_right_;
    std::string timestamp_path_;
    bool enable_stereo_;

    cv::Mat buffer_left_;      // STAŁY bufor unikający realokacji
    cv::Mat buffer_right_;     // STAŁY bufor unikający realokacji

    sensor_msgs::msg::Image image_msg_left_;
    sensor_msgs::msg::Image image_msg_right_;

    sensor_msgs::msg::Image createImageMsg(const cv::Mat& img, const rclcpp::Time& timestamp);
};

feeder_png::feeder_png() : Node("feeder_obrazow_png")
{
    this->declare_parameter("images_folder_left", "/datasets/left_images");
    this->declare_parameter("images_folder_right", "/datasets/right_images");
    this->declare_parameter("enable_stereo", true);
    this->declare_parameter("timestamp_path", "/datasets/timestamp.txt");
    this->declare_parameter("loop", true);
    
    images_folder_left_ = this->get_parameter("images_folder_left").as_string();
    images_folder_right_ = this->get_parameter("images_folder_right").as_string();
    enable_stereo_ = this->get_parameter("enable_stereo").as_bool();
    timestamp_path_ = this->get_parameter("timestamp_path").as_string();
    loop_ = this->get_parameter("loop").as_bool();

    image_publisher_left_ = this->create_publisher<sensor_msgs::msg::Image>("image_left_raw_data", 10);
    if (enable_stereo_)
        image_publisher_right_ = this->create_publisher<sensor_msgs::msg::Image>("image_right_raw_data", 10);

    // Pre-allocate sensor_msgs buffers
    image_msg_left_.encoding = "bgr8";
    image_msg_left_.is_bigendian = false;

    if (enable_stereo_) {
        image_msg_right_.encoding = "bgr8";
        image_msg_right_.is_bigendian = false;
    }
}

sensor_msgs::msg::Image feeder_png::createImageMsg(const cv::Mat& img, const rclcpp::Time& timestamp)
{
    sensor_msgs::msg::Image msg;

    msg.header.stamp = timestamp;
    msg.header.frame_id = "camera";
    msg.height = img.rows;
    msg.width = img.cols;
    msg.encoding = "bgr8";
    msg.is_bigendian = false;
    msg.step = img.cols * img.elemSize();
    msg.data.assign(img.datastart, img.dataend);   // szybkie kopiowanie

    return msg;
}

void feeder_png::send_photo_data() {

    std::ifstream time_stamps(timestamp_path_);
    if (!time_stamps.is_open()){
        RCLCPP_ERROR(this->get_logger(),"Cannot open file: %s",timestamp_path_.c_str());
        return;
    }

    std::string line;
    int frame_count = 0;

    double first_timestamp = -1.0;
    rclcpp::Time start_time = this->now();

    while(std::getline(time_stamps,line))
    {
        if (!rclcpp::ok()) return;
        if (line.empty()) continue;

        double timestamp_sec;
        std::string photo_name;

        {
            std::istringstream iss(line);
            iss >> timestamp_sec >> photo_name;
        }

        if (first_timestamp < 0)
            first_timestamp = timestamp_sec;

        // Docelowy ROS time
        double offset = timestamp_sec - first_timestamp;
        rclcpp::Time target_time = start_time + rclcpp::Duration::from_seconds(offset);

        // Synchronizacja czasowa
        rclcpp::Time now = this->now();
        if (target_time > now)
            rclcpp::sleep_for(std::chrono::nanoseconds((target_time - now).nanoseconds()));

        // Wczytaj obraz (ZWOLNIONE resize)
        std::string left_path = images_folder_left_ + "/" + photo_name + ".png";

        buffer_left_ = cv::imread(left_path, cv::IMREAD_COLOR);
        if (buffer_left_.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Cannot load image %s", left_path.c_str());
            continue;
        }

        auto msg_left = createImageMsg(buffer_left_, target_time);
        msg_left.header.frame_id = "cam0";

        if (enable_stereo_) {
            std::string right_path = images_folder_right_ + "/" + photo_name + ".png";
            buffer_right_ = cv::imread(right_path, cv::IMREAD_COLOR);

            if (buffer_right_.empty()) {
                RCLCPP_ERROR(this->get_logger(), "Cannot load stereo image %s", right_path.c_str());
                continue;
            }

            auto msg_right = createImageMsg(buffer_right_, target_time);
            msg_right.header.frame_id = "cam1";

            image_publisher_left_->publish(msg_left);
            image_publisher_right_->publish(msg_right);
        }
        else {
            image_publisher_left_->publish(msg_left);
        }

        frame_count++;

        if (frame_count % 10 == 0) {
            RCLCPP_INFO(this->get_logger(), "Published %d frames", frame_count);
        }
    }

    RCLCPP_INFO(this->get_logger(), "DONE! Published %d frames", frame_count);
}

int main(int argc, char** argv)
{
    rclcpp::init(argc,argv);
    auto feeder = std::make_shared<feeder_png>();

    if (feeder->loop_) {
        while (rclcpp::ok()) feeder->send_photo_data();
    } else {
        feeder->send_photo_data();
    }

    rclcpp::shutdown();
    return 0;
}
