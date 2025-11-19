#include "rclcpp/rclcpp.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "builtin_interfaces/msg/time.hpp"

class TfStaticPublisher : public rclcpp::Node
{
public:
  TfStaticPublisher() : Node("tf_static_publisher")
  {
    // Ustawienia QoS dla /tf_static (TRANSIENT_LOCAL)
    auto qos_static = rclcpp::QoS(rclcpp::KeepLast(10));
    qos_static.transient_local();
    qos_static.reliable();
    
    publisher_ = this->create_publisher<tf2_msgs::msg::TFMessage>("/tf_static", qos_static);
    
    // Subskrybuj /tf z domyślnym QoS
    auto qos_tf = rclcpp::QoS(rclcpp::KeepLast(100));
    qos_tf.best_effort();
    
    subscriber_ = this->create_subscription<tf2_msgs::msg::TFMessage>(
      "/tf", 
      qos_tf,
      std::bind(&TfStaticPublisher::tf_callback, this, std::placeholders::_1));
    
    RCLCPP_INFO(this->get_logger(), "TF Static Publisher started - listening to /tf");
  }

private:
  void tf_callback(const tf2_msgs::msg::TFMessage::SharedPtr msg)
  {
    if (msg->transforms.empty()) {
      return;
    }
    
    // Pobierz timestamp z pierwszej transformacji w wiadomości /tf
    auto received_stamp = msg->transforms[0].header.stamp;
    
    // Przygotuj wiadomość do publikacji
    tf2_msgs::msg::TFMessage tf_msg;
    geometry_msgs::msg::TransformStamped transform;
    
    // Użyj timestamp z /tf
    transform.header.stamp.sec = received_stamp.sec;
    transform.header.stamp.nanosec = received_stamp.nanosec;
    transform.header.frame_id = "world";
    transform.child_frame_id = "dworld";
    
    // Translacja
    transform.transform.translation.x = 0.0;
    transform.transform.translation.y = 0.0;
    transform.transform.translation.z = 0.0;
    
    // Rotacja (quaternion)
    transform.transform.rotation.x = 0.7067696;
    transform.transform.rotation.y = 0.047118;
    transform.transform.rotation.z = 0;
    transform.transform.rotation.w = 0.7058729;
    
    tf_msg.transforms.push_back(transform);
    
    publisher_->publish(tf_msg);
    
    RCLCPP_DEBUG(this->get_logger(), "Published /tf_static with timestamp: %d.%09d", 
                 received_stamp.sec, received_stamp.nanosec);
  }

  rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr publisher_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr subscriber_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TfStaticPublisher>());
  rclcpp::shutdown();
  return 0;
}