#include <rclcpp/rclcpp.hpp>
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <vector>

class occupancy_grid : public rclcpp::Node
{
public:
    occupancy_grid();
    float z_min;
    float z_max;
    int hit_thresh;
    float res;
    float msg_min_x;
    float msg_max_x;
    float msg_min_y;
    float msg_max_y;
    int msg_width;
    int msg_height;

private:
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_grid_publisher;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud;

    void send_msg(const rclcpp::Time& timestamp, const std::vector<int8_t>& pointer_msg_data);
    void point_cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
};

occupancy_grid::occupancy_grid() : Node("slam_to_navmap") {
    // stworz publisher na topic /map
    occupancy_grid_publisher = this->create_publisher<nav_msgs::msg::OccupancyGrid>("map", 100);

    // subrciption dla point cloud
    point_cloud = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "point_cloud", 10, std::bind(&occupancy_grid::point_cloud_callback, this, std::placeholders::_1));

    // deklaracja potrzebnych parametrow wraz z wartosciami domyslnymi
    this->declare_parameter("z_min", 0.0);
    this->declare_parameter("z_max", 3.0);
    this->declare_parameter("hit_thresh", 1);
    this->declare_parameter("res", 0.5);

    z_min = this->get_parameter("z_min").as_double();
    z_max = this->get_parameter("z_max").as_double();
    hit_thresh = this->get_parameter("hit_thresh").as_int();
    res = this->get_parameter("res").as_double();

    RCLCPP_INFO(this->get_logger(), "Parametr z_min %f", z_min);
    RCLCPP_INFO(this->get_logger(), "Parametr z_max %f", z_max);
    RCLCPP_INFO(this->get_logger(), "Parametr hit_thresh %i", hit_thresh);
}

void occupancy_grid::point_cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    std::vector<std::pair<float, float>> filtered_points;
    // reset dla kazdej wiadomosci
    msg_min_x = 0.0;
    msg_max_x = 0.0;
    msg_min_y = 0.0;
    msg_max_y = 0.0;

    bool flag_first = true;

    // potrzebne nam bedzie minimaly i maksymalny x i y, zeby okreslic jak szeroka i jak wysoka ma byc mapa przeszkod dla danego framea
    // pobranie danych z point cloud
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");

    while (iter_x != iter_x.end()) {
        float z = 0.0*(*iter_x) + 0.9902681*(*iter_y) + 0.1391731*(*iter_z);
        if (z >= z_min && z <= z_max) { // filtracja po z
            // odczyt x, y 
            float x = (*iter_x);
            float y = 0.0*(*iter_x) + 0.1391731*(*iter_y) - 0.9902681*(*iter_z);
            // sprawdzenie min i max x,y
            if (flag_first) {
                msg_min_x = msg_max_x = x;
                msg_min_y = msg_max_y = y;
                flag_first = false;
            } else {
                msg_min_x = std::min(msg_min_x, x);
                msg_max_x = std::max(msg_max_x, x);
                msg_min_y = std::min(msg_min_y, y);
                msg_max_y = std::max(msg_max_y, y);
            }
            // zapis do wektora
            filtered_points.push_back({x, y});
        }

        ++iter_x;
        ++iter_y;
        ++iter_z;
    }
    if (filtered_points.empty()) return;

    msg_width = static_cast<int>((msg_max_x - msg_min_x)/res +1);
    msg_height = static_cast<int>((msg_max_y - msg_min_y)/res +1);

    std::vector<int> hit_count(msg_width * msg_height, 0); // wektor zliczania trafien do kadej komórki

    for (const auto& point : filtered_points) {
        int grid_x = static_cast<int>((point.first - msg_min_x) / res);
        int grid_y = static_cast<int>((point.second - msg_min_y) / res);
        const int index = grid_y * msg_width + grid_x;
        hit_count[index]++;
    }
    
    std::vector<int8_t> occupancy_map; // 100 jesli jest przeszkoda, 0 jesli nie ma przeszkody

    for (const int& hit_point : hit_count) {
        if (hit_point >= hit_thresh) occupancy_map.push_back(100);
        else occupancy_map.push_back(0);
    }

    send_msg(msg->header.stamp, occupancy_map);
}

void occupancy_grid::send_msg(const rclcpp::Time& timestamp, const std::vector<int8_t>& pointer_msg_data) { // wskaznik do poczatku tablicy
    auto msg = nav_msgs::msg::OccupancyGrid();

    msg.header.stamp = timestamp;
    msg.header.frame_id = "dworld";

    msg.info.resolution = res;
    msg.info.width = msg_width;
    msg.info.height = msg_height;

    // chcemy, zeby statek byl na samym srodku, occupancy grid idzie tylko na jedna cwiartke ukladu, wiec trzebva przesunac o polowe ile heigth i width
    msg.info.origin.position.x = msg_min_x;
    msg.info.origin.position.y = msg_min_y;
    msg.info.origin.position.z = 0; // hardkode 0
    
    // orientacja taka jaka jest statek
    // na razie hardkod kamera rowna z plaszczyzna
    msg.info.origin.orientation.x = 0;
    msg.info.origin.orientation.y = 0;
    msg.info.origin.orientation.z = 0;
    msg.info.origin.orientation.w = 1;

    msg.data = pointer_msg_data;

    occupancy_grid_publisher->publish(msg);
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<occupancy_grid>());
    rclcpp::shutdown();
    return 0;
}