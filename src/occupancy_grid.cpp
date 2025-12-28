#include <rclcpp/rclcpp.hpp>
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <vector>
#include <cmath>

class occupancy_grid : public rclcpp::Node
{
public:
    occupancy_grid();

private:
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_grid_publisher;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_subscriber;

    float z_min;
    float z_max;
    int hit_thresh;
    float res;
    float map_size;  // rozmiar mapy (25x25m)
    
    float boat_x;
    float boat_y;
    bool pose_received;

    void point_cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
};

occupancy_grid::occupancy_grid() : Node("slam_to_navmap"), 
    boat_x(0.0), boat_y(0.0), pose_received(false) 
{
    occupancy_grid_publisher = this->create_publisher<nav_msgs::msg::OccupancyGrid>("map", 100);
    
    point_cloud = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "point_cloud", 10, std::bind(&occupancy_grid::point_cloud_callback, this, std::placeholders::_1));

    pose_subscriber = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "vo_pose", 10, std::bind(&occupancy_grid::pose_callback, this, std::placeholders::_1));

    this->declare_parameter("z_min", 0.0);
    this->declare_parameter("z_max", 3.0);
    this->declare_parameter("hit_thresh", 1);
    this->declare_parameter("res", 1.0);
    this->declare_parameter("map_size", 100.0);

    z_min = this->get_parameter("z_min").as_double();
    z_max = this->get_parameter("z_max").as_double();
    hit_thresh = this->get_parameter("hit_thresh").as_int();
    res = this->get_parameter("res").as_double();
    map_size = this->get_parameter("map_size").as_double();

    RCLCPP_INFO(this->get_logger(), "Mapa: %.0fx%.0fm, rozdzielczość: %.2fm", map_size, map_size, res);
}

void occupancy_grid::pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    boat_x = msg->pose.position.x;
    boat_y = msg->pose.position.y;
    
    if (!pose_received) {
        pose_received = true;
        RCLCPP_INFO(this->get_logger(), "Odebrano pozycję łodzi: (%.2f, %.2f)", boat_x, boat_y);
    }
}

void occupancy_grid::point_cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    if (!pose_received) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                             "Czekam na pozycję łodzi...");
        return;
    }

    // Stała mapa wokół łodzi
    float half_size = map_size / 2.0;
    float map_min_x = boat_x - half_size;
    float map_max_x = boat_x + half_size;
    float map_min_y = boat_y - half_size;
    float map_max_y = boat_y + half_size;

    int width = static_cast<int>(map_size / res);
    int height = static_cast<int>(map_size / res);
    
    std::vector<int> hit_count(width * height, 0);

    // Iteracja po wszystkich punktach
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");

    int points_added = 0;
    int points_total = 0;
    int points_in_z_range = 0;
    int points_zero = 0;
    float min_x = 1e9, max_x = -1e9, min_y = 1e9, max_y = -1e9;
    
    std::vector<std::tuple<float, float, float>> sample_points;
    
    while (iter_x != iter_x.end()) {
        float x = *iter_x - boat_x;
        float y = *iter_y - boat_y;
        float z = *iter_z;
        
        points_total++;
        
        // Zlicz punkty (0,0,0)
        if (x == 0.0f && y == 0.0f && z == 0.0f) {
            points_zero++;
        }
        
        // Zbierz przykładowe punkty (nie-zerowe)
        if (sample_points.size() < 10 && !(x == 0.0f && y == 0.0f && z == 0.0f)) {
            sample_points.push_back({x, y, z});
        }
        
        // Tracking min/max dla punktów w z_range (pomijając 0,0,0)
        if (z >= z_min && z <= z_max && !(x == 0.0f && y == 0.0f && z == 0.0f)) {
            points_in_z_range++;
            min_x = std::min(min_x, x);
            max_x = std::max(max_x, x);
            min_y = std::min(min_y, y);
            max_y = std::max(max_y, y);
        }

        // Filtruj po z (pomijaj 0,0,0)
        if (z >= z_min && z <= z_max && !(x == 0.0f && y == 0.0f && z == 0.0f)) {
            // Sprawdź czy punkt jest w mapie
            if (x >= map_min_x && x <= map_max_x && y >= map_min_y && y <= map_max_y) {
                // Oblicz indeks w siatce
                int grid_x = static_cast<int>((x - map_min_x) / res);
                int grid_y = static_cast<int>((y - map_min_y) / res);
                
                if (grid_x >= 0 && grid_x < width && grid_y >= 0 && grid_y < height) {
                    int index = grid_y * width + grid_x;
                    hit_count[index]++;
                    points_added++;
                }
            }
        }

        ++iter_x;
        ++iter_y;
        ++iter_z;
    }

    // Utwórz mapę okupacji
    std::vector<int8_t> occupancy_map;
    int occupied_cells = 0;
    
    for (int count : hit_count) {
        if (count >= hit_thresh) {
            occupancy_map.push_back(100);
            occupied_cells++;
        } else {
            occupancy_map.push_back(0);
        }
    }

    RCLCPP_INFO(this->get_logger(), "=== DEBUG INFO ===");
    RCLCPP_INFO(this->get_logger(), "Łódź: (%.2f, %.2f)", boat_x, boat_y);
    RCLCPP_INFO(this->get_logger(), "Mapa: X[%.2f, %.2f] Y[%.2f, %.2f]", 
                map_min_x, map_max_x, map_min_y, map_max_y);
    RCLCPP_INFO(this->get_logger(), "Punkty: total=%d, zero=%d, z_ok=%d",
                points_total, points_zero, points_in_z_range);
    
    if (points_in_z_range > 0) {
        RCLCPP_INFO(this->get_logger(), "Zakres punktów: X[%.2f, %.2f] Y[%.2f, %.2f]",
                    min_x, max_x, min_y, max_y);
    }
    
    RCLCPP_INFO(this->get_logger(), "Przykładowe punkty (nie-zerowe):");
    for (size_t i = 0; i < sample_points.size(); i++) {
        auto [sx, sy, sz] = sample_points[i];
        RCLCPP_INFO(this->get_logger(), "  [%zu] x=%.2f, y=%.2f, z=%.2f", i, sx, sy, sz);
    }
    
    RCLCPP_INFO(this->get_logger(), "Wynik: punktów w mapie=%d, zajętych komórek=%d/%d", 
                points_added, occupied_cells, width * height);

    // Wyślij mapę
    auto grid_msg = nav_msgs::msg::OccupancyGrid();
    grid_msg.header.stamp = msg->header.stamp;
    grid_msg.header.frame_id = msg->header.frame_id;
    
    grid_msg.info.resolution = res;
    grid_msg.info.width = width;
    grid_msg.info.height = height;
    
    grid_msg.info.origin.position.x = map_min_x;
    grid_msg.info.origin.position.y = map_min_y;
    grid_msg.info.origin.position.z = 0;
    
    grid_msg.info.origin.orientation.w = 1;
    grid_msg.info.origin.orientation.x = 0;
    grid_msg.info.origin.orientation.y = 0;
    grid_msg.info.origin.orientation.z = 0;
    
    grid_msg.data = occupancy_map;
    
    occupancy_grid_publisher->publish(grid_msg);
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<occupancy_grid>());
    rclcpp::shutdown();
    return 0;
}