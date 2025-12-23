 
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <tf2_ros/transform_broadcaster.hpp>

class ROSInterface : public rclcpp::Node
{
public:
    ROSInterface();
    ~ROSInterface();

    void publishLaserScan(const std::vector<std::tuple<int, int, int>> &hits, double heading);
    void updateMap(double x, double y, const std::vector<std::tuple<int, int, int>> &hits);
    void broadcastTF(double x, double y, double yaw);

    std::vector<int8_t> map;

private:
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;

    const double map_resolution = 0.05; // meters per cell
    const int map_width = 400;          // number of cells
    const int map_height = 400;         // number of cells

    const double map_origin_x = -10.0; // meters
    const double map_origin_y = -10.0; // meters
};
