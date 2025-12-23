#include <cmath>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.hpp>

#include "robot_simulator_pkg/ros_interface.hpp"

using namespace std::chrono_literals;

ROSInterface::ROSInterface() : Node("robot_simulator")
{
    scan_pub = this->create_publisher<sensor_msgs::msg::LaserScan>("/scan", 10);
    map_pub = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);
    tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    this->map.resize(this->map_width * this->map_height, -1); // Unknown cells
}

ROSInterface::~ROSInterface()
{
    rclcpp::shutdown();
}

void ROSInterface::publishLaserScan(const std::vector<std::tuple<int, int, int>> &hits, double heading)
{
    auto scan_msg = sensor_msgs::msg::LaserScan();
    scan_msg.header.stamp = this->now();
    scan_msg.header.frame_id = "base_link";
    scan_msg.angle_min = -M_PI / 4.5;  // -40 degrees in radians
    scan_msg.angle_max = M_PI / 4.5;   // +40 degrees in radians
    scan_msg.angle_increment = (scan_msg.angle_max - scan_msg.angle_min) / 60;
    scan_msg.time_increment = 0.0;
    scan_msg.scan_time = 0.1;
    scan_msg.range_min = 0.02;
    scan_msg.range_max = 3.0;

    scan_msg.ranges.resize(60, scan_msg.range_max);

    for (size_t i = 0; i < hits.size() && i < 60; ++i)
    {
        int distance_pixels = std::get<2>(hits[i]);
        scan_msg.ranges[i] = static_cast<float>(distance_pixels) / 100.0f; // Convert pixels to meters
    }

    scan_pub->publish(scan_msg);
};

void ROSInterface::updateMap(double x, double y, const std::vector<std::tuple<int, int, int>> &hits)
{
    for (const auto &hit : hits)
    {
        double hx = static_cast<double>(std::get<0>(hit)) / 100.0; // Convert mm to meters
        double hy = static_cast<double>(std::get<1>(hit)) / 100.0; // Convert mm to meters

        int mx = static_cast<int>((hx - this->map_origin_x) / this->map_resolution);
        int my = static_cast<int>((hy - this->map_origin_y) / this->map_resolution);

        if (mx >= 0 && mx < this->map_width && my >= 0 && my < this->map_height)
        {
            this->map[my * this->map_width + mx] = 100; // Mark as occupied
        }
    }

    auto grid_msg = nav_msgs::msg::OccupancyGrid();
    grid_msg.header.stamp = this->now();
    grid_msg.header.frame_id = "map";
    grid_msg.info.resolution = this->map_resolution;
    grid_msg.info.width = this->map_width;
    grid_msg.info.height = this->map_height;
    grid_msg.info.origin.position.x = this->map_origin_x;
    grid_msg.info.origin.position.y = this->map_origin_y;
    grid_msg.data = this->map;

    map_pub->publish(grid_msg);
}

void ROSInterface::broadcastTF(double x, double y, double yaw)
{
    geometry_msgs::msg::TransformStamped t;
    t.header.frame_id = "map";
    t.child_frame_id = "base_link";
    t.transform.translation.x = x / 100.0;
    t.transform.translation.y = y / 100.0;
    t.transform.rotation.z = sin(yaw / 2);
    t.transform.rotation.w = cos(yaw / 2);
    tf_broadcaster->sendTransform(t);
}