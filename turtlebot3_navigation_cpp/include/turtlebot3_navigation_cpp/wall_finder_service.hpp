#ifndef TURTLEBOT3_NAVIGATION_WALL_FINDER_SERVICE_HPP_
#define TURTLEBOT3_NAVIGATION_WALL_FINDER_SERVICE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <navigation_interfaces/srv/find_closest_wall.hpp>
#include "sensor_msgs/msg/laser_scan.hpp"

class WallFinderService : public rclcpp::Node {
public:
    WallFinderService();

private:
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void findClosestWall(const std::shared_ptr<navigation_interfaces::srv::FindClosestWall::Request> request,
                         std::shared_ptr<navigation_interfaces::srv::FindClosestWall::Response> response);

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Service<navigation_interfaces::srv::FindClosestWall>::SharedPtr service_;
    sensor_msgs::msg::LaserScan::SharedPtr last_scan_;
};

#endif  // TURTLEBOT3_NAVIGATION_WALL_FINDER_SERVICE_HPP_
