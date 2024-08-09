#ifndef TURTLEBOT3_NAVIGATION_ROBOT_DRIVER_HPP_
#define TURTLEBOT3_NAVIGATION_ROBOT_DRIVER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <navigation_interfaces/srv/find_closest_wall.hpp>

class RobotDriver : public rclcpp::Node {
public:
    RobotDriver();

private:
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Client<navigation_interfaces::srv::FindClosestWall>::SharedPtr client_;
};

#endif  // TURTLEBOT3_NAVIGATION_ROBOT_DRIVER_HPP_
