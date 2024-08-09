#include "turtlebot3_navigation_cpp/wall_finder_service.hpp"

WallFinderService::WallFinderService() : Node("wall_finder_service") {
    service_ = this->create_service<navigation_interfaces::srv::FindClosestWall>(
        "find_closest_wall", std::bind(&WallFinderService::findClosestWall, this, std::placeholders::_1, std::placeholders::_2));
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
}

void WallFinderService::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    // Store the latest laser scan data
    last_scan_ = msg;
}

void WallFinderService::findClosestWall(const std::shared_ptr<navigation_interfaces::srv::FindClosestWall::Request> request,
                         std::shared_ptr<navigation_interfaces::srv::FindClosestWall::Response> response) {

    if (!last_scan_) {
        RCLCPP_ERROR(this->get_logger(), "No scan data received yet.");
        return;
    }

     // Find the minimum distance and corresponding angle
    float min_distance = std::numeric_limits<float>::infinity();
    int min_index = -1;

    for (size_t i = 0; i < last_scan_->ranges.size(); ++i) {
        if (last_scan_->ranges[i] < min_distance) {
            min_distance = last_scan_->ranges[i];
            min_index = i;
        }
    }

     // Compute the angle to the closest wall
    float angle_to_closest_wall = last_scan_->angle_min + min_index * last_scan_->angle_increment;

    // Set the response fields
    response->distance = min_distance;
    response->angle = angle_to_closest_wall;

     RCLCPP_INFO(this->get_logger(), "Closest wall found at distance: %f meters, angle: %f radians", min_distance, angle_to_closest_wall);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<WallFinderService>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}