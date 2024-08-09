#include "turtlebot3_navigation_cpp/robot_driver.hpp"

RobotDriver::RobotDriver() : Node("robot_driver") {
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10, std::bind(&RobotDriver::scanCallback, this, std::placeholders::_1));
    client_ = this->create_client<navigation_interfaces::srv::FindClosestWall>("find_closest_wall");

    // Call the service to find the closest wall
    auto request = std::make_shared<navigation_interfaces::srv::FindClosestWall::Request>();
    while (!client_->wait_for_service(std::chrono::seconds(1))) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Waiting for the service to appear...");
    }
    auto result = client_->async_send_request(request);
}

void RobotDriver::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {

     // Parameters for the wall-following logic
    const float desired_distance_from_wall = 0.5;  // meters
    const float linear_speed = 0.2;  // meters per second
    const float angular_speed = 0.5;  // radians per second

    // Find the minimum distance in the scan
    float min_distance = std::numeric_limits<float>::infinity();
    int min_index = -1;

    for (size_t i = 0; i < msg->ranges.size(); ++i) {
        if (msg->ranges[i] < min_distance) {
            min_distance = msg->ranges[i];
            min_index = i;
        }
    }

    // Compute the control command
    auto cmd_msg = geometry_msgs::msg::Twist();
    
    // If the minimum distance is less than desired, turn left
    if (min_distance < desired_distance_from_wall) {
        cmd_msg.angular.z = angular_speed;
        cmd_msg.linear.x = 0.0;
    } else {
        // Otherwise, move forward
        cmd_msg.linear.x = linear_speed;
        cmd_msg.angular.z = 0.0;
    }

    // Publish the command
    cmd_vel_pub_->publish(cmd_msg);

}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RobotDriver>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
