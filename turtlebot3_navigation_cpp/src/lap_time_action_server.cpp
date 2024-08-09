#include "turtlebot3_navigation_cpp/lap_time_action_server.hpp"

LapTimeActionServer::LapTimeActionServer() : Node("lap_time_action_server") {
    action_server_ = rclcpp_action::create_server<navigation_interfaces::action::MeasureLapTime>(
        this, "measure_lap_time",
        std::bind(&LapTimeActionServer::handleGoal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&LapTimeActionServer::handleCancel, this, std::placeholders::_1),
        std::bind(&LapTimeActionServer::handleAccepted, this, std::placeholders::_1));
}

rclcpp_action::GoalResponse LapTimeActionServer::handleGoal(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const navigation_interfaces::action::MeasureLapTime::Goal> goal) {
    RCLCPP_INFO(this->get_logger(), "Received goal request");
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse LapTimeActionServer::handleCancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<navigation_interfaces::action::MeasureLapTime>> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
}

void LapTimeActionServer::handleAccepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<navigation_interfaces::action::MeasureLapTime>> goal_handle) {
    std::thread{std::bind(&LapTimeActionServer::execute, this, std::placeholders::_1), goal_handle}.detach();
}

void LapTimeActionServer::execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<navigation_interfaces::action::MeasureLapTime>> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    auto result = std::make_shared<navigation_interfaces::action::MeasureLapTime::Result>();
    result->total_time = 0.0;
    goal_handle->succeed(result);
}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LapTimeActionServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}