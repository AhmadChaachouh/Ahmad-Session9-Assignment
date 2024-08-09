#ifndef TURTLEBOT3_NAVIGATION_LAP_TIME_ACTION_SERVER_HPP_
#define TURTLEBOT3_NAVIGATION_LAP_TIME_ACTION_SERVER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include <navigation_interfaces/action/measure_lap_time.hpp>

class LapTimeActionServer : public rclcpp::Node {
public:
    LapTimeActionServer();

private:
    rclcpp_action::GoalResponse handleGoal(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const navigation_interfaces::action::MeasureLapTime::Goal> goal);
    rclcpp_action::CancelResponse handleCancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<navigation_interfaces::action::MeasureLapTime>> goal_handle);
    void handleAccepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<navigation_interfaces::action::MeasureLapTime>> goal_handle);
    void execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<navigation_interfaces::action::MeasureLapTime>> goal_handle);

    rclcpp_action::Server<navigation_interfaces::action::MeasureLapTime>::SharedPtr action_server_;
};

#endif  // TURTLEBOT3_NAVIGATION_LAP_TIME_ACTION_SERVER_HPP_
