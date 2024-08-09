#ifndef TURTLEBOT3_NAVIGATION_LAP_TIME_ACTION_CLIENT_HPP_
#define TURTLEBOT3_NAVIGATION_LAP_TIME_ACTION_CLIENT_HPP_

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include <navigation_interfaces/action/measure_lap_time.hpp>

class LapTimeActionClient : public rclcpp::Node {
public:
    LapTimeActionClient();

private:
    void send_goal();
    void goal_response_callback(std::shared_future<rclcpp_action::ClientGoalHandle<navigation_interfaces::action::MeasureLapTime>::SharedPtr> future);
    void feedback_callback(rclcpp_action::ClientGoalHandle<navigation_interfaces::action::MeasureLapTime>::SharedPtr, const std::shared_ptr<const navigation_interfaces::action::MeasureLapTime::Feedback> feedback);
    void result_callback(const rclcpp_action::ClientGoalHandle<navigation_interfaces::action::MeasureLapTime>::WrappedResult &result);

    rclcpp_action::Client<navigation_interfaces::action::MeasureLapTime>::SharedPtr client_;
};

#endif  // TURTLEBOT3_NAVIGATION_LAP_TIME_ACTION_CLIENT_HPP_
