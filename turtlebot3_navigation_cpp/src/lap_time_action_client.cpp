#include "turtlebot3_navigation_cpp/lap_time_action_client.hpp"

LapTimeActionClient::LapTimeActionClient() : Node("lap_time_action_client") {
    client_ = rclcpp_action::create_client<navigation_interfaces::action::MeasureLapTime>(this, "measure_lap_time");
    send_goal();
}

void LapTimeActionClient::send_goal() {
    auto goal_msg = navigation_interfaces::action::MeasureLapTime::Goal();
    // goal_msg.start = true;

    auto send_goal_options = rclcpp_action::Client<navigation_interfaces::action::MeasureLapTime>::SendGoalOptions();
    // send_goal_options.goal_response_callback = std::bind(&LapTimeActionClient::goal_response_callback, this, std::placeholders::_1);
    send_goal_options.feedback_callback = std::bind(&LapTimeActionClient::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
    send_goal_options.result_callback = std::bind(&LapTimeActionClient::result_callback, this, std::placeholders::_1);

    this->client_->async_send_goal(goal_msg, send_goal_options);
}

void LapTimeActionClient::goal_response_callback(std::shared_future<rclcpp_action::ClientGoalHandle<navigation_interfaces::action::MeasureLapTime>::SharedPtr> future) {
    auto goal_handle = future.get();
    if (!goal_handle) {
        RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    } else {
        RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
    }
}

void LapTimeActionClient::feedback_callback(rclcpp_action::ClientGoalHandle<navigation_interfaces::action::MeasureLapTime>::SharedPtr, const std::shared_ptr<const navigation_interfaces::action::MeasureLapTime::Feedback> feedback) {
    RCLCPP_INFO(this->get_logger(), "Received feedback: %f", feedback->current_time);
}

void LapTimeActionClient::result_callback(const rclcpp_action::ClientGoalHandle<navigation_interfaces::action::MeasureLapTime>::WrappedResult &result) {
    switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(this->get_logger(), "Result received: %f", result.result->total_time);
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
            break;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
            break;
        default:
            RCLCPP_ERROR(this->get_logger(), "Unknown result code");
            break;
    }
    rclcpp::shutdown();
}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LapTimeActionClient>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}