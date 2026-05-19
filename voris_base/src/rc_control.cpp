#include "rc_control.hpp"

RCControl::RCControl() : Node("rc_control")
{
    // Initialize publishers
    rc_pub_ = this->create_publisher<mavros_msgs::msg::OverrideRCIn>("mavros/rc/override", 10);

    // Initialize services
    mavros_arm_client_ = std::make_shared<nav2_util::ServiceClient<mavros_msgs::srv::CommandBool>>(this, "mavros/cmd/arming");
    mavros_set_mode_client_ = std::make_shared<nav2_util::ServiceClient<mavros_msgs::srv::SetMode>>(this, "mavros/set_mode");

    // Initialize timer to publish RC commands at a fixed rate
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),  // Adjust the rate as needed
        [this]() {
            follow_wp();
        }
    );

    gain_ = 0.5;

    wp = {0.0, 0.0, -1.0}; // Example waypoint, replace with actual logic to set waypoints
}

bool RCControl::set_arm(bool arm)
{
    auto request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
    request->value = arm;
    // call mavros/cmd/arming
    auto response = std::make_shared<mavros_msgs::srv::CommandBool::Response>();
    RCLCPP_INFO(get_logger(), arm ? "arming..." : "disarming...");
    auto result = mavros_arm_client_->invoke(request, response);
    result = result && response->success;
    RCLCPP_INFO(get_logger(), result ? "success" : "failure");
    return result;
}

bool RCControl::set_mode(const std::string & mode)
{
    auto request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
    request->custom_mode = mode;
    // change mode of the ardusub calling /mavros/set_mode
    auto response = std::make_shared<mavros_msgs::srv::SetMode::Response>();
    RCLCPP_INFO(get_logger(), "setting mode to %s...", mode.c_str());
    auto result = mavros_set_mode_client_->invoke(request, response);
    result = result && response->mode_sent;
    RCLCPP_INFO(get_logger(), result ? "success" : "failure");
    return result;
}

void RCControl::publish_rc(mavros_msgs::msg::OverrideRCIn & msg)
{
    for (uint16_t & channel : msg.channels) {
        channel = mavros_msgs::msg::OverrideRCIn::CHAN_NOCHANGE;
      }

    msg.channels[5 - 1] = map_pwm(msg, 0.05);
    msg.channels[6 - 1] = map_pwm(msg, 0.05);
    msg.channels[4 - 1] = map_pwm(msg, 0.05);

    rc_pub_->publish(msg);
}

void RCControl::follow_wp()
{
    
}

uint16_t RCControl::map_pwm(float value, float threshold)
{
    if (std::abs(value) < threshold) {
        return static_cast<uint16_t>(std::clamp(1500, 1100, 1900));
    }
    int pwm = 1500 + (value * 400.0 * gain_);
    return static_cast<uint16_t>(std::clamp(pwm, 1100, 1900));
}