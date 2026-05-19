#ifndef RC_CONTROL_HPP
#define RC_CONTROL_HPP

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include "mavros_msgs/msg/override_rc_in.hpp"
#include "mavros_msgs/srv/command_bool.hpp"
#include "mavros_msgs/srv/set_mode.hpp"
#include "nav2_util/service_client.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

// construtor da classe
class RCControl : public rclcpp::Node
{
public:
    RCControl();

private:
    // Subscribers 
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr odometry_sub_;

    // Publishers
    rclcpp::Publisher<mavros_msgs::msg::OverrideRCIn>::SharedPtr rc_pub_;

    // Services
    std::shared_ptr<nav2_util::ServiceClient<mavros_msgs::srv::CommandBool>> mavros_arm_client_;
    std::shared_ptr<nav2_util::ServiceClient<mavros_msgs::srv::SetMode>> mavros_set_mode_client_;   

    // timer
    rclcpp::TimerBase::SharedPtr timer_;
}