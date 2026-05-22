#ifndef RC_CONTROL_HPP
#define RC_CONTROL_HPP

#include <memory>
#include <string>
#include <vector>
#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include "mavros_msgs/msg/override_rc_in.hpp"
#include "mavros_msgs/srv/command_bool.hpp"
#include "mavros_msgs/srv/set_mode.hpp"
#include "mavros_msgs/msg/state.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/utils.h"

// construtor da classe
class RCControl : public rclcpp::Node
{
public:
    RCControl();

private:
    bool connected_{};
    bool armed_{};
    //string está dentro de std
    std::string mode_ = "";

    // Subscribers 
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    void odom_cb(const nav_msgs::msg::Odometry::ConstSharedPtr & msg);

    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
    void state_cb(const mavros_msgs::msg::State::ConstSharedPtr & msg);

    // Current robot state
    geometry_msgs::msg::PoseStamped current_pose_;
    geometry_msgs::msg::Twist current_vel_;

    // Publishers
    rclcpp::Publisher<mavros_msgs::msg::OverrideRCIn>::SharedPtr rc_pub_;

    // Services
    rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr mavros_arm_client_;
    rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr mavros_set_mode_client_;   

    // timer
    rclcpp::TimerBase::SharedPtr timer_;

    // functions
    bool set_arm(bool arm);
    bool disarm();
    bool set_mode(const std::string & mode);
    uint16_t map_pwm(float value, float threshold);
    void publish_rc(float forward, float lateral, float yaw);
    void follow_wp();

    struct waypoint
    {
        double x;
        double y;
        double z;
    };

    // Definitions
    std::vector<waypoint> wp_;
    double gain_;
    size_t index_wp_;
};

#endif // RC_CONTROL_HPP