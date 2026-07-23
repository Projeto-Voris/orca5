#ifndef TRAJECTORY_CONTROLLER_HPP
#define TRAJECTORY_CONTROLLER_HPP

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
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/utils.h"
#include "tf2_ros/transform_broadcaster.hpp"
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <Eigen/Dense>
#include <unsupported/Eigen/Splines>
#include <csignal>

class TrajectoryController : public rclcpp::Node
{
public:
    TrajectoryController();
    bool disarm();

    static std::shared_ptr<TrajectoryController> instance;
    static void sigintHandler(int);

private:
    bool connected_{};
    bool armed_{};
    bool set_origin_ = false;
    std::string mode_ = "";
    double gain_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    void odom_cb(const nav_msgs::msg::Odometry::ConstSharedPtr & msg);

    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
    void state_cb(const mavros_msgs::msg::State::ConstSharedPtr & msg);

    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    void transformPath(const nav_msgs::msg::Path::ConstSharedPtr & msg);

    // Current robot state
    geometry_msgs::msg::PoseStamped current_pose_;
    geometry_msgs::msg::Twist current_vel_;
    geometry_msgs::msg::PoseStamped pose_odom_;

    rclcpp::Publisher<mavros_msgs::msg::OverrideRCIn>::SharedPtr rc_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

    rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr mavros_arm_client_;
    rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr mavros_set_mode_client_;   

    rclcpp::TimerBase::SharedPtr timer_;

    bool set_arm(bool arm);
    bool set_mode(const std::string & mode);
    uint16_t map_pwm(float value, bool reverse, float threshold);
    void publish_rc(float forward, float lateral, float depth, float yaw);
    void publishPath();
    void pathFollower();

    struct DuctPosition
    {
        double pdx{3.0};
        double pdy{1.5};
        double pdz{-1.0};
    };
    DuctPosition duct_position_;

    rclcpp::TimerBase::SharedPtr timer_tf_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    tf2::Transform T_start_map_;
    tf2::Transform T_map_start;
    tf2::Quaternion lookAtTheDuct(const geometry_msgs::msg::Point& robot_position);
    std::vector<tf2::Quaternion> trajectory_orientation_;

    double origin_x_;
    double origin_y_;
    double origin_z_;

    struct waypoint
    {
        double x;
        double y;
        double z;
    };

    double k_depth;
    double k_yaw;
    
    std::vector<waypoint> path_transformed_;
    size_t trajectory_index_;
    std::string frame_id_;
};

#endif // TRAJECTORY_CONTROLLER_HPP