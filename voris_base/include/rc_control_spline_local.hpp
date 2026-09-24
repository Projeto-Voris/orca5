#ifndef RC_CONTROL_SPLINE_LOCAL_HPP
#define RC_CONTROL_SPLINE_LOCAL_HPP

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

// construtor da classe
class RCControlSplineLocal : public rclcpp::Node
{
public:
    RCControlSplineLocal();
    bool disarm();

    static std::shared_ptr<RCControlSplineLocal> instance;
    static void sigintHandler(int);

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
    geometry_msgs::msg::PoseStamped pose_base_;

    // Publishers
    rclcpp::Publisher<mavros_msgs::msg::OverrideRCIn>::SharedPtr rc_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

    // Services
    rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr mavros_arm_client_;
    rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr mavros_set_mode_client_;   

    // timer
    rclcpp::TimerBase::SharedPtr timer_;

    // functions
    bool set_arm(bool arm);
    bool set_mode(const std::string & mode);
    uint16_t map_pwm(float value, bool reverse, float threshold);
    void publish_rc(float forward, float lateral, float depth, float yaw);
    void follow_spline_curve();
    void publish_path();

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

    void generate_trajectory();
    std::vector<waypoint> trajectory_;
    size_t trajectory_index_;

    static constexpr double PI = 3.14159265358979323846;

    // Create a variable to store the current behavior mode of the sub
    enum class controlState
    {
        ROTATE,
        MOVE
    };

    // initial state
    controlState state_ = controlState::ROTATE;
    // define um estado inicial onde a rotação é positiva
    double rotate_direction_ = 1.0;

    rclcpp::TimerBase::SharedPtr timer_tf_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // definir a origem o ponto que o robo começa
    bool set_origin_ = false;
    double origin_x_;
    double origin_y_;
    double origin_z_;

    // Criar um vetor com deltas, para aramzenar o incremento dos pontos da trajetória
    struct DeltaWaypoint
    {
        double dx;
        double dy;
        double dz;
    };

    std::vector<DeltaWaypoint> delta_trajectory_;
    void deltaCartesianPoints();
    void buildTrajectory();
    void transform_map_to_base();
};

#endif // RC_CONTROL_SPLINE_LOCAL_HPP