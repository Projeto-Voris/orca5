#ifndef PATH_GENERATOR_HPP
#define PATH_GENERATOR_HPP

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

enum class PathType
{
    SQUARE,
    CIRCLE,
    SPIRAL,
    SERPENTINE
};

class PathGenerate : public rclcpp::Node
{
public:
    PathGenerate();

private:
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    void publishPath();
    void generatePath();
    void waypointGenerate();

    void squareGenerate();
    void circleGenerate();
    void spiralGenerate();
    void serpentineGenerate();

    struct waypoint
    {
        double x;
        double y;
        double z;
    };
    std::vector<waypoint> wp_;
    std::vector<waypoint> path_;
    size_t path_index_;
    
    PathType path_type_;
    std::string path_type_str;
    PathType stringToPathType(const std::string & type);

    struct PathParams
    {
        double delta{0.2};
        double side{2.0};
        double radius{1.0};
        double dz{-1.0};
        int turns{4};
        int depth{4};
    };
    PathParams path_params_;
    std::string frame_id_;
};

#endif // PATH_GENERATOR_HPP