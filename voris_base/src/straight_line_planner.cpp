#include <string>
#include <memory>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "nav2_core/global_planner.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_util/robot_utils.hpp"
#include "nav2_ros_common/lifecycle_node.hpp"
#include "nav2_ros_common/node_utils.hpp"

class StraightLine : public nav2_core::GlobalPlanner
{
public:
    StraightLine() = default;
    ~StraightLine() override = default;

    // metodos necessarios para escrever um plugin configure(); activate(); deactivate(); cleanup(); createPlan();
    void configure(
        const nav2::LifecycleNode::WeakPtr & parent,
        std::string name, 
        std::shared_ptr<tf2_ros::Buffer> tf) override;
    
    void activate() override;
    void deactivate() override;
    void cleanup() override;

    void createPlan(
        const geometry_msgs::msg::PoseStamped & start,
        const geometry_msgs::msg::PoseStamped & goal,
        nav_msgs::msg::Path & global_path)
        {
            int num_poses = static_cast<int>(std::hypot(
                goal.pose.position.x - start.pose.position.x,
                goal.pose.position.y - start.pose.position.y,
                goal.pose.position.z - start.pose.position.z) / distance);
            double step_x = (goal.pose.position.x - start.pose.position.x) / num_poses;
            double step_y = (goal.pose.position.y - start.pose.position.y) / num_poses;
            double step_z = (goal.pose.position.z - start.pose.position.z) / num_poses;

            for (int i = 0; i <= num_poses; ++i) {
                geometry_msgs::msg::PoseStamped pose;
                pose.header.frame_id = start.header.frame_id;
                pose.pose.position.x = start.pose.position.x + step_x * i;
                pose.pose.position.y = start.pose.position.y + step_y * i;
                pose.pose.position.z = start.pose.position.z + step_z * i;
                global_path.poses.push_back(pose);
            }
        }
}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_straightline_planner::StraightLine, nav2_core::GlobalPlanner)
