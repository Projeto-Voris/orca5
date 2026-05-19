#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "mavros_msgs/msg/state.hpp"
#include "mavros_msgs/srv/command_bool.hpp"
#include "mavros_msgs/srv/message_interval.hpp"
#include "mavros_msgs/srv/set_mode.hpp"
#include "nav2_msgs/srv/manage_lifecycle_nodes.hpp"
#include "nav2_util/service_client.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
// orca_msgs/action?targetMode --> Importa modos do ORCA
#include "orca_msgs/action/target_mode.hpp"

// Gerencia o estado do sub: Disarmado/armado, automático e remoto

using namespace std::chrono_literals;
using TargetMode = orca_msgs::action::TargetMode;
using GoalHandleTargetMode = rclcpp_action::ServerGoalHandle<TargetMode>;

class Controller : public rclcpp::Node
{
    const std::string MAVROS_ARM_SRV = "/mavros/cmd/arming";
    const std::string MAVROS_SET_MODE_SRV = "/mavros/set_mode";
    const std::string MAVROS_SET_MSG_INTERVAL_SRV = "/mavros/set_message_interval";
    const std::string NAV2_MGR_SRV = "/lifecycle_manager_navigation/manage_nodes";
    const std::string BASE_SRV = "/conn";

    // Parameters
    rclcpp::TimerBase::SharedPtr mode_timer_;
    rclcpp::TimerBase::SharedPtr mav_msg_rate_timer_;
    geometry_msgs::msg::PoseStamped current_pose_;

    // Parameters to set the message intervals for MAVROS topics this need two input message ID (vector fo some coordinate, like attitude) and message rate
    std::vector<int64_t> mav_msg_ids_;
    int64_t mav_msg_rate_{};

    const std::string ARDUSUB_MODE_ALT_HOLD = "ALT_HOLD";
    const std::string ARDUSUB_MODE_POS_HOLD = "POSHOLD";
    const std::string ARDUSUB_MODE_MANUAL = "MANUAL";

    bool connected_{};
    bool armed_{};
    bool nav2_active_{};
    bool have_pose_{};
    bool base_driving_{};
    std::string mode_;

    // Declared Services called by this node
    std::shared_ptr<nav2_util::ServiceClient<mavros_msgs::srv::CommandBool>> mavros_arm_client_;
    std::shared_ptr<nav2_util::ServiceClient<mavros_msgs::srv::SetMode>> mavros_set_mode_client_;
    std::shared_ptr<nav2_util::ServiceClient<nav2_msgs::srv::ManageLifecycleNodes>> nav2_mgr_client_;
    std::shared_ptr<nav2_util::ServiceClient<mavros_msgs::srv::MessageInterval>> mavros_set_msg_interval_client_;
    std::shared_ptr<nav2_util::ServiceClient<std_srvs::srv::SetBool>> base_client_;

    // Declared Subscribers
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;

    std::shared_ptr<GoalHandleTargetMode> goal_handle_;
    uint8_t current_mode_{TargetMode::Goal::ORCA_MODE_DISARMED};
    uint8_t target_mode_{TargetMode::Goal::ORCA_MODE_DISARMED};

    void pose_cb(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        if (!have_pose_)
        {
            have_pose_ = true;
            RCLCPP_INFO(get_logger(), "EKF is running");
        }
        current_pose_ = *msg;
    }

    void set_arm(bool arm)
    {
        auto request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
        auto response = std::make_shared<mavros_msgs::srv::CommandBool::Response>();
        request->value = arm;

        RCLCPP_INFO(get_logger(), "calling mavros/cmd/arming...");
        auto result = mavros_arm_client_->invoke(request, response);
        result = result && response->success;
        RCLCPP_INFO(get_logger(), result ? "success" : "failure");
    }

    void change_mode(const std::string & mode)
    {
        auto request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
        auto response = std::make_shared<mavros_msgs::srv::SetMode::Response>();
        request->custom_mode = mode;

        RCLCPP_INFO(get_logger(), "calling mavros/set_mode...");
        auto result = mavros_set_mode_client_->invoke(request, response);
        result = result && response->mode_sent;
        RCLCPP_INFO(get_logger(), result ? "success" : "failure");
    }
    bool set_message_rate(uint8_t msg_id)
    {
        auto request = std::make_shared<mavros_msgs::srv::MessageInterval::Request>();
        auto response = std::make_shared<mavros_msgs::srv::MessageInterval::Response>();
        request->message_id = msg_id;
        request->message_rate = static_cast<float>(mav_msg_rate_);

        RCLCPP_DEBUG(
        get_logger(), "set message rate for %d to %g hz",
        request->message_id, request->message_rate);

        return mavros_set_msg_interval_client_->invoke(request, response) && response->success;
    }

    void set_message_rates()
    {
        RCLCPP_INFO_ONCE(get_logger(), "setting message rates to %ld hz every 10s", mav_msg_rate_);

        for (auto msg_id : mav_msg_ids_) {
        set_message_rate(msg_id);
        }
    }

    bool call_nav2(uint8_t command)
    {
        auto request = std::make_shared<nav2_msgs::srv::ManageLifecycleNodes::Request>();
        auto response = std::make_shared<nav2_msgs::srv::ManageLifecycleNodes::Response>();
        request->command = command;

        RCLCPP_INFO(get_logger(), "calling nav2...");
        auto result = nav2_mgr_client_->invoke(request, response);
        result = result && response->success;
        RCLCPP_INFO(get_logger(), result ? "success" : "failure");
        return result;
    }

    bool call_base(bool conn)
    {
        auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
        auto response = std::make_shared<std_srvs::srv::SetBool::Response>();
        request->data = conn;

        RCLCPP_INFO(get_logger(), "calling base...");
        auto result = base_client_->invoke(request, response);
        result = result && response->success;
        RCLCPP_INFO(get_logger(), result ? "success" : "failure");
        return result;
    }

    void state_cb(const mavros_msgs::msg::State::SharedPtr msg)
    {
        if (connected_ != msg->connected)
        {
            connected_ = msg->connected;
            if (connected_)
            {
                RCLCPP_INFO(get_logger(), "ArduSub connected");

            }
            else 
            {
                RCLCPP_INFO(get_logger(), "ArduSub disconnected");
                mode_timer_ = nullptr;
                mav_msg_rate_timer_ = nullptr;
            }
        }

        if (armed_ != msg->armed)
        {
            armed_ = msg->armed;
            RCLCPP_INFO(get_logger(), msg->armed ? "armed" : "disarmed");
        }

        if (mode_ != msg->mode)
        {
            mode_ = msg->mode;
            RCLCPP_INFO(get_logger(), "ArduSub mode is %s", msg->mode.c_str());
        }
    }

    void go_auv()
    {
        if (connected_ && have_pose_)
        {
            if (!armed_)
            {
                set_arm(true);
            }

            if (mode_ != ARDUSUB_MODE_POS_HOLD)
            {
                change_mode(ARDUSUB_MODE_POS_HOLD);
            }

            if (!base_driving_)
            {
                if (call_base(true))
                {
                    base_driving_ = true;
                }
            }

            if (!nav2_active_)
            {
                if (call_nav2(nav2_msgs::srv::ManageLifecycleNodes::Request::STARTUP))
                {
                    nav2_active_ = true;
                }
            }

            if (armed_ && mode_ == ARDUSUB_MODE_POS_HOLD && base_driving_ && nav2_active_)
            {
                current_mode_ = TargetMode::Goal::ORCA_MODE_AUV;

                if (goal_handle_)
                {
                    goal_handle_->succeed(std::make_shared<TargetMode::Result>());
                    goal_handle_ = nullptr;
                }
            
                RCLCPP_INFO(get_logger(), "current mode is ORCA_MODE_AUV");

            }
        }
    }

    void go_rov()
    {
        // This function is use when the operator/pilot control the sub, so we don't need arm or change mode in here, but the pilot have
        if (connected_ && have_pose_)
        {
            if (base_driving_)
            {
                call_base(false);
                base_driving_ = false;
            }

            if (nav2_active_)
            {
                call_nav2(nav2_msgs::srv::ManageLifecycleNodes::Request::SHUTDOWN);
                nav2_active_ = false;
            }

            current_mode_ = TargetMode::Goal::ORCA_MODE_ROV;    

            if (goal_handle_)
            {
                goal_handle_->succeed(std::make_shared<TargetMode::Result>());
                goal_handle_ = nullptr;
            }

            RCLCPP_INFO(get_logger(), "current mode is ORCA_MODE_ROV");
        }
    }

    void go_to_target_mode()
    {
        if (current_mode_ != target_mode_)
        {
            switch (target_mode_)
            {
                case orca_msgs::action::TargetMode::Goal::ORCA_MODE_AUV:
                    go_auv();
                    break;
                case orca_msgs::action::TargetMode::Goal::ORCA_MODE_ROV:
                    go_rov();
                    break;
                case orca_msgs::action::TargetMode::Goal::ORCA_MODE_DISARMED:
                    go_disarmed();
                    break;
            }
        }
    }

    void go_disarmed()
    {
        if (connected_)
        {
            if (armed_)
            {
                set_arm(false);
            }

            if (mode_ != ARDUSUB_MODE_MANUAL)
            {
                change_mode(ARDUSUB_MODE_MANUAL);
            }

            if (base_driving_)
            {
                call_base(false);
                base_driving_ = false;
            }

            if (nav2_active_)
            {
                call_nav2(nav2_msgs::srv::ManageLifecycleNodes::Request::SHUTDOWN);
                nav2_active_ = false;
            }

            if (!armed_)
            {
                current_mode_ = TargetMode::Goal::ORCA_MODE_DISARMED;

                if (goal_handle_)
                {
                    goal_handle_->succeed(std::make_shared<TargetMode::Result>());
                    goal_handle_ = nullptr;
                }

                RCLCPP_INFO(get_logger(), "current mode is ORCA_MODE_DISARMED");

            }
        }
    } 

public:
    Controller() : Node{"controller"}
    {
        rclcpp::QoS best_effort(10);
        best_effort.best_effort();

        rclcpp::QoS reliable(10);
        reliable.reliable();
        
        // Subscribers
        pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("/mavros/local_position/pose", best_effort, 
            std::bind(&Controller::pose_cb, this, std::placeholders::_1));
        state_sub_ = this->create_subscription<mavros_msgs::msg::State>("/mavros/state", reliable,
            std::bind(&Controller::state_cb, this, std::placeholders::_1));

        // Service Clients
        mavros_arm_client_ = std::make_shared<nav2_util::ServiceClient<mavros_msgs::srv::CommandBool>>(this, MAVROS_ARM_SRV);
        mavros_set_mode_client_ = std::make_shared<nav2_util::ServiceClient<mavros_msgs::srv::SetMode>>(this, MAVROS_SET_MODE_SRV);
        nav2_mgr_client_ = std::make_shared<nav2_util::ServiceClient<nav2_msgs::srv::ManageLifecycleNodes>>(this, NAV2_MGR_SRV);
        mavros_set_msg_interval_client_ = std::make_shared<nav2_util::ServiceClient<mavros_msgs::srv::MessageInterval>>(this, MAVROS_SET_MSG_INTERVAL_SRV);
        
        // Timers
        mode_timer_ = this->create_wall_timer(1s, std::bind(&Controller::go_to_target_mode, this));
        mav_msg_rate_timer_ = this->create_wall_timer(10s, std::bind(&Controller::set_message_rates, this));

        base_client_ = std::make_shared<nav2_util::ServiceClient<std_srvs::srv::SetBool>>(this, BASE_SRV);
    }
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Controller>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}