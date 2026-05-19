#include <memory>
#include <string>
#include <utility>

#include "geographic_msgs/msg/geo_pose_stamped.hpp"
#include "mavros_msgs/msg/override_rc_in.hpp"
#include "orca_base/underwater_motion.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/set_bool.hpp"

// computa a posição z desejada e publica em /mavros/setpoint_position/global
// computa a posição x desejada e a orientação yaw desejada e publica em /mavros/rc/override

class Movement : public rclcpp::Node
{
  bool running_{false};
  bool conn_{false};
  bool init_pose_received_{false};

  BaseContext cxt_;

  // Recent messages
  geometry_msgs::msg::PoseStamped ardu_pose_;
  rclcpp::Time slam_pose_time_;
  geometry_msgs::msg::Twist cmd_vel_;

  // Motion model
  std::unique_ptr<UnderwaterMotion> underwater_motion_;

  // timer
  rclcpp::TimerBase::SharedPtr timer_;

  //Service provide for this node
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr conn_srv_;

  // subscriptions
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr ardu_pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr slam_pose_sub_;  

  // Publications
  rclcpp::Publisher<orca_msgs::msg::Motion>::SharedPtr motion_pub_;
  rclcpp::Publisher<mavros_msgs::msg::OverrideRCIn>::SharedPtr rc_pub_;
  rclcpp::Publisher<geographic_msgs::msg::GeoPoseStamped>::SharedPtr setpoint_pub_;

  void publish_setpoint()
  {
    geographic_msgs::msg::GeoPoseStamped msg;
    msg.header.stamp = now();
    msg.header.frame_id = "map";
    msg.pose.position.altitude = ardu_pose_.pose.position.z; // altitude do EKF
    setpoint_pub_->publish(msg);
  }

  void publish_rc()
  {
    if (rc_pub_->get_subscription_count() > 0) {
      mavros_msgs::msg::OverrideRCIn msg;

      // Garantir que os canais não usados não sejam afetados
      for (uint16_t & channel : msg.channels) {
        channel = mavros_msgs::msg::OverrideRCIn::CHAN_NOCHANGE;
      }

      // Forward (>1500 is forward)
      msg.channels[5 - 1] = orca::effort_to_pwm(cxt_.mdl_thrust_dz_pwm_, underwater_motion_->motion().effort.force.x);

      // Lateral (>1500 is to the right when viewed top-down, so flip the sign)
      msg.channels[6 - 1] = orca::effort_to_pwm(cxt_.mdl_thrust_dz_pwm_, -underwater_motion_->motion().effort.force.y);

      // Yaw (>1500 is clockwise when viewed top-down, so flip the sign)
      msg.channels[4 - 1] = orca::effort_to_pwm(cxt_.mdl_thrust_dz_pwm_, -underwater_motion_->motion().effort.torque.z);

      rc_pub_->publish(msg);
    }
  }

  void timer_cb()
  {
    if (running_) 
    {
      if (!underwater_motion_)
      {
        // Initialize underwater motion from the EKF pose
        underwater_motion_ = std::make_unique<UnderwaterMotion>(now(), get_logger(), cxt_, ardu_pose_.pose);
      }
      else
      {
        // Update motion from t-(1/rate) to t
        underwater_motion_->update(now(), cmd_vel_);
      }

      motion_pub_->publish(underwater_motion_->motion());

      if (conn_) 
      {
        publish_rc();
        publish_setpoint();
      } 
    }
  }

  void ardu_pose_cb(const geometry_msgs::msg::PoseStamped::SharedPtr & msg)
  {
    if (!init_pose_received_) 
    {
      ardu_pose_ = *msg;
      init_pose_received_ = true;
    } else 
    {
      // Atualizar apenas a posição z do EKF, mantendo as outras dimensões inalteradas
      ardu_pose_.pose.position.z = msg->pose.position.z;
    }
  }

  void slam_pose_cb(const geometry_msgs::msg::PoseStamped::SharedPtr & msg)
  {
    // Atualizar o tempo da última pose do SLAM recebida
    slam_pose_time_ = now();
  }

  void validate_parameters()
  {
    slam_timeout_ = std::chrono::milliseconds{cxt_.slam_timeout_ms_};
  }

  void init_parameters()
  {
    // Get parameters, this will immediately call validate_parameters()
    #undef CXT_MACRO_MEMBER
    #define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_LOAD_PARAMETER((*this), cxt_, n, t, d)
    CXT_MACRO_INIT_PARAMETERS(BASE_ALL_PARAMS, validate_parameters)

    // Register parameters
    #undef CXT_MACRO_MEMBER
    #define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_PARAMETER_CHANGED(n, t)
    CXT_MACRO_REGISTER_PARAMETERS_CHANGED((*this), cxt_, BASE_ALL_PARAMS, validate_parameters)

    // Log parameters
    #undef CXT_MACRO_MEMBER
    #define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_LOG_PARAMETER(RCLCPP_INFO, get_logger(), cxt_, n, t, d)
    BASE_ALL_PARAMS

    // Check that all command line parameters are defined
    #undef CXT_MACRO_MEMBER
    #define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_CHECK_CMDLINE_PARAMETER(n, t, d)
    CXT_MACRO_CHECK_CMDLINE_PARAMETERS((*this), BASE_ALL_PARAMS)
  }

public:
  Movement() : Node("movement")
  {
    rclcpp::QoS best_effort(10);
    best_effort.best_effort();

    rclcpp::QoS reliable(10);
    reliable.reliable();

    motion_pub_ = create_publisher<orca_msgs::msg::Motion>("motion", reliable);
    rc_pub_ = create_publisher<mavros_msgs::msg::OverrideRCIn>("/mavros/rc/override", reliable);

    setpoint_pub_ = create_publisher<geographic_msgs::msg::GeoPoseStamped>("/mavros/setpoint_position/global", best_effort);

    conn_srv_ = create_service<std_srvs::srv::SetBool>(
      "conn", std::bind(&Movement::connect_callback, this, std::placeholders::_1, std::placeholders::_2));

    cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>( "cmd_vel", reliable, std::bind(&Movement::cmd_vel_cb, this, std::placeholders::_1));

    ardu_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>( "/mavros/local_position/pose", best_effort, std::bind(&Movement::ardu_pose_cb, this, std::placeholders::_1));

    // pose do slam já é publicado no /mavros/vision_pose/pose, então só precisa se inscrever para receber as mensagens
    slam_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>("/mavros/vision_pose/pose", reliable, std::bind(&Movement::slam_pose_cb, this, std::placeholders::_1));

    timer_ = create_wall_timer(100ms, std::bind(&Movement::timer_cb, this));
  }

};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Movement>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
