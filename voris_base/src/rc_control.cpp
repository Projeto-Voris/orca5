#include "rc_control.hpp"

RCControl::RCControl() : Node("rc_control")
{
    // Initialize publishers
    rc_pub_ = this->create_publisher<mavros_msgs::msg::OverrideRCIn>("mavros/rc/override", 10);

    // Initialize services
    mavros_arm_client_ = this->create_client<mavros_msgs::srv::CommandBool>("mavros/cmd/arming");
    mavros_set_mode_client_ = this->create_client<mavros_msgs::srv::SetMode>("/mavros/set_mode");

    // Subcriptions
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/model/bluerov2/odometry", 10, std::bind(&RCControl::odom_cb, this, std::placeholders::_1));
    state_sub_ = this->create_subscription<mavros_msgs::msg::State>("/mavros/state", 10, std::bind(&RCControl::state_cb, this, std::placeholders::_1));

    // Initialize timer to publish RC commands at a fixed rate
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),  // Adjust the rate as needed
        [this]() {
            follow_wp();
        }
    );

    gain_ = 0.5;

    index_wp_ = 0;
    wp_ = {{4.0, 0.0, 0.0}, {4.0, -2.5, 0.0}, {0.0, -2.5, 0.0}, {0.0, 0.0, 0.0}};
}

void RCControl::state_cb(const mavros_msgs::msg::State::ConstSharedPtr & msg)
{
    connected_ = msg->connected;
    armed_ = msg->armed;
    mode_ = msg->mode;
}

void RCControl::odom_cb(const nav_msgs::msg::Odometry::ConstSharedPtr & msg)
{
    // position (position and orientation)
    current_pose_.header = msg->header;
    current_pose_.pose = msg->pose.pose;
    RCLCPP_INFO(this->get_logger(), "x:%4f | y:%4f", current_pose_.pose.position.x, current_pose_.pose.position.y);
    
    // velocity (linear and angular)
    current_vel_ = msg->twist.twist;
}

bool RCControl::set_arm(bool arm)
{
    auto request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
    request->value = arm;
    // call mavros/cmd/arming
    if(mavros_arm_client_->service_is_ready()) {
        mavros_arm_client_->async_send_request(request);
        RCLCPP_INFO(this->get_logger(), "Robot Armed");
        return true;
    }
    return false;
}

bool RCControl::disarm()
{
    if (!connected_) {
        return false; 
    }
    if (armed_) {
        bool result = set_arm(false);
        if (result) {
        RCLCPP_INFO(get_logger(), "Robot disarmed");
        }
        return result;
    }

    return true;
}

bool RCControl::set_mode(const std::string & mode)
{
    auto request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
    request->custom_mode = mode;
    if(mavros_set_mode_client_->service_is_ready()) {
        mavros_set_mode_client_->async_send_request(request);
        RCLCPP_INFO(this->get_logger(), "Robot change mode to: ");
        return true;
    }
    return false;
}

uint16_t RCControl::map_pwm(float value, float threshold)
{
    if (std::abs(value) < threshold) {
        return static_cast<uint16_t>(std::clamp(1500, 1100, 1900));
    }
    int pwm = 1500 + (value * 400.0 * gain_);
    return static_cast<uint16_t>(std::clamp(pwm, 1100, 1900));
}

void RCControl::publish_rc(float forward, float lateral, float depth)
{
    mavros_msgs::msg::OverrideRCIn rc_msg;

    for (uint16_t & channel : rc_msg.channels) {
        channel = mavros_msgs::msg::OverrideRCIn::CHAN_NOCHANGE;
      }
    
    // rc_msg.channels[0] = 1500
    // rc_msg.channels[1] = 1500
    // rc_msg
    rc_msg.channels[5 - 1] = map_pwm(forward, 0.05); // Forward 
    rc_msg.channels[6 - 1] = map_pwm(lateral, 0.05); // Lateral 
    rc_msg.channels[3 - 1] = map_pwm(depth, 0.05); // Depth

    RCLCPP_INFO(this->get_logger(), "MOV: Frwd:%4d | Side:%4d | depth:%4d", rc_msg.channels[5 - 1], rc_msg.channels[6 - 1], rc_msg.channels[3 - 1]);

    rc_pub_->publish(rc_msg);
}

void RCControl::follow_wp()
{
    if (index_wp_ >= wp_.size()) {
        RCLCPP_INFO(get_logger(), "All waypoints reached");
        disarm();
        publish_rc(0.0, 0.0, 0.0);
        return;
    } else {
        RCLCPP_INFO_ONCE(get_logger(), "Starting follow waypoints");
        if (!connected_) {
            RCLCPP_INFO(get_logger(), "Robot not connected");
            return;
        }
        if (mode_!= "MANUAL") {
            set_mode("MANUAL");
            return;
        }
        if (!armed_) {
            set_arm(true);
            return;
        }
    }

    auto target_wp_ = wp_[index_wp_];

    // erro_pose_ = wp - current_pose_
    double error_x = target_wp_.x - current_pose_.pose.position.x;
    // coloquei soma aqui porque a direção do robo difere com a coordenada do robo, talvez uma transformação seria mais ideal
    double error_y = target_wp_.y + current_pose_.pose.position.y;
    double error_z = target_wp_.z - current_pose_.pose.position.z;
    // double target_yaw = std::atan2(error_y, error_x);
    // double error_yaw = target_yaw - current_pose_.pose.orientation.z;
    RCLCPP_INFO(this->get_logger(), "error_x:%4f | error_y:%4f | error_z:%4f", error_x, error_y, error_z);
    // erro_vel_ = wp_vel_ - current_vel_ // pensar na logica depois talvez definir uma porcentagem 

    // trasformar para as coordenadas do corpo
    // double yaw = tf2::getYaw(current_pose_.pose.orientation);
    // double body_x = std::cos(yaw)*error_x+std::sin(yaw)*error_y;
    // double body_y = -std::sin(yaw)*error_x+std::cos(yaw)*error_y;
    // RCLCPP_INFO(this->get_logger(), "body_x:%4f | body_y:%4f", body_x, body_y);

    // definir um ganho para definir como o robo reage conforme a distancia, londe mais rapido perto mais devagar (segurança do movimento)
    publish_rc(error_x, error_y, error_z);

    // threshold para quando se aproximar suficiente do ponto trocar para o proximo
    float threshold = 0.5;
    double distance = std::sqrt(error_x*error_x + error_y*error_y + error_z*error_z);
    if (distance < threshold)
    {
        RCLCPP_INFO(get_logger(), "waypoint %ld reached", index_wp_);
        index_wp_++;
        return;
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RCControl>(); 
    rclcpp::spin(node); 
    rclcpp::shutdown();
    return 0;
}