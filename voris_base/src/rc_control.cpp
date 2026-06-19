#include "rc_control.hpp"

// Purpose:
// Follow waypoints using RC override commands

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
    timer_ = this->create_wall_timer(std::chrono::milliseconds(100), [this]() {follow_wp();});

    gain_ = 0.5;

    index_wp_ = 0;
    wp_ = {{0.0, 0.0, -4.0},{4.0, 0.0, -4.0}, {4.0, -2.5, -4.0}, {0.0, -2.5, -4.0}, {0.0, 0.0, 0.0}}; // usando map
    //wp_ = {{0.0, 0.0, 4.0},{4.0, 0.0, 4.0}, {4.0, 2.5, 4.0}, {0.0, 2.5, 4.0}, {0.0, 0.0, 0.0}}; // usando odom
    //wp_ = {{0.0, 0.0, -4.0}, {3.0,0.0,-4.0}, {3.5,0.0,-4.0}, {4.0, -1.0, -3.5},{4.0,-1.5,-3.5}, {4.0,-2.0, -3.0}, {3.5, -2.0,-2.5}, {3.0,-2.0,-2.0}, {3.0, 1.0, -1.5}, {0.0,0.0,0.0}};
    //wp_ = {{2.5,-1.0,0.0}};

    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    auto broadcast_timer_cb = [this](){geometry_msgs::msg::TransformStamped t;
        t.header.stamp = this->get_clock()->now();
        t.header.frame_id = "map";
        t.child_frame_id = "odom";
        t.transform.translation.x = 0.0;
        t.transform.translation.y = 0.0;
        t.transform.translation.z = 0.0;
        //rotação de 180° em x
        t.transform.rotation.x = 1.0;
        t.transform.rotation.y = 0.0;
        t.transform.rotation.z = 0.0;
        t.transform.rotation.w = 0.0;

        tf_broadcaster_->sendTransform(t);
    };
    timer_tf_ = this->create_wall_timer(std::chrono::milliseconds(100), broadcast_timer_cb);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
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
    // RCLCPP_INFO(this->get_logger(), "x:%4f | y:%4f", current_pose_.pose.position.x, current_pose_.pose.position.y);

    // velocity (linear and angular)
    current_vel_ = msg->twist.twist;

    if (!set_origin_)
    {
        origin_x_ = current_pose_.pose.position.x;
        origin_y_ = current_pose_.pose.position.y;
        origin_z_ = current_pose_.pose.position.z;

        set_origin_ = true;

        RCLCPP_INFO_ONCE(this->get_logger(), "origin_x:%4f | origin_y:%4f | origin_z:%4f", origin_x_, origin_y_,origin_z_);
    }
    RCLCPP_INFO_ONCE(this->get_logger(), "header_frame_id:%s ", current_pose_.header.frame_id.c_str());
}

void RCControl::transform_map_to_odom()
{
    try
    {
        current_pose_.header = current_pose_.header;
        current_pose_.pose = current_pose_.pose;
        auto tf = tf_buffer_->lookupTransform("odom", "map", tf2::TimePointZero);
        tf2::doTransform(current_pose_, pose_odom_, tf);
        current_pose_ = pose_odom_;

        if (!set_origin_)
            {
                origin_x_ = current_pose_.pose.position.x;
                origin_y_ = current_pose_.pose.position.y;
                origin_z_ = current_pose_.pose.position.z;

                set_origin_ = true;

                RCLCPP_INFO_ONCE(this->get_logger(), "origin_x:%4f | origin_y:%4f | origin_z:%4f", origin_x_, origin_y_,origin_z_);
            }
    }
    catch (const tf2::TransformException & ex)
    {
        RCLCPP_WARN(this->get_logger(), "%s", ex.what());
    }
}

bool RCControl::set_arm(bool arm)
{
    auto request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
    request->value = arm;
    // call mavros/cmd/arming
    if(mavros_arm_client_->service_is_ready()) {
        mavros_arm_client_->async_send_request(request);
        RCLCPP_INFO_ONCE(this->get_logger(), "Robot Armed");
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
        RCLCPP_INFO_ONCE(get_logger(), "Robot disarmed");
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

uint16_t RCControl::map_pwm(float value, bool reverse, float threshold)
{
    if (std::abs(value) < threshold) {
        return static_cast<uint16_t>(std::clamp(1500, 1100, 1900));
    }
    if (reverse) value=-value;
    int pwm = 1500 + (value * 400.0 * gain_);
    return static_cast<uint16_t>(std::clamp(pwm, 1100, 1900));
}

void RCControl::publish_rc(float forward, float lateral, float depth, float yaw)
{
    mavros_msgs::msg::OverrideRCIn rc_msg;

    for (uint16_t & channel : rc_msg.channels) {
        channel = mavros_msgs::msg::OverrideRCIn::CHAN_NOCHANGE;
      }
    rc_msg.channels[5 - 1] = map_pwm(forward,false, 0.1); // Forward 
    rc_msg.channels[6 - 1] = map_pwm(lateral,false, 0.1); // Lateral 
    rc_msg.channels[4 - 1] = map_pwm(yaw,true, 0.1); // Yaw
    rc_msg.channels[3 - 1] = map_pwm(depth,false, 0.1); // Depth

    // RCLCPP_INFO(this->get_logger(), "MOV: Frwd:%4d | Side:%4d | depth:%4d | yaw:%4d ", rc_msg.channels[5 - 1], rc_msg.channels[6 - 1], rc_msg.channels[3 - 1], rc_msg.channels[4 - 1]);

    rc_pub_->publish(rc_msg);
}

void RCControl::follow_wp()
{
    if (index_wp_ >= wp_.size()) {
        RCLCPP_INFO_ONCE(get_logger(), "All waypoints reached");
        disarm();
        publish_rc(0.0, 0.0, 0.0, 0.0);
        return;
    } else {
        RCLCPP_INFO_ONCE(get_logger(), "Starting follow waypoints");
        if (!connected_) {
            RCLCPP_INFO_ONCE(get_logger(), "Robot not connected");
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

    // Calculo do erro da distancia do ponto final (erro_posição = ponto_final - ponto_atual)

    // usando posição inicial como referencia
    double current_x = current_pose_.pose.position.x - origin_x_;
    double current_y = current_pose_.pose.position.y - origin_y_;
    double current_z = current_pose_.pose.position.z - origin_z_;
    double error_x = target_wp_.x - current_x;
    // Soma do erro em y devido a divergencia do sistema de coordena do ardusub com o do mavros
    double error_y = target_wp_.y + current_y;
    double error_z = target_wp_.z - current_z;

    // Sem usar posição inicial como referêrencia
    // double error_x = target_wp_.x - current_pose_.pose.position.x;
    // double error_y = target_wp_.y + current_pose_.pose.position.y;
    // double error_z = target_wp_.z - current_pose_.pose.position.z;

    // RCLCPP_INFO(this->get_logger(), "error_x:%4f | error_y:%4f | error_z:%4f", error_x, error_y, error_z);

    // Threshold para quando se aproximar suficiente do ponto trocar para o proximo e diminuir velocidade
    float threshold = 0.8;
    double yaw_threshold = 0.3;
    // Calculo da distancia horizontal
    double distance = std::sqrt(error_x*error_x + error_y*error_y);

    // Calculo da posição yaw para fazer o giro em torno do eixo e virar para a posição de referência
    double target_yaw = std::atan2(error_y, error_x);
    double current_yaw = -tf2::getYaw(current_pose_.pose.orientation);
    double error_yaw = std::atan2(std::sin(target_yaw - current_yaw), std::cos(target_yaw - current_yaw));

    if (state_ == controlState::ROTATE)
    {
        // controle da velocidade de rotação do sub
        double yaw_speed = 0.2;
        double yaw_cmd = rotate_direction_*yaw_speed;

        // stop sub to rotate
        publish_rc(0.0, 0.0, error_z, yaw_cmd);

        // align the sub with target position
        if (std::abs(error_yaw) < yaw_threshold)
        {
            // robo não pode girar quando está alinhado
            publish_rc(0.0, 0.0, error_z, 0.0);
            RCLCPP_INFO(this->get_logger(), "Rotation complete");

            state_ = controlState::MOVE;
        }
        return;
    }

    if (state_ == controlState::MOVE)
    {
        if (distance < threshold)
        {
            // evita que o robo fique muito torto antes de ir para proxima posição
            publish_rc(0.0, 0.0, error_z, 0.0);
            RCLCPP_INFO(get_logger(), "waypoint %ld reached", index_wp_);
            index_wp_++;
            state_ = controlState::ROTATE;
            return;
        }
    }

    double forward_cmd;
    double vel_forward;

    // Verifica qual eixo o sub está (x ou y) pega a informação de velocidade para aquele eixo
    if (std::abs(std::cos(current_yaw)) >
        std::abs(std::sin(current_yaw)))
    {
        vel_forward = current_vel_.linear.x;
    }
    else
    {
        vel_forward = current_vel_.linear.y;
    }

    // usa a velocidade atual descontada da distancia como um erro de posição na direção x do robo
    forward_cmd = distance - vel_forward;
    RCLCPP_INFO(this->get_logger(), "X:%4f | Y:%4f | Z:%4f", current_pose_.pose.position.x, current_pose_.pose.position.y, current_pose_.pose.position.z);
    RCLCPP_INFO(this->get_logger(), "forward:%4f", forward_cmd);
    RCLCPP_INFO(this->get_logger(), "error_z:%4f", error_z);
    publish_rc(forward_cmd, 0.0, error_z, 0.0);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RCControl>(); 
    rclcpp::spin(node); 
    rclcpp::shutdown();
    return 0;
}