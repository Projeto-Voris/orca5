#include "rc_control_spline.hpp"

// Purpose:
// Follow waypoints using RC override commands

RCControlSpline::RCControlSpline() : Node("rc_control_spline")
{
    // Initialize publishers
    rc_pub_ = this->create_publisher<mavros_msgs::msg::OverrideRCIn>("mavros/rc/override", 10);
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/trajectory_path", 10);

    // Initialize services
    mavros_arm_client_ = this->create_client<mavros_msgs::srv::CommandBool>("mavros/cmd/arming");
    mavros_set_mode_client_ = this->create_client<mavros_msgs::srv::SetMode>("/mavros/set_mode");

    // Subcriptions
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/model/bluerov2/odometry", 10, std::bind(&RCControlSpline::odom_cb, this, std::placeholders::_1));
    state_sub_ = this->create_subscription<mavros_msgs::msg::State>("/mavros/state", 10, std::bind(&RCControlSpline::state_cb, this, std::placeholders::_1));

    // Initialize timer to publish RC commands at a fixed rate
    timer_ = this->create_wall_timer(std::chrono::milliseconds(100), [this]() {follow_spline_curve();});

    gain_ = 0.5;

    index_wp_ = 0;
    // Trajetória linear
    //wp_ = {{0.0, 0.0, 0.0},{1.0, 0.0, 0.0}, {2.0, 0.0, 0.0}, {3.0, 0.0, 0.0},{3.0, 2.5, 0.0}, {2.0, 2.5, 0.0}, {1.0,2.5,0.0},{0.0, 2.5, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 0.0}};
    // Trajetória circular: 1 volta
    //wp_ = {{0.0, 0.0, -2.0}, {1.0, 0.0, -2.0}, {2.0, 0.0, -2.0}, {3.0, 0.0, -2.0}, {4.0, 1.0,-2.0}, {3.0, 2.0, -2.0}, {2.0, 1.0, -2.0},{1.5, 0.0, -2.0}, {1.0, 0.0, -2.0}, {0.0, 0.0, 0.0}};
    // Trajetória circular: espiral
    wp_ = {{0.0, 0.0, -4.0}, {1.0,0.0,-4.0}, {2.0, 0.0, -4.0}, {3.0, 0.0, -4.0}, {4.0, 1.0,-4.0}, {3.0, 2.0, -4.0}, {2.0, 1.0, -4.0}, {3.0,0.0,-3.0},{4.0,1.0,-3.0},{3.0,2.0,-3.0}, {2.0,1.0,-3.0},{3.0, 0.0, -2.0}, {4.0,1.0,-2.0}, {3.0,2.0,-2.0}, {1.0, 1.0,0.0}, {0.0,0.0,0.0}};
    trajectory_index_ = 0;
}

void RCControlSpline::state_cb(const mavros_msgs::msg::State::ConstSharedPtr & msg)
{
    connected_ = msg->connected;
    armed_ = msg->armed;
    mode_ = msg->mode;
}

void RCControlSpline::odom_cb(const nav_msgs::msg::Odometry::ConstSharedPtr & msg)
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
        generate_trajectory();
        publish_path();

        RCLCPP_INFO_ONCE(this->get_logger(), "origin_x:%4f | origin_y:%4f | origin_z:%4f", origin_x_, origin_y_,origin_z_);
    }
    RCLCPP_INFO_ONCE(this->get_logger(), "header_frame_id:%s ", current_pose_.header.frame_id.c_str());
}

bool RCControlSpline::set_arm(bool arm)
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

bool RCControlSpline::disarm()
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

bool RCControlSpline::set_mode(const std::string & mode)
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

uint16_t RCControlSpline::map_pwm(float value, bool reverse, float threshold)
{
    if (std::abs(value) < threshold) {
        return static_cast<uint16_t>(std::clamp(1500, 1100, 1900));
    }
    if (reverse) value=-value;
    int pwm = 1500 + (value * 400.0 * gain_);
    return static_cast<uint16_t>(std::clamp(pwm, 1100, 1900));
}

void RCControlSpline::publish_rc(float forward, float lateral, float depth, float yaw)
{
    mavros_msgs::msg::OverrideRCIn rc_msg;

    for (uint16_t & channel : rc_msg.channels) {
        channel = mavros_msgs::msg::OverrideRCIn::CHAN_NOCHANGE;
      }
    rc_msg.channels[5 - 1] = map_pwm(forward,false, 0.1); // Forward 
    rc_msg.channels[6 - 1] = map_pwm(lateral,true, 0.1); // Lateral 
    rc_msg.channels[4 - 1] = map_pwm(yaw,true, 0.1); // Yaw
    rc_msg.channels[3 - 1] = map_pwm(depth,false, 0.1); // Depth

    // RCLCPP_INFO(this->get_logger(), "MOV: Frwd:%4d | Side:%4d | depth:%4d | yaw:%4d ", rc_msg.channels[5 - 1], rc_msg.channels[6 - 1], rc_msg.channels[3 - 1], rc_msg.channels[4 - 1]);

    rc_pub_->publish(rc_msg);
}

void RCControlSpline::publish_path()
{
    nav_msgs::msg::Path path;
    path.header.stamp = this->now();
    path.header.frame_id = "map";
    for (const auto& p : trajectory_)
    {
        geometry_msgs::msg::PoseStamped pose;
        pose.header = path.header;
        pose.pose.position.x = p.x;
        pose.pose.position.y = p.y;
        pose.pose.position.z = p.z;
        pose.pose.orientation.w = 1.0;
        path.poses.push_back(pose);
    }
    path_pub_->publish(path);
}

void RCControlSpline::generate_trajectory()
{
    // Gera uma trajetória curva 3D, resultado conjunto de pontos
    if (wp_.size() < 2)
    {
        // Para geração de uma curva é necessário nó mínimo dois pontos
        RCLCPP_INFO(this->get_logger(), "Need at least 2 waypoints");
    }

    // Matriz de pontos 3 x numeros de waypoints para calculo da spline
    Eigen::MatrixXd points(3, wp_.size());
    for (size_t i = 0; i < wp_.size(); i++)
    {
        points(0, i) = wp_[i].x;
        points(1, i) = wp_[i].y;
        points(2, i) = wp_[i].z;
    }

    // Faz a interpolação da matrix de pontos para obter uma trjatória curva suave
    auto spline = Eigen::SplineFitting<Eigen::Spline<double, 3>>::Interpolate(points, 3);
    trajectory_.clear();

    // Transformar a trajetória obtida em pontos para seguir
    constexpr int sample = 500; // Defini o número de pontos que será obtido
    for (int i = 0; i <= sample; i++)
    {
        // Normalização para u entre 0 e 1, spline eigen espera u entre [0,1], avalia onde é o ponto
        // u = 0  ponto inicial; u = 1 ponto é final
        double u = static_cast<double>(i)/static_cast<double>(sample);
        Eigen::Vector3d p = spline(u);

        waypoint pt;
        pt.x = p(0);
        pt.y = p(1);
        pt.z = p(2);

        trajectory_.push_back(pt);
    }
    trajectory_index_ = 0;
    RCLCPP_INFO(this->get_logger(),"Generated trajectory with %ld points",trajectory_.size());
}

void RCControlSpline::deltaCartesianPoints(){
    // Armazena waypoints como deslocamento ao invés de coordenas absolutas
    // Dizer Ex.: "vá 4 metros para frente" ao invés de "vá para o ponto (4, 0)"
    delta_trajectory_.clear();
    for (size_t i = 0; i < trajectory_.size(); ++i)
    {
        DeltaWaypoint d;
        if (i == 0)
        {
            //primeiro ponto (ponto inicial)
            d.dx = trajectory_[0].x;
            d.dy = trajectory_[0].y;
            d.dz = trajectory_[0].z;
        }
        else 
        {
            d.dx = trajectory_[i].x - trajectory_[i-1].x;
            d.dy = trajectory_[i].y - trajectory_[i-1].y;
            d.dz = trajectory_[i].z - trajectory_[i-1].z;
        }
        delta_trajectory_.push_back(d);
        trajectory_.clear();

        double x = origin_x_;
        double y = origin_y_;
        double z = origin_z_;

        for (const auto &d : delta_trajectory_)
        {
            x += d.dx;
            y += d.dy;
            z += d.dz;

            trajectory_.push_back({x,y,z});
        }
    }
}

void RCControlSpline::follow_spline_curve()
{
    if (trajectory_index_>= trajectory_.size()) {
        RCLCPP_INFO_ONCE(get_logger(), "Trajectory completed");
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

    if (trajectory_.empty())
    {
        RCLCPP_ERROR(this->get_logger(), "Trajectory is empty");
        return;
    }

    auto target = trajectory_[trajectory_index_];

    // inicio onde o robô começa
    double current_x = current_pose_.pose.position.x;
    double current_y = current_pose_.pose.position.y;
    double current_z = current_pose_.pose.position.z;

    double error_x = target.x - current_x;
    double error_y = target.y - current_y;
    double error_z = target.z - current_z;

    // Calcular a rotação em yaw com base nos waypoints
    double target_yaw = atan2(error_y,error_x);
    double current_yaw = tf2::getYaw(current_pose_.pose.orientation);
    double error_yaw = std::atan2(std::sin(target_yaw - current_yaw), std::cos(target_yaw - current_yaw));

    double yaw_cmd = 0.8*error_yaw;
    yaw_cmd = std::clamp(yaw_cmd, -0.8, 0.8);

    double distance = std::sqrt(error_x*error_x + error_y*error_y);
    float threshold = 0.8;

    if (distance < threshold)
    {
        trajectory_index_++;
    }

    // Calcular a velocidade do camando para frente
    double accel_f = 0.8; // reduzir velocidade
    double forward_cmd = accel_f*distance;
    forward_cmd = std::clamp(forward_cmd, 0.0, 0.3);
    publish_rc(forward_cmd, 0.0, error_z, yaw_cmd);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RCControlSpline>(); 
    rclcpp::spin(node); 
    rclcpp::shutdown();
    return 0;
}