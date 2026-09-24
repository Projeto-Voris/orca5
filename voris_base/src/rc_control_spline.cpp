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

    trajectory_index_ = 0;
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
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

    // velocity (linear and angular)
    current_vel_ = msg->twist.twist;

    if (!set_origin_)
    {
        origin_x_ = current_pose_.pose.position.x;
        origin_y_ = current_pose_.pose.position.y;
        origin_z_ = current_pose_.pose.position.z;

        // Transformation to the start position
        T_map_start.setOrigin(tf2::Vector3(origin_x_,origin_y_,origin_z_));
        tf2::Quaternion q;
        tf2::fromMsg(current_pose_.pose.orientation, q);
        T_map_start.setRotation(q);
        T_start_map_ = T_map_start.inverse();

        set_origin_ = true;
        trajectory_type_ = TrajectoryType::SQUARE;
        generateTrajectory();

        RCLCPP_INFO(get_logger(),"Origin: %.2f %.2f %.2f",T_start_map_.getOrigin().x(),T_start_map_.getOrigin().y(),T_start_map_.getOrigin().z());

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
    for (const auto& p : trajectory_transformed_)
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

void RCControlSpline::generateWaypoints()
{
    // Função que gera os waypoints
    wp_.clear();
    switch (trajectory_type_)
    {
        case TrajectoryType::SQUARE:
        {
            // Generate a square trajectory in the middle of the under side
            double L = trajectory_params_.side;
            double d = trajectory_params_.delta;
            double h = L/2;
            for (double x=0; x<=h; x+=d)
                wp_.push_back({x,-h,0});
            for (double y=-h+d; y<=h; y+=d)
                wp_.push_back({h,y,0});
            for (double x=h-d; x>=-h; x-=d)
                wp_.push_back({x,h,0});
            for (double y=h-d; y>=-h; y-=d)
                wp_.push_back({-h,y,0});
            for (double x=-h+d; x<0; x+=d)
                wp_.push_back({x,-h,0});

            break;
        }   
        case TrajectoryType::CIRCLE:
        {
            // Generate a circle trajectory with the center in (r,0)
            double r = trajectory_params_.radius;
            int N = std::ceil((2*M_PI*r)/trajectory_params_.delta);
            for (int i=0; i<N; i++)
            {
                double theta = 2*M_PI*i/N;
                wp_.push_back({r-r*cos(theta), r*sin(theta), 0.0});
            }
            break;
        }
        case TrajectoryType::SPIRAL:
        {
            // Generate a spiral trajectory with the center in (r,0)
            double r = trajectory_params_.radius;
            double dz = trajectory_params_.dz;
            int turns = trajectory_params_.turns;

            double length = turns*2*M_PI*r;
            int N = std::ceil(length/trajectory_params_.delta);

            for (int i=0; i<=N; i++)
            {
                double theta = turns*2*M_PI*i/N;
                wp_.push_back({r - r*cos(theta), r*sin(theta), dz*theta/(2*M_PI)});
            }
            break;
        }
    }
}

void RCControlSpline::generateTrajectory()
{
    generateWaypoints();
    if (wp_.size() < 2)
    {
        // The number of waypoints must be bigger than two points
        RCLCPP_INFO(this->get_logger(), "Need at least 2 waypoints");
    }

    // Generate a matrix of points 3 x number of waypoints
    Eigen::MatrixXd points(3, wp_.size());
    for (size_t i = 0; i < wp_.size(); i++)
    {
        points(0, i) = wp_[i].x;
        points(1, i) = wp_[i].y;
        points(2, i) = wp_[i].z;
    }

    // Interpolate the matrix to get a smooth curve 
    auto spline = Eigen::SplineFitting<Eigen::Spline<double, 3>>::Interpolate(points, 3);
    trajectory_.clear();

    // Transform the trajectory into points
    constexpr int sample = 100; // Define the number of point that will be get
    for (int i = 0; i <= sample; i++)
    {
        // Normalization to u between 0 and 1, spline eigen wait u between [0,1], evaluate where is the point 
        // u = 0  initial point; u = 1 final point
        double u = static_cast<double>(i)/static_cast<double>(sample);
        Eigen::Vector3d p = spline(u);

        waypoint pt;
        pt.x = p(0);
        pt.y = p(1);
        pt.z = p(2);

        trajectory_.push_back(pt);
    }
    trajectory_index_ = 0;
    // Transform the trajectory to the initial position
    trajectory_transformed_.clear();

    for (const auto &p : trajectory_)
    {
        tf2::Vector3 pt(p.x, p.y, p.z);

        tf2::Vector3 pt_map = T_map_start * pt;

        trajectory_transformed_.push_back({
            pt_map.x(),
            pt_map.y(),
            pt_map.z()

        });
    }
    publish_path();
    RCLCPP_INFO(this->get_logger(),"Generated trajectory with %ld points",trajectory_.size());
}

tf2::Quaternion RCControlSpline::lookAtTheDuct(size_t index, const geometry_msgs::msg::Point& robot_position)
{
    // Process Gram-Schmidt to change the orientation of the vector to keep the x axis toward the duct

    if (index == 0) {index = 1;}
    if (index >= trajectory_transformed_.size()-1) { index = trajectory_transformed_.size()-2;}

    // pp - previous point; p0 - current point; p1 - next point 
    Eigen::Vector3d pp(trajectory_transformed_[index-1].x, trajectory_transformed_[index-1].y, trajectory_transformed_[index-1].z);
    Eigen::Vector3d p0(trajectory_transformed_[index].x, trajectory_transformed_[index].y, trajectory_transformed_[index].z);
    Eigen::Vector3d p1(trajectory_transformed_[index+1].x, trajectory_transformed_[index+1].y, trajectory_transformed_[index+1].z);
    // Vector of the duct 
    Eigen::Vector3d pduct(duct_position_.pdx, duct_position_.pdy, duct_position_.pdz);

    // Vector of the current pose of the ROV
    Eigen::Vector3d robot(robot_position.x, robot_position.y, robot_position.z);

    // Vector that point to the duct
    Eigen::Vector3d duct = (pduct - robot).normalized(); 
    RCLCPP_INFO_ONCE(this->get_logger(),"Vector: x=%.3f y=%.3f z=%.3f", duct.x(), duct.y(), duct.z());

    // u is a vector tantent to the curve 
    Eigen::Vector3d u = (p1-p0).normalized();

    Eigen::Vector3d x_axis = duct.normalized();
    Eigen::Vector3d y_axis = u - u.dot(x_axis)*x_axis;
    y_axis.normalize();
    Eigen::Vector3d z_axis = x_axis.cross(y_axis).normalized();
    // Ensure that the vector x_axis point to inside of the trajectory
    if (z_axis.z()<0) 
    {
        z_axis = -z_axis;
        y_axis = -y_axis;
    }

    // calculete the matrix of rotate
    Eigen::Matrix3d R;
    R.col(0) = x_axis; 
    R.col(1) = y_axis;
    R.col(2) = z_axis;

    // Convert to quartenion
    Eigen::Quaterniond q(R);
    tf2::Quaternion q_tf;
    q_tf.setValue(q.x(), q.y(), q.z(), q.w());
    return q_tf;
}

void RCControlSpline::follow_spline_curve()
{
    if (trajectory_index_>= trajectory_transformed_.size()) {
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

    if (trajectory_transformed_.empty())
    {
        RCLCPP_ERROR(this->get_logger(), "Trajectory is empty");
        return;
    }
    waypoint target = trajectory_transformed_[trajectory_index_];
    tf2::Quaternion q_desired = lookAtTheDuct(trajectory_index_, current_pose_.pose.position);

    // Current Pose
    double current_x = current_pose_.pose.position.x;
    double current_y = current_pose_.pose.position.y;
    double current_z = current_pose_.pose.position.z;
    double current_yaw = tf2::getYaw(current_pose_.pose.orientation);

    double error_x = target.x - current_x;
    double error_y = target.y - current_y;
    double error_z = target.z - current_z;

    // calculte the orientation
    tf2::Quaternion q_current;
    tf2::fromMsg(current_pose_.pose.orientation, q_current);
    tf2::Quaternion q_error = q_desired*q_current.inverse();
    q_error.normalize();

    // Obtain the yaw angle 
    tf2::Matrix3x3 m(q_error);
    double roll,pitch,yaw;
    m.getRPY(roll,pitch,yaw);

    // Rotation of the ROV
    double error_forward = std::cos(current_yaw)*error_x + std::sin(current_yaw)*error_y;
    double error_lateral = -std::sin(current_yaw)*error_x + std::cos(current_yaw)*error_y;

    double yaw_cmd = 0.3*yaw;
    double forward_cmd = 0.3*error_forward;
    double lateral_cmd = 0.3*error_lateral;
    double depth_cmd = 0.3*error_z;

    forward_cmd = std::clamp(forward_cmd,-0.3,0.3);
    lateral_cmd = std::clamp(lateral_cmd,-0.3,0.3);
    depth_cmd = std::clamp(depth_cmd,-0.3,0.3);
    yaw_cmd = std::clamp(yaw_cmd, -0.3, 0.3);

    // Euclidean distance
    double distance = std::hypot(error_x,error_y);
    float threshold = 0.8;

    if (distance < threshold)
    {
        trajectory_index_++;
    }

    publish_rc(forward_cmd, lateral_cmd, depth_cmd, yaw_cmd);
    publish_path();
}

std::shared_ptr<RCControlSpline> RCControlSpline::instance = nullptr;
void RCControlSpline::sigintHandler(int)
{
    if (instance){
        instance->disarm();
    }
    rclcpp::shutdown();
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RCControlSpline>(); 
    RCControlSpline::instance = node;
    std::signal(SIGINT, RCControlSpline::sigintHandler);
    rclcpp::spin(node); 
    rclcpp::shutdown();
    return 0;
}