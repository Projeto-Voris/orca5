#include "path_generator.hpp"

//Purpose: generate a path 

PathGenerate::PathGenerate() : Node("path_generate")
{
    this->declare_parameter<std::string>("frame_id", "map");
    this->declare_parameter<double>("delta", 0.2);
    this->declare_parameter<double>("side", 2.0);
    this->declare_parameter<double>("radius", 1.0);
    this->declare_parameter<double>("dz", -1.0);
    this->declare_parameter<int>("turns", 4);
    this->declare_parameter<int>("depth", 4);
    this->declare_parameter<std::string>("path_type", "circle");

    this->get_parameter("frame_id", frame_id_);
    this->get_parameter("delta", path_params_.delta);
    this->get_parameter("side", path_params_.side);
    this->get_parameter("radius", path_params_.radius);
    this->get_parameter("dz", path_params_.dz);
    this->get_parameter("turns", path_params_.turns);
    this->get_parameter("depth", path_params_.depth);
    this->get_parameter("path_type", path_type_str);
    path_type_ = stringToPathType(path_type_str);

    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("local_path", 10);
    timer_ = this->create_wall_timer(std::chrono::milliseconds(1000), [this](){generatePath();});
    path_index_ = 0;

    RCLCPP_INFO(this->get_logger(), "Generating path");
}

void PathGenerate::publishPath()
{
    nav_msgs::msg::Path path;
    path.header.stamp = this->now();
    path.header.frame_id = frame_id_;
    for (const auto& p : path_)
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

PathType PathGenerate::stringToPathType(const std::string & type)
{
    if (type == "square") return PathType::SQUARE;
    if (type == "circle") return PathType::CIRCLE;
    if (type == "spiral") return PathType::SPIRAL;
    if (type == "serpentine") return PathType::SERPENTINE;

    throw std::runtime_error("Unknown path type: " + type);
}

void PathGenerate::waypointGenerate()
{
    switch (path_type_)
    {
        case PathType::SQUARE:
        {
            squareGenerate();
            break;
        }
        case PathType::CIRCLE:
        {
            circleGenerate();
            break;
        }
        case PathType::SPIRAL:
        {
            spiralGenerate();
            break;
        }
        case PathType::SERPENTINE:
        {
            serpentineGenerate();
            break;
        }
    }
}

void PathGenerate::squareGenerate()
{
    // Generate a square trajectory in the middle of the under side
    double L = path_params_.side;
    double d = path_params_.delta;
    double h = L/2;
    for (double y = 0; y <= h; y += d)
        wp_.push_back({0,y,0});
    for (double x = d; x <= L; x += d)
        wp_.push_back({x,h,0});
    for (double y = h-d; y >= -h; y -= d)
        wp_.push_back({L,y,0});
    for (double x = L-d; x >= 0; x -= d)
        wp_.push_back({x,-h,0});
    for (double y = -h+d; y < 0; y += d)
        wp_.push_back({0,y,0});
}

void PathGenerate::circleGenerate()
{
    // Generate a circle trajectory with the center in (r,0)
    double r = path_params_.radius;
    int N = std::ceil((2*M_PI*r)/path_params_.delta);
    for (int i=0; i<N; i++)
    {
        double theta = 2*M_PI*i/N;
        wp_.push_back({r-r*cos(theta), r*sin(theta), 0.0});
    }
}

void PathGenerate::spiralGenerate()
{
    // Generate a spiral trajectory with the center in (r,0)
    double r = path_params_.radius;
    double dz = path_params_.dz;
    int turns = path_params_.turns;

    double length = turns*2*M_PI*r;
    int N = std::ceil(length/path_params_.delta);

    for (int i=0; i<=N; i++)
    {
        double theta = turns*2*M_PI*i/N;
        wp_.push_back({r - r*cos(theta), r*sin(theta), dz*theta/(2*M_PI)});
    }
}

void PathGenerate::serpentineGenerate()
{
    // Generate a serpentine trajectory 
    double L = path_params_.side;
    double w = path_params_.depth;
    double d = path_params_.delta;
    double h = L/2;

    // Go down
    for (double z = 0; z >= -w; z -= d)
        wp_.push_back({0, 0, z});
    // Go right
    for (double y = d; y <= h; y += d)
        wp_.push_back({0, y, -w});
    // Go up
    for (double z = -w+d; z <= 0; z += d)
        wp_.push_back({0, h, z});
}

void PathGenerate::generatePath()
{
    wp_.clear();
    waypointGenerate();
    if (wp_.size() < 2)
    {
        // The number of waypoints must be bigger than two points
        RCLCPP_INFO(this->get_logger(), "Need at least 2 waypoints");
        return;
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
    path_.clear();

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

        path_.push_back(pt);
    }
    path_index_ = 0;
    publishPath();
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PathGenerate>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}