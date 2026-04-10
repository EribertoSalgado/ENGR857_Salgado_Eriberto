// LIDAR-based delivery bot controller
// Behavior:
//  - Maintain ~1 m distance to the closest object in front
//  - Turn toward the object if it drifts to the side
//  - Stop if nothing is detected within ~2 m in the forward field of view

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

using namespace std::chrono_literals;

namespace
{
constexpr double kDegToRad = 0.017453292519943295769; // pi / 180

// Wrap angle to [-pi, pi] so comparisons against symmetric FOVs work
inline double wrapToPi(double rad)
{
    return std::atan2(std::sin(rad), std::cos(rad));
}
}

class DeliveryBotNode : public rclcpp::Node
{
public:
    DeliveryBotNode()
    : Node("delivery_bot")
    {
        // Tunable parameters
        scan_topic_          = this->declare_parameter<std::string>("scan_topic", "scan");
        desired_distance_    = this->declare_parameter<double>("desired_distance", 0.3); // ~1 foot
        stop_distance_       = this->declare_parameter<double>("stop_distance", 1); // stop if no object within this range
        min_detect_distance_ = this->declare_parameter<double>("min_detect_distance", 0.2);

        linear_kp_           = this->declare_parameter<double>("linear_kp", 0.6);
        angular_kp_          = this->declare_parameter<double>("angular_kp", 1.2);

        max_forward_speed_   = this->declare_parameter<double>("max_forward_speed", 0.35);
        max_reverse_speed_   = this->declare_parameter<double>("max_reverse_speed", 0.25);
        max_angular_speed_   = this->declare_parameter<double>("max_angular_speed", 1.2);

        // Limit scan processing to a specific angular sector (degrees in the scan frame, after angle_offset_)
        // Default to ±90° forward-looking window; adjust via parameters if your LiDAR is mounted differently.
        view_angle_min_rad_  = this->declare_parameter<double>("view_angle_min_deg", -30) * kDegToRad;
        view_angle_max_rad_  = this->declare_parameter<double>("view_angle_max_deg", 30) * kDegToRad;
        angle_deadband_      = this->declare_parameter<double>("angle_deadband", 0.05);
        distance_deadband_   = this->declare_parameter<double>("distance_deadband", 0.05);
        turn_slowdown_gain_  = this->declare_parameter<double>("turn_slowdown_gain", 0.5);
        invert_angular_      = this->declare_parameter<bool>("invert_angular", false);
        enable_turning_      = this->declare_parameter<bool>("enable_turning", true); // start with straight-line only
        invert_linear_       = this->declare_parameter<bool>("invert_linear", false);
        angle_offset_        = this->declare_parameter<double>("angle_offset", 1.5708); // radians, rotate scan frame to align front

        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            scan_topic_, rclcpp::SensorDataQoS(),
            std::bind(&DeliveryBotNode::scanCallback, this, std::placeholders::_1));

        timer_ = this->create_wall_timer(100ms, std::bind(&DeliveryBotNode::controlLoop, this));
    }

private:
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        latest_scan_ = msg;
    }

    void controlLoop()
    {
        geometry_msgs::msg::Twist cmd; // default zeros → stop

        if (!latest_scan_)
        {
            cmd_pub_->publish(cmd);
            return;
        }

        auto scan = latest_scan_;

        double best_range = std::numeric_limits<double>::infinity();
        double best_angle = 0.0;
        bool found_any = false;

        // Find the closest valid return within the forward field of view
        for (size_t i = 0; i < scan->ranges.size(); ++i)
        {
            double r = scan->ranges[i];
            if (!std::isfinite(r) || r < scan->range_min || r > scan->range_max)
                continue;

            double angle = scan->angle_min + i * scan->angle_increment;
            // Compensate mounting rotation and wrap so 0 rad means "front"
            double adj_angle = wrapToPi(angle + angle_offset_);
            if (adj_angle < view_angle_min_rad_ || adj_angle > view_angle_max_rad_)
                continue; // ignore returns outside the desired sector

            if (r < min_detect_distance_ || r > stop_distance_)
                continue; // too close (likely ground/robot) or beyond stop band

            found_any = true;
            if (r < best_range)
            {
                best_range = r;
                best_angle = adj_angle;
            }
        }

        // No object within the stop_distance band → stay still
        if (!found_any)
        {
            cmd_pub_->publish(cmd);
            return;
        }

        // Distance control (forward/backward)
        double distance_error = best_range - desired_distance_;
        if (std::abs(distance_error) > distance_deadband_)
        {
            double linear_cmd = linear_kp_ * distance_error;
            if (linear_cmd >= 0.0)
            {
                cmd.linear.x = std::min(linear_cmd, max_forward_speed_);
            }
            else
            {
                cmd.linear.x = std::max(linear_cmd, -max_reverse_speed_);
            }
        }

        if (invert_linear_)
        {
            cmd.linear.x = -cmd.linear.x;
        }

        // Heading control (turn toward the object)
        if (enable_turning_ && std::abs(best_angle) > angle_deadband_)
        {
            double angular_cmd = angular_kp_ * best_angle;
            angular_cmd = std::clamp(angular_cmd, -max_angular_speed_, max_angular_speed_);
            if (invert_angular_)
            {
                angular_cmd = -angular_cmd;
            }
            cmd.angular.z = angular_cmd;

            // Slow linear speed while turning so we don't overshoot
            double slowdown = 1.0 / (1.0 + turn_slowdown_gain_ * std::abs(best_angle));
            cmd.linear.x *= slowdown;
        }
        else
        {
            cmd.angular.z = 0.0; // no turning: pure forward/back distance keeping
        }

        cmd_pub_->publish(cmd);
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    sensor_msgs::msg::LaserScan::SharedPtr latest_scan_;

    // Parameters
    std::string scan_topic_;
    double desired_distance_;
    double stop_distance_;
    double min_detect_distance_;
    double linear_kp_;
    double angular_kp_;
    double max_forward_speed_;
    double max_reverse_speed_;
    double max_angular_speed_;
    double view_angle_min_rad_;
    double view_angle_max_rad_;
    double angle_deadband_;
    double distance_deadband_;
    double turn_slowdown_gain_;
    bool invert_angular_;
    bool enable_turning_;
    bool invert_linear_;
    double angle_offset_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DeliveryBotNode>());
    rclcpp::shutdown();
    return 0;
}
