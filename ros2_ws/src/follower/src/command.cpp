// LIDAR-based follower controller
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

class FollowerNode : public rclcpp::Node
{
public:
    FollowerNode()
    : Node("follower")
    {
        // Tunable parameters
        scan_topic_          = this->declare_parameter<std::string>("scan_topic", "scan");
        desired_distance_    = this->declare_parameter<double>("desired_distance", 0.3); // 1 foot = 0.3048 m 
        stop_distance_       = this->declare_parameter<double>("stop_distance", 0.6); // beyond this → stop (no target)
        min_detect_distance_ = this->declare_parameter<double>("min_detect_distance", 0.20);

        linear_kp_           = this->declare_parameter<double>("linear_kp", 0.6);
        angular_kp_          = this->declare_parameter<double>("angular_kp", 1.2);

        max_forward_speed_   = this->declare_parameter<double>("max_forward_speed", 0.35);
        max_reverse_speed_   = this->declare_parameter<double>("max_reverse_speed", 0.25);
        max_angular_speed_   = this->declare_parameter<double>("max_angular_speed", 1.2);

        // Angular window to consider targets (degrees, normalized to [0,360) after offset).
        angle_min_deg_       = this->declare_parameter<double>("angle_min_deg", 0);   // default: full 360
        angle_max_deg_       = this->declare_parameter<double>("angle_max_deg", ); // default: full 360
        angle_deadband_      = this->declare_parameter<double>("angle_deadband", 0.05);
        distance_deadband_   = this->declare_parameter<double>("distance_deadband", 0.05);
        turn_slowdown_gain_  = this->declare_parameter<double>("turn_slowdown_gain", 0.5);
        invert_angular_      = this->declare_parameter<bool>("invert_angular", false);
        enable_turning_      = this->declare_parameter<bool>("enable_turning", true); // turn to
        invert_linear_       = this->declare_parameter<bool>("invert_linear", false);
        angle_offset_        = this->declare_parameter<double>("angle_offset", -90); // radians, rotate scan frame to align front

        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            scan_topic_, rclcpp::SensorDataQoS(),
            std::bind(&FollowerNode::scanCallback, this, std::placeholders::_1));

        timer_ = this->create_wall_timer(100ms, std::bind(&FollowerNode::controlLoop, this));
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
            double adj_angle = angle + angle_offset_; // compensate mounting rotation
            adj_angle = std::atan2(std::sin(adj_angle), std::cos(adj_angle)); // normalize to [-pi, pi]
            double angle_deg = adj_angle * 57.29577951308232; // rad → deg
            if (angle_deg < 0.0)
                angle_deg += 360.0; // map to [0, 360)

            bool in_window;
            if (angle_min_deg_ <= angle_max_deg_)
            {
                in_window = (angle_deg >= angle_min_deg_ && angle_deg <= angle_max_deg_);
            }
            else
            {
                // handle wrap-around windows (e.g., 300° to 60°)
                in_window = (angle_deg >= angle_min_deg_ || angle_deg <= angle_max_deg_);
            }
            if (!in_window)
                continue; // ignore returns outside the allowed angular sector

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
    double angle_min_deg_;
    double angle_max_deg_;
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
    rclcpp::spin(std::make_shared<FollowerNode>());
    rclcpp::shutdown();
    return 0;
}
