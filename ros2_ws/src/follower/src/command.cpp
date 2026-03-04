// Simple LIDAR-based follower controller
// Behaviors:
//  - Maintain ~1 m distance to nearest object in front (move forward/backward)
//  - Turn to face object if it drifts to the side
//  - Stop if no object is detected within 2 m ahead

#include <algorithm>
#include <cmath>
#include <functional>
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
    : Node("follower_controller")
    {
        // Tunable parameters
        scan_topic_ = this->declare_parameter<std::string>("scan_topic", "scan");
        desired_distance_    = this->declare_parameter<double>("desired_distance", 1.0);
        stop_distance_       = this->declare_parameter<double>("stop_distance", 2.0);
        min_detect_distance_ = this->declare_parameter<double>("min_detect_distance", 0.20);

        linear_kp_  = this->declare_parameter<double>("linear_kp", 0.6);
        angular_kp_ = this->declare_parameter<double>("angular_kp", 1.5);

        max_forward_speed_ = this->declare_parameter<double>("max_forward_speed", 0.4);
        max_reverse_speed_ = this->declare_parameter<double>("max_reverse_speed", 0.25);
        max_angular_speed_ = this->declare_parameter<double>("max_angular_speed", 1.2);

        front_fov_         = this->declare_parameter<double>("front_fov", 1.57);  // +/- 90 deg
        central_fov_       = this->declare_parameter<double>("central_fov", 1.0); // widen cone for distance control (~57 deg)
        search_angular_speed_ = this->declare_parameter<double>("search_angular_speed", 0.3);
        angle_deadband_    = this->declare_parameter<double>("angle_deadband", 0.08);
        distance_deadband_ = this->declare_parameter<double>("distance_deadband", 0.05);

        invert_linear_ = this->declare_parameter<bool>("invert_linear", false);
        invert_angular_ = this->declare_parameter<bool>("invert_angular", false);

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
        geometry_msgs::msg::Twist cmd;  // default zeros → stop

        if (!latest_scan_)
        {
            cmd_pub_->publish(cmd);
            return;
        }

        auto scan = latest_scan_;
        double best_range = std::numeric_limits<double>::infinity();
        double best_angle = 0.0;
        bool found_any = false;
        bool found_central = false;

        // Find the closest valid return within the forward field of view
        for (size_t i = 0; i < scan->ranges.size(); ++i)
        {
            double r = scan->ranges[i];
            if (!std::isfinite(r) || r < scan->range_min || r > scan->range_max)
                continue;

            double angle = scan->angle_min + i * scan->angle_increment;
            if (std::abs(angle) > front_fov_)
                continue; // ignore objects far off to the side/back

            if (r < min_detect_distance_ || r > stop_distance_)
                continue; // ignore things too close (self/ground) or beyond stop distance

            found_any = true;

            if (std::abs(angle) <= central_fov_)
            {
                if (r < best_range)
                {
                    best_range = r;
                    best_angle = angle;
                    found_central = true;
                }
            }
        }

        // No object within the stop_distance band → stay still
        if (!found_any)
        {
            cmd_pub_->publish(cmd);
            return;
        }

        // Nothing in central cone yet: stop (no turning allowed)
        if (!found_central)
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

        // No turning: keep angular.z = 0
        cmd.angular.z = 0.0;

        // When turning hard, reduce forward/back speed to keep stable and bias forward motion toward facing target
        double turn_factor = 1.0; // no turn, keep linear scaling simple
        double heading_factor = std::max(0.0, std::cos(best_angle)); // no forward motion if target is far to the side
        cmd.linear.x *= turn_factor * heading_factor;

        // Allow quick inversion if the platform wiring uses opposite sign
        if (invert_linear_)
        {
            cmd.linear.x = -cmd.linear.x;
        }
        // invert_angular_ ignored because angular.z is fixed to 0

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
    double front_fov_;
    double central_fov_;
    double search_angular_speed_;
    double angle_deadband_;
    double distance_deadband_;
    bool invert_linear_;
    bool invert_angular_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FollowerNode>());
    rclcpp::shutdown();
    return 0;
}
