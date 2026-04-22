// Delivery bot "move forward a fixed distance" controller.
// Default behavior: drive forward 3 feet (0.9144 m) and stop.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

using namespace std::chrono_literals;

namespace
{
constexpr double kFeetToMeters = 0.3048;
constexpr double kDefaultWheelRadiusM = 3.5 * 0.0254 / 2.0; // Matches qbot_platform_driver_interface.cpp
}

class DeliveryBotNode : public rclcpp::Node
{
public:
    DeliveryBotNode()
    : Node("delivery_bot")
    {
        joint_topic_ = this->declare_parameter<std::string>("joint_topic", "qbot_joint");
        cmd_topic_ = this->declare_parameter<std::string>("cmd_topic", "cmd_vel");

        target_distance_ft_ = this->declare_parameter<double>("target_distance_ft", 3.0);
        forward_speed_mps_ = this->declare_parameter<double>("forward_speed_mps", 0.25);
        min_speed_mps_ = this->declare_parameter<double>("min_speed_mps", 0.05);
        slowdown_distance_m_ = this->declare_parameter<double>("slowdown_distance_m", 0.20);
        wheel_radius_m_ = this->declare_parameter<double>("wheel_radius_m", kDefaultWheelRadiusM);
        feedback_timeout_s_ = this->declare_parameter<double>("feedback_timeout_s", 0.5);

        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(cmd_topic_, 10);
        joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            joint_topic_, rclcpp::SensorDataQoS(),
            std::bind(&DeliveryBotNode::jointCallback, this, std::placeholders::_1));

        timer_ = this->create_wall_timer(50ms, std::bind(&DeliveryBotNode::controlLoop, this));
    }

private:
    void jointCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        if (msg->position.size() < 2)
        {
            return;
        }

        last_joint_time_ = this->get_clock()->now();
        const double p0 = msg->position[0];
        const double p1 = msg->position[1];

        if (!have_start_)
        {
            start_pos0_ = p0;
            start_pos1_ = p1;
            have_start_ = true;
            RCLCPP_INFO(this->get_logger(), "Got wheel encoders. Starting forward motion.");
            return;
        }

        const double dtheta0 = p0 - start_pos0_;
        const double dtheta1 = p1 - start_pos1_;
        distance_traveled_m_ = wheel_radius_m_ * (dtheta0 + dtheta1) / 2.0;
    }

    void controlLoop()
    {
        geometry_msgs::msg::Twist cmd; // default zeros -> stop

        if (done_)
        {
            cmd_pub_->publish(cmd);
            return;
        }

        if (!have_start_)
        {
            // Wait for wheel encoder feedback from the driver before moving.
            cmd_pub_->publish(cmd);
            return;
        }

        const rclcpp::Time now = this->get_clock()->now();
        if ((now - last_joint_time_).seconds() > feedback_timeout_s_)
        {
            // Safety: if feedback stops updating, stop the robot.
            cmd_pub_->publish(cmd);
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "No joint feedback. Holding stop.");
            return;
        }

        const double target_distance_m = target_distance_ft_ * kFeetToMeters;
        const double traveled_abs_m = std::abs(distance_traveled_m_);
        const double remaining_m = target_distance_m - traveled_abs_m;

        if (remaining_m <= 0.0)
        {
            done_ = true;
            cmd_pub_->publish(cmd);
            RCLCPP_INFO(this->get_logger(), "Reached target distance: %.3f m (%.2f ft). Stopping.", target_distance_m, target_distance_ft_);
            return;
        }

        double speed_cmd = forward_speed_mps_;
        if (slowdown_distance_m_ > 0.0 && remaining_m < slowdown_distance_m_)
        {
            const double scale = std::clamp(remaining_m / slowdown_distance_m_, 0.0, 1.0);
            speed_cmd = std::max(min_speed_mps_, forward_speed_mps_ * scale);
        }

        cmd.linear.x = std::max(0.0, speed_cmd);
        cmd.angular.z = 0.0;
        cmd_pub_->publish(cmd);
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Parameters
    std::string joint_topic_;
    std::string cmd_topic_;
    double target_distance_ft_{3.0};
    double forward_speed_mps_{0.25};
    double min_speed_mps_{0.05};
    double slowdown_distance_m_{0.20};
    double wheel_radius_m_{kDefaultWheelRadiusM};
    double feedback_timeout_s_{0.5};

    // State
    bool have_start_{false};
    bool done_{false};
    rclcpp::Time last_joint_time_{0};
    double start_pos0_{0.0};
    double start_pos1_{0.0};
    double distance_traveled_m_{0.0};
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DeliveryBotNode>());
    rclcpp::shutdown();
    return 0;
}
