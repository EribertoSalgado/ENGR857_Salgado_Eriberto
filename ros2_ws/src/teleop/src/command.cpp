// Move the QBot forward 1 foot, then stop.

#include <chrono>
#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/color_rgba.hpp"

using namespace std::chrono_literals;

class CommandPublisher : public rclcpp::Node
{
public:
    CommandPublisher()
    : Node("one_foot_forward_command")
    {
        command_publisher_ =
            this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

        led_publisher_ =
            this->create_publisher<std_msgs::msg::ColorRGBA>("qbot_led_strip", 10);

        move_duration_ = std::chrono::duration<double>(
            kDistanceMeters / kForwardSpeedMetersPerSecond);

        timer_ = this->create_wall_timer(
            100ms, std::bind(&CommandPublisher::timer_callback, this));
    }

private:
    static constexpr double kDistanceMeters = 0.3048;  // 1 foot
    static constexpr double kForwardSpeedMetersPerSecond = 0.10;
    static constexpr int kStopHoldMilliseconds = 500;

    void timer_callback()
    {
        const auto now = std::chrono::steady_clock::now();

        if (!move_started_)
        {
            publish_velocity(0.0, 0.0);
            publish_led(1.0, 1.0, 0.0);

            if (command_publisher_->get_subscription_count() == 0)
            {
                if (!waiting_logged_)
                {
                    RCLCPP_INFO(
                        this->get_logger(),
                        "Waiting for a cmd_vel subscriber before moving.");
                    waiting_logged_ = true;
                }
                return;
            }

            move_started_ = true;
            move_start_time_ = now;
            RCLCPP_INFO(
                this->get_logger(),
                "Moving forward %.4f meters (1 foot) at %.2f m/s.",
                kDistanceMeters,
                kForwardSpeedMetersPerSecond);
        }

        if (!stopping_)
        {
            if (now - move_start_time_ < move_duration_)
            {
                publish_velocity(kForwardSpeedMetersPerSecond, 0.0);
                publish_led(0.0, 1.0, 0.0);
                return;
            }

            stopping_ = true;
            stop_start_time_ = now;
            RCLCPP_INFO(this->get_logger(), "One-foot move complete. Stopping.");
        }

        publish_velocity(0.0, 0.0);
        publish_led(1.0, 1.0, 0.0);

        if (now - stop_start_time_ >= std::chrono::milliseconds(kStopHoldMilliseconds))
        {
            rclcpp::shutdown();
        }
    }

    void publish_velocity(double linear_x, double angular_z)
    {
        geometry_msgs::msg::Twist twist;
        twist.linear.x = linear_x;
        twist.angular.z = angular_z;
        command_publisher_->publish(twist);
    }

    void publish_led(double red, double green, double blue)
    {
        std_msgs::msg::ColorRGBA led_msg;
        led_msg.r = red;
        led_msg.g = green;
        led_msg.b = blue;
        led_msg.a = 1.0;
        led_publisher_->publish(led_msg);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr command_publisher_;
    rclcpp::Publisher<std_msgs::msg::ColorRGBA>::SharedPtr led_publisher_;
    std::chrono::steady_clock::time_point move_start_time_;
    std::chrono::steady_clock::time_point stop_start_time_;
    std::chrono::duration<double> move_duration_{0.0};
    bool move_started_ = false;
    bool stopping_ = false;
    bool waiting_logged_ = false;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CommandPublisher>());
    if (rclcpp::ok())
    {
        rclcpp::shutdown();
    }
    return 0;
}
