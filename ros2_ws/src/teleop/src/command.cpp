// Simple forward drive: move the robot straight ahead ~3 feet, then stop.

#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/color_rgba.hpp>

using namespace std::chrono_literals;

class ForwardThreeFeet : public rclcpp::Node
{
public:
    ForwardThreeFeet()
    : Node("forward_three_feet"),
      speed_mps_(0.25),                 // forward speed in m/s
      target_distance_m_(0.9144),       // 3 feet in meters
      started_(false),
      completed_(false)
    {
        cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        led_pub_ = create_publisher<std_msgs::msg::ColorRGBA>("/qbot_led_strip", 10);

        timer_ = create_wall_timer(50ms, std::bind(&ForwardThreeFeet::tick, this));
    }

private:
    void tick()
    {
        const auto now = this->now();

        if (!started_)
        {
            start_time_ = now;
            started_ = true;
            RCLCPP_INFO(get_logger(), "Driving forward 3 ft at %.2f m/s", speed_mps_);
        }

        double elapsed = (now - start_time_).seconds();
        double traveled = elapsed * speed_mps_;

        geometry_msgs::msg::Twist cmd{};
        std_msgs::msg::ColorRGBA led{};

        if (!completed_ && traveled < target_distance_m_)
        {
            cmd.linear.x = speed_mps_;
            led.r = 0.0; led.g = 1.0; led.b = 0.0; led.a = 1.0;  // green while moving
        }
        else
        {
            completed_ = true;
            cmd.linear.x = 0.0;
            led.r = 1.0; led.g = 1.0; led.b = 0.0; led.a = 1.0;  // yellow when done
        }

        cmd_pub_->publish(cmd);
        led_pub_->publish(led);
    }

    // Publishers
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Publisher<std_msgs::msg::ColorRGBA>::SharedPtr led_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // State
    rclcpp::Time start_time_;
    double speed_mps_;
    double target_distance_m_;
    bool started_;
    bool completed_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ForwardThreeFeet>());
    rclcpp::shutdown();
    return 0;
}
