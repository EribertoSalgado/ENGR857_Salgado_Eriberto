#include <chrono>
#include <cmath>
#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"

#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/color_rgba.hpp>

#include "quanser/quanser_hid.h"
#include "quanser/quanser_messages.h"

using namespace std::chrono_literals;

class CommandPublisher : public rclcpp::Node
{
public:
    CommandPublisher()
    : Node("joystick_publisher")
    {
        this->declare_parameter("front_obstacle_distance", 1.0);
        this->declare_parameter("front_obstacle_angle_degrees", -90.0);
        this->declare_parameter("front_obstacle_cone_degrees", 60.0);
        this->declare_parameter("obstacle_scan_timeout_seconds", 0.5);
        this->declare_parameter("yellow_blink_period_seconds", 0.5);

        command_publisher_ =
            this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

        led_publisher_ =
            this->create_publisher<std_msgs::msg::ColorRGBA>("qbot_led_strip", 10);

        scan_subscriber_ =
            this->create_subscription<sensor_msgs::msg::LaserScan>(
                "/scan",
                10,
                std::bind(&CommandPublisher::scan_callback, this, std::placeholders::_1));

        open_controller();

        timer_ = this->create_wall_timer(
            50ms, std::bind(&CommandPublisher::timer_callback, this));
    }

    ~CommandPublisher()
    {
        if (controller_open_)
        {
            game_controller_close(gamepad_);
        }
    }

private:
    static constexpr double kPi = 3.14159265358979323846;
    static constexpr t_uint32 kAButtonMask = (1U << 0);
    static constexpr t_uint32 kLeftBumperMask = (1U << 4);

    void open_controller()
    {
        t_uint8 controller_number = 1;
        t_uint16 buffer_size = 12;
        t_double deadzone[6] = {0.0};
        t_double saturation[6] = {0.0};
        t_boolean auto_center = false;
        t_uint16 max_force_feedback_effects = 0;
        t_double force_feedback_gain = 0.0;

        const t_error open_result = game_controller_open(
            controller_number,
            buffer_size,
            deadzone,
            saturation,
            auto_center,
            max_force_feedback_effects,
            force_feedback_gain,
            &gamepad_);

        if (open_result >= 0)
        {
            controller_open_ = true;
            RCLCPP_INFO(this->get_logger(), "Controller connected.");
            return;
        }

        controller_open_ = false;
        RCLCPP_ERROR(
            this->get_logger(),
            "Could not open controller %u. Quanser error: %d",
            static_cast<unsigned int>(controller_number),
            open_result);
    }

    void timer_callback()
    {
        poll_controller();
        publish_velocity();
        publish_status_led();
    }

    void poll_controller()
    {
        if (!controller_open_)
        {
            throttle_ = 0.0;
            steering_ = 0.0;
            return;
        }

        t_boolean is_new = false;
        const t_error poll_result =
            game_controller_poll(gamepad_, &controller_state_, &is_new);

        if (poll_result < 0)
        {
            throttle_ = 0.0;
            steering_ = 0.0;
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                2000,
                "Could not poll controller. Quanser error: %d",
                poll_result);
            return;
        }

        const bool a_pressed = (controller_state_.buttons & kAButtonMask) != 0;
        const bool left_bumper_pressed =
            (controller_state_.buttons & kLeftBumperMask) != 0;

        if (!left_bumper_pressed)
        {
            throttle_ = 0.0;
            steering_ = 0.0;
            return;
        }

        const t_double left_stick_x = -1.0 * controller_state_.x;
        const t_double right_trigger = controller_state_.rz;

        if (right_trigger == 0.0)
        {
            throttle_ = 0.0;
        }
        else
        {
            throttle_ = 0.3 * (0.5 + 0.5 * right_trigger);
        }

        steering_ = 0.5 * left_stick_x;

        if (a_pressed)
        {
            throttle_ = -throttle_;
        }
    }

    void publish_velocity()
    {
        geometry_msgs::msg::Twist twist;
        twist.linear.x = throttle_;
        twist.angular.z = steering_;
        command_publisher_->publish(twist);
    }

    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan)
    {
        obstacle_in_front_ = scan_has_front_obstacle(*scan);
        last_scan_time_ = this->get_clock()->now();

        if (obstacle_in_front_ != previous_obstacle_in_front_)
        {
            if (obstacle_in_front_)
            {
                RCLCPP_INFO(this->get_logger(), "Front obstacle detected.");
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), "Front obstacle cleared.");
            }
            previous_obstacle_in_front_ = obstacle_in_front_;
        }
    }

    bool scan_has_front_obstacle(const sensor_msgs::msg::LaserScan & scan)
    {
        const double obstacle_distance =
            this->get_parameter("front_obstacle_distance").as_double();
        const double front_angle =
            degrees_to_radians(
                this->get_parameter("front_obstacle_angle_degrees").as_double());
        const double half_cone =
            std::abs(degrees_to_radians(
                this->get_parameter("front_obstacle_cone_degrees").as_double())) / 2.0;

        for (std::size_t i = 0; i < scan.ranges.size(); ++i)
        {
            const float range = scan.ranges[i];
            if (!std::isfinite(range) ||
                range <= scan.range_min ||
                range > scan.range_max ||
                range > obstacle_distance)
            {
                continue;
            }

            const double angle =
                scan.angle_min + static_cast<double>(i) * scan.angle_increment;
            if (std::abs(shortest_angular_distance(angle, front_angle)) <= half_cone)
            {
                return true;
            }
        }

        return false;
    }

    bool front_obstacle_is_active()
    {
        if (!obstacle_in_front_ || last_scan_time_.nanoseconds() == 0)
        {
            return false;
        }

        const double timeout_seconds =
            this->get_parameter("obstacle_scan_timeout_seconds").as_double();
        const rclcpp::Duration scan_age =
            this->get_clock()->now() - last_scan_time_;

        return scan_age.seconds() <= timeout_seconds;
    }

    void publish_status_led()
    {
        if (front_obstacle_is_active())
        {
            if (yellow_blink_is_on())
            {
                publish_led(1.0, 1.0, 0.0);
            }
            else
            {
                publish_led(0.0, 0.0, 0.0);
            }
            return;
        }

        publish_led(0.0, 1.0, 0.0);
    }

    bool yellow_blink_is_on()
    {
        const double period_seconds =
            this->get_parameter("yellow_blink_period_seconds").as_double();
        const double safe_period_seconds =
            period_seconds > 0.0 ? period_seconds : 0.5;
        const double now_seconds = this->get_clock()->now().seconds();

        return std::fmod(now_seconds, safe_period_seconds) <
            (safe_period_seconds / 2.0);
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

    static double degrees_to_radians(double degrees)
    {
        return degrees * kPi / 180.0;
    }

    static double shortest_angular_distance(double angle, double target)
    {
        double difference = std::fmod(angle - target + kPi, 2.0 * kPi);
        if (difference < 0.0)
        {
            difference += 2.0 * kPi;
        }
        return difference - kPi;
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr command_publisher_;
    rclcpp::Publisher<std_msgs::msg::ColorRGBA>::SharedPtr led_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_;

    t_game_controller gamepad_;
    t_game_controller_states controller_state_{};

    rclcpp::Time last_scan_time_{0, 0, RCL_ROS_TIME};
    bool controller_open_ = false;
    bool obstacle_in_front_ = false;
    bool previous_obstacle_in_front_ = false;
    double throttle_ = 0.0;
    double steering_ = 0.0;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CommandPublisher>());
    rclcpp::shutdown();

    return 0;
}
