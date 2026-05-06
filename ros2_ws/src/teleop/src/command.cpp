// Press A on the controller to move the QBot forward 1 foot, then stop.

#include <chrono>
#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/color_rgba.hpp"

#include "quanser/quanser_messages.h"
#include "quanser/quanser_hid.h"

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

        open_controller();

        RCLCPP_INFO(
            this->get_logger(),
            "Press A on the controller to move forward %.4f meters (1 foot).",
            kDistanceMeters);

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
    enum class MotionState
    {
        Idle,
        Moving,
        StopHold
    };

    static constexpr double kDistanceMeters = 0.3048;  // 1 foot
    static constexpr double kForwardSpeedMetersPerSecond = 0.10;
    static constexpr int kStopHoldMilliseconds = 500;
    static constexpr int kControllerRetryMilliseconds = 2000;
    static constexpr t_uint32 kAButtonMask = (1U << 0);

    void timer_callback()
    {
        const auto now = std::chrono::steady_clock::now();

        if (!controller_open_ &&
            now - last_controller_open_attempt_ >=
                std::chrono::milliseconds(kControllerRetryMilliseconds))
        {
            open_controller();
        }

        const bool a_pressed = poll_a_button_pressed();

        if (state_ == MotionState::Idle)
        {
            publish_velocity(0.0, 0.0);
            publish_led(1.0, 1.0, 0.0);

            if (a_pressed)
            {
                if (command_publisher_->get_subscription_count() == 0)
                {
                    RCLCPP_WARN(
                        this->get_logger(),
                        "A pressed, but no cmd_vel subscriber is ready yet.");
                    return;
                }

                state_ = MotionState::Moving;
                move_start_time_ = now;
                RCLCPP_INFO(
                    this->get_logger(),
                    "A pressed. Moving forward %.4f meters (1 foot) at %.2f m/s.",
                    kDistanceMeters,
                    kForwardSpeedMetersPerSecond);
            }
        }

        if (state_ == MotionState::Moving)
        {
            if (now - move_start_time_ < move_duration_)
            {
                publish_velocity(kForwardSpeedMetersPerSecond, 0.0);
                publish_led(0.0, 1.0, 0.0);
                return;
            }

            state_ = MotionState::StopHold;
            stop_start_time_ = now;
            RCLCPP_INFO(this->get_logger(), "One-foot move complete. Stopping.");
        }

        if (state_ == MotionState::StopHold)
        {
            publish_velocity(0.0, 0.0);
            publish_led(1.0, 1.0, 0.0);

            if (now - stop_start_time_ >= std::chrono::milliseconds(kStopHoldMilliseconds))
            {
                state_ = MotionState::Idle;
                RCLCPP_INFO(this->get_logger(), "Ready for the next A press.");
            }
        }
    }

    void open_controller()
    {
        last_controller_open_attempt_ = std::chrono::steady_clock::now();

        t_uint8 controller_number = 1;
        t_uint16 buffer_size = 12;
        t_double deadzone[6] = {0.0};
        t_double saturation[6] = {0.0};
        t_boolean auto_center = false;
        t_uint16 max_force_feedback_effects = 0;
        t_double force_feedback_gain = 0.0;

        const t_error result = game_controller_open(
            controller_number,
            buffer_size,
            deadzone,
            saturation,
            auto_center,
            max_force_feedback_effects,
            force_feedback_gain,
            &gamepad_);

        if (result >= 0)
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
            result);
    }

    bool poll_a_button_pressed()
    {
        if (!controller_open_)
        {
            return false;
        }

        t_boolean is_new = false;
        const t_error result = game_controller_poll(gamepad_, &controller_state_, &is_new);
        if (result < 0)
        {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                2000,
                "Could not poll controller. Quanser error: %d",
                result);
            return false;
        }

        const bool a_is_down = (controller_state_.buttons & kAButtonMask) != 0;
        const bool a_was_pressed = a_is_down && !a_was_down_;
        a_was_down_ = a_is_down;
        return a_was_pressed;
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

    t_game_controller gamepad_;
    t_game_controller_states controller_state_{};

    std::chrono::steady_clock::time_point move_start_time_;
    std::chrono::steady_clock::time_point stop_start_time_;
    std::chrono::steady_clock::time_point last_controller_open_attempt_;
    std::chrono::duration<double> move_duration_{0.0};
    MotionState state_ = MotionState::Idle;
    bool controller_open_ = false;
    bool a_was_down_ = false;
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
