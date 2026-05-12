// Delivery bot FSM for QBot Platform.
// A -> signal intent, move forward 1 foot, wait for Y, return home 1 foot.
// LIDAR obstacle in front -> stop and blink yellow in any state.

#include <chrono>
#include <cmath>
#include <cstdlib>
#include <exception>
#include <functional>
#include <memory>
#include <string>
#include <thread>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/color_rgba.hpp"

#include "quanser/quanser_hid.h"
#include "quanser/quanser_messages.h"

using namespace std::chrono_literals;

class CommandPublisher : public rclcpp::Node
{
public:
    CommandPublisher()
    : Node("delivery_bot_command")
    {
        command_publisher_ =
            this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

        led_publisher_ =
            this->create_publisher<std_msgs::msg::ColorRGBA>("qbot_led_strip", 10);

        scan_subscriber_ =
            this->create_subscription<sensor_msgs::msg::LaserScan>(
                "/scan",
                10,
                std::bind(&CommandPublisher::scan_callback, this, std::placeholders::_1));

        this->declare_parameter("front_obstacle_distance", 1.0);
        this->declare_parameter("front_obstacle_angle_degrees", -90.0);
        this->declare_parameter("front_obstacle_cone_degrees", 60.0);
        this->declare_parameter("obstacle_scan_timeout_seconds", 0.5);

        this->declare_parameter("audio_enabled", true);
        this->declare_parameter<std::string>(
            "audio_player_command",
            "(command -v gst-launch-1.0 >/dev/null 2>&1 && gst-launch-1.0 -q playbin uri={uri}) || "
            "(command -v gst-play-1.0 >/dev/null 2>&1 && gst-play-1.0 --quiet {uri}) || "
            "(command -v mpg123 >/dev/null 2>&1 && mpg123 -q {file}) || "
            "(command -v mpg321 >/dev/null 2>&1 && mpg321 -q {file}) || "
            "(command -v mpv >/dev/null 2>&1 && mpv --no-video --really-quiet {file}) || "
            "(command -v play >/dev/null 2>&1 && play -q {file}) || "
            "(command -v ffplay >/dev/null 2>&1 && ffplay -nodisp -autoexit -loglevel quiet {file}) || "
            "(command -v cvlc >/dev/null 2>&1 && cvlc --play-and-exit --quiet {file})");
        this->declare_parameter<std::string>(
            "awaiting_pickup_audio_file",
            "audio/AwaitingPickup.mp3");
        this->declare_parameter<std::string>(
            "go_home_audio_file",
            "audio/GoHome.mp3");
        this->declare_parameter("awaiting_pickup_repeat_seconds", 30.0);
        this->declare_parameter("delivery_stop_wait_seconds", 15.0);

        move_duration_ = std::chrono::duration<double>(
            kDistanceMeters / kForwardSpeedMetersPerSecond);

        try
        {
            package_share_directory_ =
                ament_index_cpp::get_package_share_directory("qbot_platform");
        }
        catch (const std::exception & e)
        {
            RCLCPP_WARN(
                this->get_logger(),
                "Could not locate qbot_platform package share directory for audio files: %s",
                e.what());
        }

        open_controller();

        RCLCPP_INFO(
            this->get_logger(),
            "Delivery bot ready. Press A to deliver, then press Y to return home.");

        timer_ = this->create_wall_timer(
            50ms, std::bind(&CommandPublisher::timer_callback, this));
    }

    ~CommandPublisher()
    {
        publish_velocity(0.0, 0.0);

        if (controller_open_)
        {
            game_controller_close(gamepad_);
        }
    }

private:
    enum class MotionState
    {
        WaitingForInput,
        SignalingMove,
        MovingToDelivery,
        DeliveryStop,
        ReturningHome
    };

    struct ButtonPresses
    {
        bool a = false;
        bool y = false;
    };

    static constexpr double kPi = 3.14159265358979323846;
    static constexpr double kDistanceMeters = 0.6096;
    static constexpr double kForwardSpeedMetersPerSecond = 0.10;
    static constexpr int kSafetyDelayMilliseconds = 1500;
    static constexpr int kControllerRetryMilliseconds = 2000;
    static constexpr int kBlinkIntervalMilliseconds = 250;
    static constexpr t_uint32 kAButtonMask = (1U << 0);
    static constexpr t_uint32 kYButtonMask = (1U << 3);

    void timer_callback()
    {
        const auto now = std::chrono::steady_clock::now();

        if (!controller_open_ &&
            now - last_controller_open_attempt_ >=
                std::chrono::milliseconds(kControllerRetryMilliseconds))
        {
            open_controller();
        }

        if (front_obstacle_is_active())
        {
            publish_velocity(0.0, 0.0);
            publish_blinking_yellow(now);
            if (!obstacle_pause_active_)
            {
                obstacle_pause_active_ = true;
                obstacle_pause_start_time_ = now;
            }
            return;
        }

        if (obstacle_pause_active_)
        {
            state_start_time_ += now - obstacle_pause_start_time_;
            obstacle_pause_active_ = false;
        }

        const ButtonPresses buttons = poll_button_presses();

        switch (state_)
        {
            case MotionState::WaitingForInput:
                handle_waiting_for_input(now, buttons);
                break;
            case MotionState::SignalingMove:
                handle_signaling_move(now);
                break;
            case MotionState::MovingToDelivery:
                handle_moving_to_delivery(now);
                break;
            case MotionState::DeliveryStop:
                handle_delivery_stop(now, buttons);
                break;
            case MotionState::ReturningHome:
                handle_returning_home(now);
                break;
        }
    }

    void handle_waiting_for_input(
        const std::chrono::steady_clock::time_point & now,
        const ButtonPresses & buttons)
    {
        publish_velocity(0.0, 0.0);
        publish_blue();

        if (!buttons.a)
        {
            return;
        }

        if (command_publisher_->get_subscription_count() == 0)
        {
            RCLCPP_WARN(
                this->get_logger(),
                "A pressed, but no cmd_vel subscriber is ready yet.");
            return;
        }

        transition_to(MotionState::SignalingMove, now);
        RCLCPP_INFO(
            this->get_logger(),
            "A pressed. Signaling intent to move for %.1f seconds.",
            kSafetyDelayMilliseconds / 1000.0);
    }

    void handle_signaling_move(const std::chrono::steady_clock::time_point & now)
    {
        publish_velocity(0.0, 0.0);
        publish_yellow();

        if (now - state_start_time_ < std::chrono::milliseconds(kSafetyDelayMilliseconds))
        {
            return;
        }

        transition_to(MotionState::MovingToDelivery, now);
        RCLCPP_INFO(
            this->get_logger(),
            "Moving forward %.4f meters at %.2f m/s.",
            kDistanceMeters,
            kForwardSpeedMetersPerSecond);
    }

    void handle_moving_to_delivery(const std::chrono::steady_clock::time_point & now)
    {
        if (now - state_start_time_ < move_duration_)
        {
            publish_velocity(kForwardSpeedMetersPerSecond, 0.0);
            publish_blinking_green(now);
            return;
        }

        publish_velocity(0.0, 0.0);
        publish_green();
        transition_to(MotionState::DeliveryStop, now);
        play_awaiting_pickup_audio(now);
        RCLCPP_INFO(this->get_logger(), "Delivery stop reached. Press Y to return home.");
    }

    void handle_delivery_stop(
        const std::chrono::steady_clock::time_point & now,
        const ButtonPresses & buttons)
    {
        publish_velocity(0.0, 0.0);
        publish_green();

        const double wait_seconds =
            this->get_parameter("delivery_stop_wait_seconds").as_double();
        if (wait_seconds > 0.0 &&
            now - state_start_time_ >= std::chrono::duration<double>(wait_seconds))
        {
            transition_to(MotionState::ReturningHome, now);
            play_go_home_audio();
            RCLCPP_INFO(
                this->get_logger(),
                "Delivery wait finished after %.1f seconds. Returning home.",
                wait_seconds);
            return;
        }

        if (!buttons.y)
        {
            maybe_repeat_awaiting_pickup_audio(now);
            return;
        }

        if (command_publisher_->get_subscription_count() == 0)
        {
            RCLCPP_WARN(
                this->get_logger(),
                "Y pressed, but no cmd_vel subscriber is ready yet.");
            return;
        }

        transition_to(MotionState::ReturningHome, now);
        play_go_home_audio();
        RCLCPP_INFO(this->get_logger(), "Y pressed. Returning home.");
    }

    void handle_returning_home(const std::chrono::steady_clock::time_point & now)
    {
        if (now - state_start_time_ < move_duration_)
        {
            publish_velocity(-kForwardSpeedMetersPerSecond, 0.0);
            publish_blinking_green(now);
            return;
        }

        publish_velocity(0.0, 0.0);
        publish_blue();
        transition_to(MotionState::WaitingForInput, now);
        awaiting_pickup_audio_has_played_ = false;
        RCLCPP_INFO(this->get_logger(), "Returned home. Waiting for the next A press.");
    }

    void transition_to(
        MotionState next_state,
        const std::chrono::steady_clock::time_point & now)
    {
        state_ = next_state;
        state_start_time_ = now;
        obstacle_pause_active_ = false;
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

    ButtonPresses poll_button_presses()
    {
        ButtonPresses presses;

        if (!controller_open_)
        {
            return presses;
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
            return presses;
        }

        const bool a_is_down = (controller_state_.buttons & kAButtonMask) != 0;
        const bool y_is_down = (controller_state_.buttons & kYButtonMask) != 0;

        presses.a = a_is_down && !a_was_down_;
        presses.y = y_is_down && !y_was_down_;

        a_was_down_ = a_is_down;
        y_was_down_ = y_is_down;
        return presses;
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

    void publish_blue()
    {
        publish_led(0.0, 0.0, 1.0);
    }

    void publish_yellow()
    {
        publish_led(1.0, 1.0, 0.0);
    }

    void publish_green()
    {
        publish_led(0.0, 1.0, 0.0);
    }

    bool blink_is_on(const std::chrono::steady_clock::time_point & now) const
    {
        const auto elapsed_ms =
            std::chrono::duration_cast<std::chrono::milliseconds>(
                now.time_since_epoch()).count();

        return (elapsed_ms / kBlinkIntervalMilliseconds) % 2 == 0;
    }

    void publish_blinking_green(const std::chrono::steady_clock::time_point & now)
    {
        publish_led(0.0, blink_is_on(now) ? 1.0 : 0.0, 0.0);
    }

    void publish_blinking_yellow(const std::chrono::steady_clock::time_point & now)
    {
        if (blink_is_on(now))
        {
            publish_yellow();
        }
        else
        {
            publish_led(0.0, 0.0, 0.0);
        }
    }

    void play_awaiting_pickup_audio(const std::chrono::steady_clock::time_point & now)
    {
        last_awaiting_pickup_audio_time_ = now;
        awaiting_pickup_audio_has_played_ = true;
        play_audio_file(
            this->get_parameter("awaiting_pickup_audio_file").as_string(),
            "awaiting pickup");
    }

    void maybe_repeat_awaiting_pickup_audio(
        const std::chrono::steady_clock::time_point & now)
    {
        const double repeat_seconds =
            this->get_parameter("awaiting_pickup_repeat_seconds").as_double();
        if (repeat_seconds <= 0.0)
        {
            return;
        }

        if (!awaiting_pickup_audio_has_played_)
        {
            play_awaiting_pickup_audio(now);
            return;
        }

        const auto repeat_interval =
            std::chrono::duration<double>(repeat_seconds);
        if (now - last_awaiting_pickup_audio_time_ >= repeat_interval)
        {
            play_awaiting_pickup_audio(now);
        }
    }

    void play_go_home_audio()
    {
        play_audio_file(
            this->get_parameter("go_home_audio_file").as_string(),
            "go home");
    }

    void play_audio_file(const std::string & audio_file, const char * cue_name)
    {
        if (!this->get_parameter("audio_enabled").as_bool())
        {
            return;
        }

        const std::string audio_path = resolve_audio_path(audio_file);
        if (audio_path.empty())
        {
            return;
        }

        std::string audio_command =
            this->get_parameter("audio_player_command").as_string();
        if (audio_command.empty())
        {
            RCLCPP_WARN(
                this->get_logger(),
                "Audio cue '%s' skipped because audio_player_command is empty.",
                cue_name);
            return;
        }

        const std::string quoted_audio_path = shell_quote(audio_path);
        const std::string quoted_audio_uri = shell_quote(make_file_uri(audio_path));
        const bool inserted_file_placeholder =
            replace_all(audio_command, "{file}", quoted_audio_path);
        const bool inserted_uri_placeholder =
            replace_all(audio_command, "{uri}", quoted_audio_uri);

        if (!inserted_file_placeholder && !inserted_uri_placeholder)
        {
            audio_command += " " + quoted_audio_path;
        }

        const auto logger = this->get_logger();
        const std::string cue_label(cue_name);
        std::thread(
            [audio_command, logger, cue_label]()
            {
                const int result = std::system(audio_command.c_str());
                if (result != 0)
                {
                    RCLCPP_WARN(
                        logger,
                        "Audio cue '%s' command returned %d: %s",
                        cue_label.c_str(),
                        result,
                        audio_command.c_str());
                }
            }).detach();
    }

    std::string resolve_audio_path(const std::string & audio_file) const
    {
        if (audio_file.empty() || path_is_absolute(audio_file))
        {
            return audio_file;
        }

        if (package_share_directory_.empty())
        {
            RCLCPP_WARN(
                this->get_logger(),
                "Audio file '%s' is relative, but the package share directory is unavailable.",
                audio_file.c_str());
            return "";
        }

        return package_share_directory_ + "/" + audio_file;
    }

    static bool replace_all(
        std::string & text,
        const std::string & placeholder,
        const std::string & replacement)
    {
        bool replaced = false;
        std::string::size_type placeholder_position = text.find(placeholder);
        while (placeholder_position != std::string::npos)
        {
            text.replace(
                placeholder_position,
                placeholder.length(),
                replacement);
            replaced = true;
            placeholder_position =
                text.find(placeholder, placeholder_position + replacement.length());
        }
        return replaced;
    }

    static std::string make_file_uri(const std::string & path)
    {
        std::string uri = "file://";
        for (const char character : path)
        {
            if (character == ' ')
            {
                uri += "%20";
            }
            else if (character == '\'')
            {
                uri += "%27";
            }
            else
            {
                uri += character;
            }
        }
        return uri;
    }

    static bool path_is_absolute(const std::string & path)
    {
        if (path.empty())
        {
            return false;
        }

        if (path.front() == '/' || path.front() == '\\')
        {
            return true;
        }

        return path.size() >= 3 &&
            ((path[0] >= 'A' && path[0] <= 'Z') || (path[0] >= 'a' && path[0] <= 'z')) &&
            path[1] == ':' &&
            (path[2] == '\\' || path[2] == '/');
    }

    static std::string shell_quote(const std::string & value)
    {
        std::string quoted = "'";
        for (const char character : value)
        {
            if (character == '\'')
            {
                quoted += "'\\''";
            }
            else
            {
                quoted += character;
            }
        }
        quoted += "'";
        return quoted;
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

    std::chrono::steady_clock::time_point state_start_time_;
    std::chrono::steady_clock::time_point last_controller_open_attempt_;
    std::chrono::steady_clock::time_point last_awaiting_pickup_audio_time_;
    std::chrono::steady_clock::time_point obstacle_pause_start_time_;
    std::chrono::duration<double> move_duration_{0.0};
    std::string package_share_directory_;
    rclcpp::Time last_scan_time_{0, 0, RCL_ROS_TIME};
    MotionState state_ = MotionState::WaitingForInput;
    bool controller_open_ = false;
    bool a_was_down_ = false;
    bool y_was_down_ = false;
    bool awaiting_pickup_audio_has_played_ = false;
    bool obstacle_in_front_ = false;
    bool previous_obstacle_in_front_ = false;
    bool obstacle_pause_active_ = false;
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
