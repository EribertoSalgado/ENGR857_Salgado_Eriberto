// Half Speed, X to kill, LED Direction Control

#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/color_rgba.hpp>

#include "quanser/quanser_messages.h"
#include "quanser/quanser_memory.h"
#include "std_msgs/msg/header.hpp"
#include "quanser/quanser_hid.h"

using namespace std::chrono_literals;

bool node_running = false;

// joystick inputs
t_double LLA = 0.0;
t_double RT  = 0.0;
t_double A  = 0;
t_double X  = 0;
t_double LB = 0.0;
t_double RB = 0.0;
t_double throttle;
t_double steering;

// joystick definition
t_game_controller gamepad;
t_error result;
t_uint8 controller_number = 1;
t_uint16 buffer_size   = 12;
t_double deadzone[6]   = {0.0};
t_double saturation[6] = {0.0};
t_boolean auto_center  = false;
t_uint16 max_force_feedback_effects = 0;
t_double force_feedback_gain = 0.0;
t_game_controller_states data;
t_boolean is_new;


class CommandPublisher : public rclcpp::Node
{
public:
    CommandPublisher()
    : Node("joystick_publisher")
    {
        // Velocity publisher
        command_publisher_ =
            this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

        // ✅ LED publisher (correct topic + type)
        led_publisher_ =
            this->create_publisher<std_msgs::msg::ColorRGBA>("/qbot_led_strip", 10);

        result = game_controller_open(controller_number, buffer_size,
                     deadzone, saturation, auto_center,
                     max_force_feedback_effects,
                     force_feedback_gain, &gamepad);
        auto timer_callback =
        [this]() -> void {
        if (result >= 0)
        {
            while (rclcpp::ok())
            {
                result = game_controller_poll(gamepad, &data, &is_new);

                LLA = -1 * data.x;
                RT  = data.rz;

                A  = (t_uint8)(data.buttons & (1 << 0));
                X  = (t_uint8)((data.buttons & (1 << 2)) / 4);
                LB = (t_uint8)((data.buttons & (1 << 4)) / 16);
                RB = (t_uint8)((data.buttons & (1 << 5)) / 32);

                // ✅ Kill switch
                if (X == 1)
                {
                    geometry_msgs::msg::Twist stop_msg;
                    stop_msg.linear.x = 0;
                    stop_msg.angular.z = 0;
                    command_publisher_->publish(stop_msg);

                    game_controller_close(gamepad);
                    rclcpp::shutdown();
                    return;
                }
		//Testing for Motion
		i = 0;  
		do{
		throttle = 1; 
		i++; 
		}while(i != 10 

                // Motion control
                if (LB == 1)
                {
                    if (RT == 0)
                        throttle = 0;
                    else
                        throttle = 0.3 * (0.5 + 0.5 * RT);

                    steering = 0.5 * LLA;

                    if (A == 1)
                        throttle = -throttle;

                    if (RB == 1)
                        throttle = 0.5 * throttle;
                }
                else
                {
                    throttle = 0;
                    steering = 0;
                }

                // Publish velocity
                geometry_msgs::msg::Twist twist;
                twist.linear.x = throttle;
                twist.angular.z = steering;
                command_publisher_->publish(twist);

                // ✅ LED color logic
                std_msgs::msg::ColorRGBA led_msg;

                if (throttle > 0.01)   // Forward → Green
                {
                    led_msg.r = 0.0;
                    led_msg.g = 1.0;
                    led_msg.b = 0.0;
                    led_msg.a = 1.0;
                }
                else if (throttle < -0.01)  // Backward → Blue
                {
                    led_msg.r = 0.0;
                    led_msg.g = 0.0;
                    led_msg.b = 1.0;
                    led_msg.a = 1.0;
                }
                else   // Stationary → Yellow
                {
                    led_msg.r = 1.0;
                    led_msg.g = 1.0;
                    led_msg.b = 0.0;
                    led_msg.a = 1.0;
                }

                led_publisher_->publish(led_msg);
            }
        }

        game_controller_close(gamepad);
        };

        timer_ = this->create_wall_timer(100ms, timer_callback);
    };

private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr command_publisher_;
    rclcpp::Publisher<std_msgs::msg::ColorRGBA>::SharedPtr led_publisher_;
};


int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CommandPublisher>());
    rclcpp::shutdown();
    return 0;
}
