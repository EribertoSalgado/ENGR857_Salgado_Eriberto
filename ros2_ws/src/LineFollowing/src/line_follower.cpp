#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.hpp"
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

class LineFollower : public rclcpp::Node
{
public:
  LineFollower()
  : Node("line_follower")
  {
    this->declare_parameter<std::string>("image_topic", "/downward_camera/image_raw");
    this->declare_parameter<double>("linear_speed", 0.10);
    this->declare_parameter<double>("angular_gain", 0.005);
    this->declare_parameter<int>("threshold", 100);
    this->declare_parameter<int>("min_contour_area", 200);

    image_topic_ = this->get_parameter("image_topic").as_string();
    linear_speed_ = this->get_parameter("linear_speed").as_double();
    angular_gain_ = this->get_parameter("angular_gain").as_double();
    threshold_value_ = this->get_parameter("threshold").as_int();
    min_contour_area_ = this->get_parameter("min_contour_area").as_int();

    command_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    image_subscriber_ = image_transport::create_subscription(
      this,
      image_topic_,
      std::bind(&LineFollower::image_callback, this, std::placeholders::_1),
      "raw");

    RCLCPP_INFO(this->get_logger(), "Line follower started on '%s'.", image_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "Using linear_speed=%.3f, angular_gain=%.5f, threshold=%d.",
      linear_speed_, angular_gain_, threshold_value_);
  }

private:
  void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
  {
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (const cv_bridge::Exception & e)
    {
      RCLCPP_WARN(this->get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }

    cv::Mat gray;
    cv::cvtColor(cv_ptr->image, gray, cv::COLOR_BGR2GRAY);

    cv::Mat mask;
    cv::threshold(gray, mask, threshold_value_, 255, cv::THRESH_BINARY_INV);
    cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, cv::Mat::ones(5, 5, CV_8U));

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    double best_cx = 0.0;
    int best_area = 0;
    bool found_line = false;

    for (const auto & contour : contours)
    {
      const double area = cv::contourArea(contour);
      if (area < min_contour_area_)
      {
        continue;
      }

      if (area > best_area)
      {
        const cv::Moments moments = cv::moments(contour);
        if (moments.m00 > 0.0)
        {
          best_area = static_cast<int>(area);
          best_cx = moments.m10 / moments.m00;
          found_line = true;
        }
      }
    }

    geometry_msgs::msg::Twist twist;
    if (found_line)
    {
      const double error = best_cx - (gray.cols / 2.0);
      twist.linear.x = linear_speed_;
      twist.angular.z = -angular_gain_ * error;
    }
    else
    {
      twist.linear.x = 0.0;
      twist.angular.z = 0.0;
    }

    command_publisher_->publish(twist);
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr command_publisher_;
  image_transport::Subscriber image_subscriber_;
  std::string image_topic_;
  double linear_speed_;
  double angular_gain_;
  int threshold_value_;
  int min_contour_area_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LineFollower>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
