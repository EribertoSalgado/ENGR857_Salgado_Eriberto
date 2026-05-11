#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <apriltag_msgs/msg/april_tag_detection_array.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

class AprilTagPosePublisher : public rclcpp::Node
{
public:
    AprilTagPosePublisher()
    : Node("apriltag_pose_publisher")
    {
        pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "apriltag_pose", 10);

        detection_subscriber_ = this->create_subscription<apriltag_msgs::msg::AprilTagDetectionArray>(
            "tag_detections", 10,
            std::bind(&AprilTagPosePublisher::detection_callback, this, std::placeholders::_1));

        // Known tag positions (example: tag 0 at origin)
        tag_poses_[0] = create_pose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);  // Adjust as needed
        // Add more tags as needed
    }

private:
    void detection_callback(const apriltag_msgs::msg::AprilTagDetectionArray::SharedPtr msg)
    {
        if (msg->detections.empty())
        {
            return;
        }

        // Use the first detected tag for simplicity
        const auto& detection = msg->detections[0];
        int tag_id = detection.id;

        if (tag_poses_.find(tag_id) == tag_poses_.end())
        {
            RCLCPP_WARN(this->get_logger(), "Unknown tag ID: %d", tag_id);
            return;
        }

        // Tag pose in camera frame
        tf2::Transform tag_in_camera;
        tf2::fromMsg(detection.pose.pose.pose, tag_in_camera);

        // Known tag pose in map frame
        tf2::Transform tag_in_map;
        tf2::fromMsg(tag_poses_[tag_id], tag_in_map);

        // Camera to base_link transform (assume known or from TF)
        // For simplicity, assume camera is on base_link with known transform
        tf2::Transform camera_to_base;
        camera_to_base.setOrigin(tf2::Vector3(0.1, 0.0, 0.2));  // Example: camera 10cm forward, 20cm up
        camera_to_base.setRotation(tf2::Quaternion(0.0, 0.0, 0.0, 1.0));  // No rotation

        // Compute base_link in map: map_T_base = map_T_tag * tag_T_camera * camera_T_base
        tf2::Transform map_to_base = tag_in_map * tag_in_camera.inverse() * camera_to_base.inverse();

        geometry_msgs::msg::PoseWithCovarianceStamped pose_msg;
        pose_msg.header.stamp = this->now();
        pose_msg.header.frame_id = "map";
        tf2::toMsg(map_to_base, pose_msg.pose.pose);

        // Set covariance (example values)
        pose_msg.pose.covariance = {
            0.1, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.1, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.1, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.1, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.1, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.1
        };

        pose_publisher_->publish(pose_msg);
    }

    geometry_msgs::msg::Pose create_pose(double x, double y, double z, double qx, double qy, double qz, double qw)
    {
        geometry_msgs::msg::Pose pose;
        pose.position.x = x;
        pose.position.y = y;
        pose.position.z = z;
        pose.orientation.x = qx;
        pose.orientation.y = qy;
        pose.orientation.z = qz;
        pose.orientation.w = qw;
        return pose;
    }

    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_publisher_;
    rclcpp::Subscription<apriltag_msgs::msg::AprilTagDetectionArray>::SharedPtr detection_subscriber_;
    std::map<int, geometry_msgs::msg::Pose> tag_poses_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AprilTagPosePublisher>());
    rclcpp::shutdown();
    return 0;
}