#include <functional>
#include <memory>
#include <sstream>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"

class FramePublisher : public rclcpp::Node
{
public:
    FramePublisher()
        : Node("tugbot_tf2_frame_publisher")
    {
        // Initialize the transform broadcaster
        tf_broadcaster_ =
            std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        subscription_ = this->create_subscription<geometry_msgs::msg::TransformStamped>(
            "/tugbot/pose", 10,
            std::bind(&FramePublisher::handle_turtle_pose, this, std::placeholders::_1));
    }

private:
    void handle_turtle_pose(const std::shared_ptr<geometry_msgs::msg::TransformStamped> msg)
    {
        if (msg->child_frame_id == "tugbot/wheel_left" || msg->child_frame_id == "tugbot/wheel_right" || msg->child_frame_id == "tugbot/warnign_light")
        {
            geometry_msgs::msg::TransformStamped t(*msg);
            t.header.frame_id = "base_link";
            t.child_frame_id = msg->child_frame_id.substr(7);
            tf_broadcaster_->sendTransform(t);
        }
    }

    rclcpp::Subscription<geometry_msgs::msg::TransformStamped>::SharedPtr subscription_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::string turtlename_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FramePublisher>());
    rclcpp::shutdown();
    return 0;
}