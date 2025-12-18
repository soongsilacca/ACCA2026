#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

class OdomBaseLinkTFPublisher : public rclcpp::Node
{
public:
    OdomBaseLinkTFPublisher()
        : Node("odom_baselink_tf_publisher")
    {
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "localization/kinematic_state/rotated",
            10,
            std::bind(&OdomBaseLinkTFPublisher::odomCallback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Started odom → base_link TF publisher (no origin offset).");
    }

private:
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        geometry_msgs::msg::TransformStamped tf_msg;

        tf_msg.header.stamp = msg->header.stamp;
        tf_msg.header.frame_id = "map";
        tf_msg.child_frame_id = "base_link_ys";

        tf_msg.transform.translation.x = msg->pose.pose.position.x;
        tf_msg.transform.translation.y = msg->pose.pose.position.y;
        tf_msg.transform.translation.z = msg->pose.pose.position.z;

        tf_msg.transform.rotation = msg->pose.pose.orientation;

        tf_broadcaster_->sendTransform(tf_msg);
    }

    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OdomBaseLinkTFPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
