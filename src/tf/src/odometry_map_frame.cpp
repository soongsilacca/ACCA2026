#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

class OdometryTransformer : public rclcpp::Node
{
public:
    OdometryTransformer()
        : Node("odometry_transformer"),
          tf_buffer_(this->get_clock()),
          tf_listener_(tf_buffer_)
    {
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odometry/navsat", 10, std::bind(&OdometryTransformer::odom_callback, this, std::placeholders::_1));
        
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/localization/kinematic_state", 10);
    }

private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        geometry_msgs::msg::TransformStamped transform_stamped;
        try
        {
            transform_stamped = tf_buffer_.lookupTransform("map", msg->header.frame_id, tf2::TimePointZero);
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_WARN(this->get_logger(), "Could not transform %s to map: %s", msg->header.frame_id.c_str(), ex.what());
            return;
        }

        nav_msgs::msg::Odometry transformed_msg = *msg;
        transformed_msg.header.frame_id = "map";
        tf2::doTransform(msg->pose.pose, transformed_msg.pose.pose, transform_stamped);

        // Twist 변환
        tf2::Quaternion q(
            transform_stamped.transform.rotation.x,
            transform_stamped.transform.rotation.y,
            transform_stamped.transform.rotation.z,
            transform_stamped.transform.rotation.w);
        tf2::Matrix3x3 m(q);
        tf2::Vector3 linear(msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z);
        tf2::Vector3 angular(msg->twist.twist.angular.x, msg->twist.twist.angular.y, msg->twist.twist.angular.z);

        linear = m * linear;
        angular = m * angular;

        transformed_msg.twist.twist.linear.x = linear.x();
        transformed_msg.twist.twist.linear.y = linear.y();
        transformed_msg.twist.twist.linear.z = linear.z();
        transformed_msg.twist.twist.angular.x = angular.x();
        transformed_msg.twist.twist.angular.y = angular.y();
        transformed_msg.twist.twist.angular.z = angular.z();

        odom_pub_->publish(transformed_msg);
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OdometryTransformer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
