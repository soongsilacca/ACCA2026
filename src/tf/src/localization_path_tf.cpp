#include <memory>
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

class LocalizationAdjuster : public rclcpp::Node
{
public:
    LocalizationAdjuster()
        : Node("localization_adjuster_node")

    {
        // /global_path topic subscription
        global_path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            "/global_path", 10, std::bind(&LocalizationAdjuster::callback_global_path, this, std::placeholders::_1));

        // /localization/kinematic_state topic subscription
        kinematic_state_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/localization/kinematic_state", 10, std::bind(&LocalizationAdjuster::callback_kinematic_state, this, std::placeholders::_1));

        // /localization/kinematic_state2 topic publisher
        kinematic_state2_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/localization/kinematic_state2", 10);

        dx_ = 0.0;
        dy_ = 0.0;
    }

private:
    void callback_global_path(const nav_msgs::msg::Path::SharedPtr msg)
    {
        if (dx_ == 0. && dy_ == 0.)
        {
            first_path_pose_ = msg->poses[0];
        }
    }

    void callback_kinematic_state(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        if (dx_ == 0. && dy_ == 0.)
        {
            // Calculate position difference
            dx_ = first_path_pose_.pose.position.x - msg->pose.pose.position.x;
            dy_ = first_path_pose_.pose.position.y - msg->pose.pose.position.y;
        }
        // Create new odometry message
        auto new_odom_msg = *msg;  // Start with the received messages
        new_odom_msg.pose.pose.position.x += dx_;
        new_odom_msg.pose.pose.position.y += dy_;

        // Publish the adjusted kinematic state
        kinematic_state2_pub_->publish(new_odom_msg);
    }


    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr global_path_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr kinematic_state_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr kinematic_state2_pub_;

    geometry_msgs::msg::PoseStamped first_path_pose_;

    double dx_;
    double dy_;

};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LocalizationAdjuster>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
