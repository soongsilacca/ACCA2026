#include <memory>
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <Eigen/Geometry>

class MapOdomTFPublisher : public rclcpp::Node
{
public:
    MapOdomTFPublisher()
        : Node("map_odom_tf_node_2"), initialized_(false), twice_(false)
    {
        tf_publisher_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        
        // 오도메트리 데이터 구독
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "odometry/wheel", 10, std::bind(&MapOdomTFPublisher::callback_odom, this, std::placeholders::_1));
    }

    void publish_tf()
    {
        if (!initialized_ || twice_)
            return;

        geometry_msgs::msg::TransformStamped tf_msg;

        tf_msg.header.frame_id = "map";
        tf_msg.header.stamp = odom_->header.stamp;
        tf_msg.child_frame_id = "odom";

        // odom의 초기 위치를 map의 원점으로 설정한 후 계속 유지
        tf_msg.transform.translation.x = 0.0;
        tf_msg.transform.translation.y = 0.0;
        tf_msg.transform.translation.z = 0.0;

        tf_msg.transform.rotation.x = 0.0;
        tf_msg.transform.rotation.y = 0.0;
        tf_msg.transform.rotation.z = 0.0;
        tf_msg.transform.rotation.w = 1.0;
        // twice_ = true;
        tf_publisher_->sendTransform(tf_msg);
    }

private:
    void callback_odom(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        if (!initialized_)
        {
            // 초기 위치 저장 (map 기준)
            initial_x_ = msg->pose.pose.position.x;
            initial_y_ = msg->pose.pose.position.y;

            // 초기 orientation 저장
            initial_qx_ = msg->pose.pose.orientation.x;
            initial_qy_ = msg->pose.pose.orientation.y;
            initial_qz_ = msg->pose.pose.orientation.z;
            initial_qw_ = msg->pose.pose.orientation.w;
            
            initialized_ = true;
            
            RCLCPP_INFO(this->get_logger(), "Initialized map->odom transformation.");
        }

        odom_ = msg;
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_publisher_;
    nav_msgs::msg::Odometry::SharedPtr odom_;

    bool initialized_;
    bool twice_;
    double initial_x_, initial_y_;
    double initial_qx_, initial_qy_, initial_qz_, initial_qw_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MapOdomTFPublisher>();

    rclcpp::Rate rate(10);
    while (rclcpp::ok())
    {
        rclcpp::spin_some(node);
        node->publish_tf();
        rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}
