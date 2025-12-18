#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

class MapOdomTFPublisherStatic : public rclcpp::Node
{
public:
    MapOdomTFPublisherStatic()
        : Node("map_odom_tf_static_node")
    {
        tf_publisher_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    }

    void publish_tf()
    {
        geometry_msgs::msg::TransformStamped tf_msg;
        
        tf_msg.header.frame_id = "map";
        tf_msg.header.stamp = this->get_clock()->now();
        tf_msg.child_frame_id = "odom";

        // 모든 값 0
        tf_msg.transform.translation.x = 0.0;
        tf_msg.transform.translation.y = 0.0;  
        tf_msg.transform.translation.z = 0.0;

        tf_msg.transform.rotation.x = 0.0;
        tf_msg.transform.rotation.y = 0.0;
        tf_msg.transform.rotation.z = 0.0;
        tf_msg.transform.rotation.w = 1.0;
        // cout << "Publishing static transform from 'map' to 'odom'" << endl;
        tf_publisher_->sendTransform(tf_msg);
    }

private:
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_publisher_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MapOdomTFPublisherStatic>();

    rclcpp::Rate rate(10);
    while (rclcpp::ok())
    {
        node->publish_tf();
        rclcpp::spin_some(node);
        rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}
