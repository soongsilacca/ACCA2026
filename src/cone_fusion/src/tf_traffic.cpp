#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <yolo_msg/msg/traffic_sign.hpp>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

class TrafficSignTransformer : public rclcpp::Node
{
public:
    TrafficSignTransformer()
        : Node("traffic_sign_transformer"),
          tf_buffer_(this->get_clock()),
          tf_listener_(tf_buffer_)
    {
        // Subscribe to original traffic_sign messages
        traffic_sign_sub_ = this->create_subscription<yolo_msg::msg::TrafficSign>(
            "/traffic_sign", 10,
            std::bind(&TrafficSignTransformer::traffic_sign_callback, this, std::placeholders::_1));

        // Publish transformed traffic signs (in map frame)
        traffic_sign_map_pub_ = this->create_publisher<yolo_msg::msg::TrafficSign>("/traffic_sign_map", 10);

        // For RViz visualization
        traffic_sign_rviz_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/traffic_sign_rviz", 10);
    }

private:
    void traffic_sign_callback(const yolo_msg::msg::TrafficSign::SharedPtr msg)
    {
        geometry_msgs::msg::PoseStamped input_pose, transformed_pose;
        input_pose.header.frame_id = "velodyne";  // original frame
        input_pose.pose = msg->pose;

        yolo_msg::msg::TrafficSign output_msg;
        output_msg.class_id = msg->class_id;

        bool tf_success = false;

        try
        {
            tf_buffer_.transform(input_pose, transformed_pose, "map", tf2::durationFromSec(0.5));
            output_msg.pose = transformed_pose.pose;
            tf_success = true;

            RCLCPP_INFO(this->get_logger(), "[TF OK] Transformed pose (class_id: %d)", msg->class_id);
        }
        catch (tf2::TransformException &ex)
        {
            // Fallback: use original pose with "map" as frame
            RCLCPP_WARN(this->get_logger(),
                "[TF FAIL] Using raw pose. Reason: %s", ex.what());

            output_msg.pose = msg->pose;
        }

        traffic_sign_map_pub_->publish(output_msg);

        // For RViz: always publish PoseStamped in "map" frame
        geometry_msgs::msg::PoseStamped rviz_pose;
        rviz_pose.header.frame_id = "map";
        rviz_pose.pose = output_msg.pose;
        traffic_sign_rviz_pub_->publish(rviz_pose);
    }

    rclcpp::Subscription<yolo_msg::msg::TrafficSign>::SharedPtr traffic_sign_sub_;
    rclcpp::Publisher<yolo_msg::msg::TrafficSign>::SharedPtr traffic_sign_map_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr traffic_sign_rviz_pub_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TrafficSignTransformer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
