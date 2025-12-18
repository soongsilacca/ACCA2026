#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

class ConePosesTransformerNode : public rclcpp::Node {
public:
    ConePosesTransformerNode() : Node("cone_poses_transformer_node"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_) {
        cone_poses_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
            "/cone_poses", 10,
            std::bind(&ConePosesTransformerNode::conePosesCallback, this, std::placeholders::_1));

        cone_poses_odom_publisher_ = this->create_publisher<geometry_msgs::msg::PoseArray>("/cone_poses_odom", 10);

        velodyne_points_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/velodyne_points", 10,
            std::bind(&ConePosesTransformerNode::velodynePointsCallback, this, std::placeholders::_1));

        velodyne_points_odom_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/velodyne_points2", 10);
    }

private:
    void conePosesCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg) {
        geometry_msgs::msg::TransformStamped transform_stamped;
        try {
            transform_stamped = tf_buffer_.lookupTransform("odom", msg->header.frame_id, tf2::TimePointZero);

            geometry_msgs::msg::PoseArray transformed_poses;
            transformed_poses.header.frame_id = "odom";
            transformed_poses.header.stamp = msg->header.stamp;

            for (const auto& pose : msg->poses) {
                geometry_msgs::msg::PoseStamped pose_stamped, transformed_pose_stamped;
                pose_stamped.pose = pose;
                pose_stamped.header = msg->header;

                tf2::doTransform(pose_stamped, transformed_pose_stamped, transform_stamped);

                transformed_poses.poses.push_back(transformed_pose_stamped.pose);
            }

            cone_poses_odom_publisher_->publish(transformed_poses);
        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "Could not transform cone poses: %s", ex.what());
        }
    }

    void velodynePointsCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        sensor_msgs::msg::PointCloud2 transformed_cloud;
        try {
            geometry_msgs::msg::TransformStamped transform_stamped = tf_buffer_.lookupTransform("odom", msg->header.frame_id, tf2::TimePointZero);
            
            tf2::doTransform(*msg, transformed_cloud, transform_stamped);

            transformed_cloud.header.frame_id = "odom";
            transformed_cloud.header.stamp = msg->header.stamp;

            velodyne_points_odom_publisher_->publish(transformed_cloud);
        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "Could not transform Velodyne points: %s", ex.what());
        }
    }

    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr cone_poses_subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr cone_poses_odom_publisher_;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr velodyne_points_subscriber_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr velodyne_points_odom_publisher_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ConePosesTransformerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
