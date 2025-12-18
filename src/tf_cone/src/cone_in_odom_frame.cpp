#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <nav_msgs/msg/odometry.hpp>
#include <unordered_set>
#include <cmath>

struct PointHash {
    size_t operator()(const std::pair<float, float>& p) const {
        return std::hash<float>()(p.first) ^ std::hash<float>()(p.second);
    }
};

class PointTransformerNode : public rclcpp::Node {
public:
    PointTransformerNode() 
        : Node("point_transformer_node"), 
          tf_buffer_(this->get_clock()), 
          tf_listener_(tf_buffer_) 
    {
        yellow_subscriber_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
            "point/yellow", 10,
            std::bind(&PointTransformerNode::yellowCallback, this, std::placeholders::_1));
        blue_subscriber_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
            "point/blue", 10,
            std::bind(&PointTransformerNode::blueCallback, this, std::placeholders::_1));

        yellow_odom_publisher_ = this->create_publisher<geometry_msgs::msg::PointStamped>("yellow_odom", 10);
        blue_odom_publisher_ = this->create_publisher<geometry_msgs::msg::PointStamped>("blue_odom", 10);

        yellow_count_ = 0;
        blue_count_ = 0;
    }

private:
    void yellowCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
        try 
        {
            geometry_msgs::msg::TransformStamped transform_stamped;
            transform_stamped = tf_buffer_.lookupTransform("odom", msg->header.frame_id, tf2::TimePointZero);
            
            geometry_msgs::msg::PointStamped transformed_point;
            tf2::doTransform(*msg, transformed_point, transform_stamped);

            transformed_point.header.frame_id = "odom";
            transformed_point.header.stamp = this->get_clock()->now();

            std::pair<float, float> new_yellow = {transformed_point.point.x, transformed_point.point.y};
            if (isNewPoint(yellow_set_, new_yellow, 0.5))
            {
                yellow_count_++;
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "yellow: %d", yellow_count_);
                yellow_set_.insert(new_yellow);
                yellow_odom_publisher_->publish(transformed_point);
            }
        } 
        catch (tf2::TransformException &ex) 
        {
            RCLCPP_WARN(this->get_logger(), "Could not transform yellow point: %s", ex.what());
        }
    }

    void blueCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
        try 
        {
            geometry_msgs::msg::TransformStamped transform_stamped;
            transform_stamped = tf_buffer_.lookupTransform("odom", msg->header.frame_id, tf2::TimePointZero);
            
            geometry_msgs::msg::PointStamped transformed_point;
            tf2::doTransform(*msg, transformed_point, transform_stamped);

            transformed_point.header.frame_id = "odom";
            transformed_point.header.stamp = this->get_clock()->now();

            std::pair<float, float> new_blue = {transformed_point.point.x, transformed_point.point.y};
            if (isNewPoint(blue_set_, new_blue, 0.5))
            {
                blue_count_++;
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "blue: %d", blue_count_);
                blue_set_.insert(new_blue);
                blue_odom_publisher_->publish(transformed_point);
            }
        } 
        catch (tf2::TransformException &ex) 
        {
            RCLCPP_WARN(this->get_logger(), "Could not transform blue point: %s", ex.what());
        }
    }

    bool isNewPoint(const std::unordered_set<std::pair<float, float>, PointHash>& points, 
                    const std::pair<float, float>& new_point, float threshold)
    {
        for (const auto& point : points)
        {
            float distance = std::hypot(point.first - new_point.first, point.second - new_point.second);
            if (distance < threshold) return false;
        }
        return true;
    }

    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr yellow_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr blue_subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr yellow_odom_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr blue_odom_publisher_;
    
    std::unordered_set<std::pair<float, float>, PointHash> yellow_set_;
    std::unordered_set<std::pair<float, float>, PointHash> blue_set_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    int blue_count_;
    int yellow_count_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PointTransformerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
