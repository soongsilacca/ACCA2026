#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

struct OdomFramePoint {
    float x;
    float y;
    int yellow_count;
    int blue_count;
};

class PointTransformerNode : public rclcpp::Node {
public:
    PointTransformerNode() : Node("point_transformer_node"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_) {
        yellow_subscriber_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
            "point/yellow", 10,
            std::bind(&PointTransformerNode::yellowCallback, this, std::placeholders::_1));
        blue_subscriber_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
            "point/blue", 10,
            std::bind(&PointTransformerNode::blueCallback, this, std::placeholders::_1));

        yellow_odom_publisher_ = this->create_publisher<geometry_msgs::msg::PointStamped>("yellow_odom", 10);
        blue_odom_publisher_ = this->create_publisher<geometry_msgs::msg::PointStamped>("blue_odom", 10);
    }

private:
    void yellowCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
        transformAndProcessPoint(msg, true);
    }

    void blueCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
        transformAndProcessPoint(msg, false);
    }

    void transformAndProcessPoint(const geometry_msgs::msg::PointStamped::SharedPtr& msg, bool is_yellow) {
        geometry_msgs::msg::TransformStamped transform_stamped;
        try {
            transform_stamped = tf_buffer_.lookupTransform("odom", msg->header.frame_id, msg->header.stamp);

            geometry_msgs::msg::PointStamped transformed_point;
            tf2::doTransform(*msg, transformed_point, transform_stamped);

            transformed_point.header.frame_id = "odom";
            transformed_point.header.stamp = msg->header.stamp;

            processTransformedPoint(transformed_point, is_yellow);
        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "Could not transform point: %s", ex.what());
        }
    }

    void processTransformedPoint(const geometry_msgs::msg::PointStamped& transformed_point, bool is_yellow) {
        std::vector<float> new_point = {transformed_point.point.x, transformed_point.point.y};
        bool point_updated = false;

        for (auto& point : odom_frame_points_) 
        {
            float distance = std::sqrt(std::pow(point.x - new_point[0], 2) + std::pow(point.y - new_point[1], 2));
            if (distance < 0.3) 
            {

                if (is_yellow)
                {
                    point.yellow_count++;
                    if (point.yellow_count == 50) 
                    {
                        yellow_odom_publisher_->publish(transformed_point);
                    }
                } 
                else 
                {
                    point.blue_count++;
                    if (point.blue_count == 50) 
                    {
                        blue_odom_publisher_->publish(transformed_point);
                    }
                }
                point_updated = true;
                break;
            }
        }

        if (!point_updated) {
            OdomFramePoint new_odom_point = {new_point[0], new_point[1], 0, 0};
            if (is_yellow) {
                new_odom_point.yellow_count = 1;
            } else {
                new_odom_point.blue_count = 1;
            }
            odom_frame_points_.push_back(new_odom_point);
        }
    }

    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr yellow_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr blue_subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr yellow_odom_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr blue_odom_publisher_;

    std::vector<OdomFramePoint> odom_frame_points_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PointTransformerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
