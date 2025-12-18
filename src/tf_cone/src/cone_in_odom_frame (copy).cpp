// #include "rclcpp/rclcpp.hpp"
// #include "geometry_msgs/msg/point_stamped.hpp"
// #include <tf2_ros/transform_listener.h>
// #include <tf2_ros/buffer.h>
// #include <tf2_geometry_msgs/tf2_geometry_msgs.h>
// #include <nav_msgs/msg/odometry.hpp>

// class PointTransformerNode : public rclcpp::Node {
// public:
//     PointTransformerNode() 
//         : Node("point_transformer_node"), 
//           tf_buffer_(this->get_clock()), 
//           tf_listener_(tf_buffer_) 
//     {
//         yellow_subscriber_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
//             "point/yellow", 10,
//             std::bind(&PointTransformerNode::yellowCallback, this, std::placeholders::_1));
//         blue_subscriber_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
//             "point/blue", 10,
//             std::bind(&PointTransformerNode::blueCallback, this, std::placeholders::_1));

//         // odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
//         //     "/odometry/kalman", 10, std::bind(&PointTransformerNode::callback_odom, this, std::placeholders::_1));

//         yellow_odom_publisher_ = this->create_publisher<geometry_msgs::msg::PointStamped>("yellow_odom", 10);
//         blue_odom_publisher_ = this->create_publisher<geometry_msgs::msg::PointStamped>("blue_odom", 10);

//         yellow_count_ = 0;
//         blue_count_ = 0;
//         // odom_ = nullptr;
//     }

// private:

//     // geometry_msgs::msg::TransformStamped getTF(const rclcpp::Time& stamp) {
//     //     geometry_msgs::msg::TransformStamped tf_msg;

//     //     tf_msg.header.frame_id = "odom";
//     //     tf_msg.header.stamp = stamp;
//     //     tf_msg.child_frame_id = "base_link";
//     //     if (odom_) {
//     //         tf_msg.transform.translation.x = odom_->pose.pose.position.x;
//     //         tf_msg.transform.translation.y = odom_->pose.pose.position.y;
//     //         tf_msg.transform.translation.z = 0.0;

//     //         tf_msg.transform.rotation = odom_->pose.pose.orientation;
//     //     } else {
//     //         RCLCPP_WARN(this->get_logger(), "Odometry data is not available.");
//     //     }

//     //     return tf_msg;
//     // }


//     // void callback_odom(const nav_msgs::msg::Odometry::SharedPtr msg)
//     // {
//     //     odom_ = msg;
//     // }

//     void yellowCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
//         geometry_msgs::msg::TransformStamped transform_stamped;
//         try 
//         {
//             // transform_stamped = tf_buffer_.lookupTransform("odom", msg->header.frame_id, msg->header.stamp);
//             transform_stamped = tf_buffer_.lookupTransform("odom", msg->header.frame_id, tf2::TimePointZero);
//             // transform_stamped = getTF(msg->header.stamp);

            
//             geometry_msgs::msg::PointStamped transformed_point;
//             tf2::doTransform(*msg, transformed_point, transform_stamped);

//             transformed_point.header.frame_id = "odom";
//             transformed_point.header.stamp = this->get_clock()->now();

//             std::vector<float> new_yellow = {transformed_point.point.x, transformed_point.point.y};
//             if (isYellowNewPoint(yellow_vector_, new_yellow, 0.5))
//             {
//                 yellow_count_++;
//                 std::cout << "yellow: " << yellow_count_ << std::endl;
//                 yellow_vector_.push_back(new_yellow);
//                 yellow_odom_publisher_->publish(transformed_point);
//             }
//         } 
//         catch (tf2::TransformException &ex) 
//         {
//             RCLCPP_WARN(this->get_logger(), "Could not transform yellow point: %s", ex.what());
//         }
//     }

//     void blueCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
//         geometry_msgs::msg::TransformStamped transform_stamped;
//         try 
//         {
//             // transform_stamped = tf_buffer_.lookupTransform("odom", msg->header.frame_id, msg->header.stamp);
//             transform_stamped = tf_buffer_.lookupTransform("odom", msg->header.frame_id, tf2::TimePointZero);
//             // transform_stamped = getTF(msg->header.stamp);

            
//             geometry_msgs::msg::PointStamped transformed_point;
//             tf2::doTransform(*msg, transformed_point, transform_stamped);

//             transformed_point.header.frame_id = "odom";
//             transformed_point.header.stamp = this->get_clock()->now();

//             std::vector<float> new_blue = {transformed_point.point.x, transformed_point.point.y};
//             if (isBlueNewPoint(blue_vector_, new_blue, 0.5))
//             {
//                 blue_count_++;
//                 std::cout << "blue: " << blue_count_ << std::endl;
//                 blue_vector_.push_back(new_blue);
//                 blue_odom_publisher_->publish(transformed_point);
//             }
//         } 
//         catch (tf2::TransformException &ex) 
//         {
//             RCLCPP_WARN(this->get_logger(), "Could not transform blue point: %s", ex.what());
//         }
//     }

//     bool isYellowNewPoint(const std::vector<std::vector<float>>& points, const std::vector<float>& new_point, float threshold)
//     {
//         for (const auto& point : points)
//         {
//             float distance = std::sqrt(std::pow(point[0] - new_point[0], 2) + std::pow(point[1] - new_point[1], 2));
//             if (distance < threshold)
//             {
//                 return false;
//             }
//         }
//         return true;
//     }

//     bool isBlueNewPoint(const std::vector<std::vector<float>>& points, const std::vector<float>& new_point, float threshold)
//     {
//         for (const auto& point : points)
//         {   
//             float distance = std::sqrt(std::pow(point[0] - new_point[0], 2) + std::pow(point[1] - new_point[1], 2));
//             if (distance < threshold)
//             {
//                 return false;
//             }
//         }
//         return true;
//     }

//     rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr yellow_subscriber_;
//     rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr blue_subscriber_;
//     rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
//     rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr yellow_odom_publisher_;
//     rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr blue_odom_publisher_;
//     nav_msgs::msg::Odometry::SharedPtr odom_;
    
//     std::vector<std::vector<float>> yellow_vector_;
//     std::vector<std::vector<float>> blue_vector_;

//     tf2_ros::Buffer tf_buffer_;
//     tf2_ros::TransformListener tf_listener_;

//     int blue_count_;
//     int yellow_count_;
// };

// int main(int argc, char *argv[]) {
//     rclcpp::init(argc, argv);
//     auto node = std::make_shared<PointTransformerNode>();
//     rclcpp::spin(node);
//     rclcpp::shutdown();
//     return 0;
// }



// #include "rclcpp/rclcpp.hpp"
// #include "geometry_msgs/msg/point_stamped.hpp"
// #include "nav_msgs/msg/odometry.hpp"
// #include <cmath>

// class PointTransformerNode : public rclcpp::Node {
// public:
//     PointTransformerNode() 
//         : Node("point_transformer_node")
//     {
//         // 구독자 설정
//         yellow_subscriber_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
//             "point/yellow", 10,
//             std::bind(&PointTransformerNode::yellowCallback, this, std::placeholders::_1));
//         blue_subscriber_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
//             "point/blue", 10,
//             std::bind(&PointTransformerNode::blueCallback, this, std::placeholders::_1));

//         odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
//             "/odometry/kalman", 10, 
//             std::bind(&PointTransformerNode::odomCallback, this, std::placeholders::_1));

//         // 퍼블리셔 설정
//         yellow_odom_publisher_ = this->create_publisher<geometry_msgs::msg::PointStamped>("yellow_odom", 10);
//         blue_odom_publisher_ = this->create_publisher<geometry_msgs::msg::PointStamped>("blue_odom", 10);

//         yellow_count_ = 0;
//         blue_count_ = 0;
//     }

// private:
//     // Odometry 콜백 함수
//     void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
//         odom_x_ = msg->pose.pose.position.x;
//         odom_y_ = msg->pose.pose.position.y;

//         // 오일러 각으로 변환하여 Yaw 추출
//         double siny_cosp = 2 * (msg->pose.pose.orientation.w * msg->pose.pose.orientation.z + msg->pose.pose.orientation.x * msg->pose.pose.orientation.y);
//         double cosy_cosp = 1 - 2 * (msg->pose.pose.orientation.y * msg->pose.pose.orientation.y + msg->pose.pose.orientation.z * msg->pose.pose.orientation.z);
//         odom_yaw_ = std::atan2(siny_cosp, cosy_cosp);
//     }

//     // Yellow Point 콜백 함수
//     void yellowCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
//         geometry_msgs::msg::PointStamped transformed_point;
//         transformPointToOdom(msg, transformed_point);
//         std::vector<float> new_yellow = {transformed_point.point.x, transformed_point.point.y};
//         if (isYellowNewPoint(yellow_vector_, new_yellow, 0.5))
//             {
//                 yellow_count_++;
//                 std::cout << "yellow: " << yellow_count_ << std::endl;
//                 yellow_vector_.push_back(new_yellow);
//                 yellow_odom_publisher_->publish(transformed_point);
//             }
//     }

//     // Blue Point 콜백 함수
//     void blueCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
//         geometry_msgs::msg::PointStamped transformed_point;
//         transformPointToOdom(msg, transformed_point);
//         std::vector<float> new_blue = {transformed_point.point.x, transformed_point.point.y};
//         if (isBlueNewPoint(blue_vector_, new_blue, 0.5))
//             {
//                 blue_count_++;
//                 std::cout << "blue: " << blue_count_ << std::endl;
//                 blue_vector_.push_back(new_blue);
//                 blue_odom_publisher_->publish(transformed_point);
//             }
//     }

//     // Point를 Odom 프레임으로 변환
//     void transformPointToOdom(const geometry_msgs::msg::PointStamped::SharedPtr& input_point, geometry_msgs::msg::PointStamped& output_point) {
//         // base_link에서의 포인트 좌표
//         double x_base_link = input_point->point.x;
//         double y_base_link = input_point->point.y;

//         // odom 프레임으로 변환
//         double x_odom = odom_x_ + std::cos(odom_yaw_) * x_base_link - std::sin(odom_yaw_) * y_base_link;
//         double y_odom = odom_y_ + std::sin(odom_yaw_) * x_base_link + std::cos(odom_yaw_) * y_base_link;

//         output_point.header.stamp = this->get_clock()->now();
//         output_point.header.frame_id = "odom";
//         output_point.point.x = x_odom;
//         output_point.point.y = y_odom;
//         output_point.point.z = 0.0;
//     }

//     bool isYellowNewPoint(const std::vector<std::vector<float>>& points, const std::vector<float>& new_point, float threshold)
//     {
//         for (const auto& point : points)
//         {
//             float distance = std::sqrt(std::pow(point[0] - new_point[0], 2) + std::pow(point[1] - new_point[1], 2));
//             if (distance < threshold)
//             {
//                 return false;
//             }
//         }
//         return true;
//     }

//     bool isBlueNewPoint(const std::vector<std::vector<float>>& points, const std::vector<float>& new_point, float threshold)
//     {
//         for (const auto& point : points)
//         {   
//             float distance = std::sqrt(std::pow(point[0] - new_point[0], 2) + std::pow(point[1] - new_point[1], 2));
//             if (distance < threshold)
//             {
//                 return false;
//             }
//         }
//         return true;
//     }

//     // 멤버 변수들
//     rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr yellow_subscriber_;
//     rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr blue_subscriber_;
//     rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
//     rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr yellow_odom_publisher_;
//     rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr blue_odom_publisher_;

//     double odom_x_ = 0.0;
//     double odom_y_ = 0.0;
//     double odom_yaw_ = 0.0;



//     std::vector<std::vector<float>> yellow_vector_;
//     std::vector<std::vector<float>> blue_vector_;
//     int blue_count_;
//     int yellow_count_;
// };

// int main(int argc, char *argv[]) {
//     rclcpp::init(argc, argv);
//     auto node = std::make_shared<PointTransformerNode>();
//     rclcpp::spin(node);
//     rclcpp::shutdown();
//     return 0;
// }
