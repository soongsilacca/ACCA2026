#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "yolo_msg/msg/bounding_box.hpp"
#include "yolo_msg/msg/traffic_sign.hpp" // 커스텀 메시지
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <unordered_map>

class TrafficSignChecker : public rclcpp::Node
{
public:
    TrafficSignChecker()
    : Node("traffic_sign_checker")
    {
        pose_sub_ = create_subscription<geometry_msgs::msg::PoseArray>(
            "cone_poses", 10, //adaptive_clstering한 포인트클라우드 토픽
            std::bind(&TrafficSignChecker::poseCallback, this, std::placeholders::_1));

        bbox_sub_ = create_subscription<yolo_msg::msg::BoundingBox>(
            "yolo/detections", 10,
            std::bind(&TrafficSignChecker::bboxCallback, this, std::placeholders::_1));

        traffic_pub_ = create_publisher<yolo_msg::msg::TrafficSign>("traffic_sign", 10);
        traffic_rviz_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("traffic_sign_rviz_check", 10);

        // 카메라 내부 파라미터
        C_RTlc_ << 0.0572, -0.9980, 0.0270, 0.0271,
                  0.1006, -0.0211, -0.9947, -0.2647,
                  0.9933, 0.0596, 0.0992, -0.1308,
                  0.0, 0.0, 0.0, 1.0;

        C_Mc_ << 600.0215532765091, 0.0, 328.4470315774296, 0.0,
                0.0, 600.7558135485723, 236.1876388527033, 0.0,
                0.0, 0.0, 1.0, 0.0;

                
        }
private:
    void bboxCallback(const yolo_msg::msg::BoundingBox::SharedPtr msg)
    {
        last_bbox_ = *msg;
        has_bbox_ = true;
        
    }

    // 클래스 이름을 ID로 매핑
    int getClassID(const std::string& class_name) {
        static const std::unordered_map<std::string, int> class_map = {
            {"A1", 1}, {"A2", 2}, {"A3", 3},
            {"B1", 4}, {"B2", 5}, {"B3", 6}
        };

        auto it = class_map.find(class_name);
        if (it != class_map.end()) {
            return it->second;
        }
        return -1; // 매칭 실패 시
    }

    void poseCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
    {
        if (!has_bbox_) {
            RCLCPP_WARN(get_logger(), "No bounding box received yet.");
            return;
        }

        for (const auto& pose : msg->poses)
        {
            Eigen::Vector4d point_lidar;
            point_lidar << pose.position.x, pose.position.y, pose.position.z, 1.0;

            Eigen::Matrix<double, 3, 4> P = C_Mc_ * C_RTlc_;
            Eigen::Vector3d pixel_homo = P * point_lidar;

            double u = pixel_homo(0) / pixel_homo(2) - 30;
            double v = pixel_homo(1) / pixel_homo(2) - 10;

            int x = last_bbox_.x, y = last_bbox_.y, w = last_bbox_.width, h = last_bbox_.height;

            if (u >= x && u <= x + w && v >= y && v <= y + h)
            {
                // class_id 변환
                int class_id = getClassID(last_bbox_.class_name);

                // 메시지 생성 및 발행
                yolo_msg::msg::TrafficSign yolo_msg;
                yolo_msg.pose = pose;
                yolo_msg.class_id = class_id;

                traffic_pub_->publish(yolo_msg);

                // RViz용 PoseStamped
                geometry_msgs::msg::PoseStamped rviz_pose;
                rviz_pose.header.stamp = get_clock()->now();
                rviz_pose.header.frame_id = "velodyne"; 
                rviz_pose.pose = pose;
                traffic_rviz_pub_->publish(rviz_pose);

                RCLCPP_INFO(get_logger(),
                    "3D point (%.1f, %.1f) inside bbox (class_name: %s, class_id: %d).",
                    u, v, last_bbox_.class_name.c_str(), class_id);
                break;
            }
        }
    }

    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr pose_sub_;
    rclcpp::Subscription<yolo_msg::msg::BoundingBox>::SharedPtr bbox_sub_;
    rclcpp::Publisher<yolo_msg::msg::TrafficSign>::SharedPtr traffic_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr traffic_rviz_pub_;

    Eigen::Matrix<double, 3, 4> C_Mc_;
    Eigen::Matrix<double, 4, 4> C_RTlc_;

    yolo_msg::msg::BoundingBox last_bbox_;
    bool has_bbox_ = false;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TrafficSignChecker>());
    rclcpp::shutdown();
    return 0;
}
