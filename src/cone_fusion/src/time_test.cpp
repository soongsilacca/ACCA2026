#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "yolo_msg/msg/bounding_box.hpp"

class TimeDiffChecker : public rclcpp::Node
{
public:
    TimeDiffChecker()
    : Node("time_diff_checker")
    {
        pose_sub_ = create_subscription<geometry_msgs::msg::PoseArray>(
            "cone_poses", 10,
            std::bind(&TimeDiffChecker::poseCallback, this, std::placeholders::_1));

        bbox_sub_ = create_subscription<yolo_msg::msg::BoundingBox>(
            "yolo/detections", 10,
            std::bind(&TimeDiffChecker::bboxCallback, this, std::placeholders::_1));
    }

private:
    void poseCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
    {
        last_pose_time_ = this->get_clock()->now();  // 수신 시각으로 교체

        if (has_bbox_) {
            double dt = (last_pose_time_ - last_bbox_time_).seconds();
            RCLCPP_INFO(this->get_logger(),
                        "Pose recv time: %.3f, BBox recv time: %.3f, Δt = %.3f sec",
                        last_pose_time_.seconds(),
                        last_bbox_time_.seconds(),
                        dt);
        }
    }
    void bboxCallback(const yolo_msg::msg::BoundingBox::SharedPtr msg)
    {
        // BoundingBox에는 header가 없으므로 수신 시간으로 대체
        last_bbox_time_ = this->get_clock()->now();
        has_bbox_ = true;

        RCLCPP_INFO(this->get_logger(),
                    "Received bbox at local node time: %.3f",
                    last_bbox_time_.seconds());
    }

    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr pose_sub_;
    rclcpp::Subscription<yolo_msg::msg::BoundingBox>::SharedPtr bbox_sub_;

    rclcpp::Time last_pose_time_;
    rclcpp::Time last_bbox_time_;
    bool has_bbox_ = false;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TimeDiffChecker>());
    rclcpp::shutdown();
    return 0;
}
