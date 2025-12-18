#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "yolo_msg/msg/bounding_box_array.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include <omp.h>
#include <mutex>

using std::placeholders::_1;

class FusionNode : public rclcpp::Node {
public:
    FusionNode() : Node("bs_Camera_LiDAR_Fusion") {
        cone_poses_subscription = this->create_subscription<geometry_msgs::msg::PoseArray>(
            "/cone_poses", 10,
            std::bind(&FusionNode::ConePoseCallback, this, _1));

        bounding_boxes_subscription = this->create_subscription<yolo_msg::msg::BoundingBoxArray>(
            "/bounding_box", 10,
            std::bind(&FusionNode::boundingBoxesCallback, this, _1));

        yellow_publisher = this->create_publisher<geometry_msgs::msg::PointStamped>("point/yellow", 10);
        blue_publisher = this->create_publisher<geometry_msgs::msg::PointStamped>("point/blue", 10);

        R_RTlc << -0.258213671681845, -0.964342407567990,  0.058046711556751, -0.246695652137148,
                    -0.324116745139498,  0.029870691462926, -0.945545386172290, -0.119482435620032,
                    0.910095618754842, -0.262966657121200, -0.320272543259242, -0.167583798275389,
                    0.0,	0.0,	0.0,	1.000000000000000;

        R_Mc << 520.7113714280379, 0.0,    308.8513561709799, 0.0,
                0.0,	521.4439302987403, 236.2402544548886, 0.0,
                0.0,	0.0,	1.0, 0.0;

        // backup 1001
        L_RTlc <<  0.363452501928163,	-0.931015506651510,	0.033352739266673,	0.218154602864188,
                    -0.298642902093187,	-0.150347366693313,	-0.942447922358443,	-0.088227291598970,
                    0.882448126447962,	0.332574496470819,	-0.332685299383161,	-0.044970737877070,
                    0.0,	0.0,	0.0,	1.000000000000000;

        
        // backup_1001
        L_Mc << 511.1833277232166,	0.0,	329.4307342734572, 0.0,
                0.0,	514.3170736290211,	258.4781200652204, 0.0,
                0.0,	0.0,	1.0, 0.0;


        
        C_RTlc << 0.0572, -0.9980, 0.0270, 0.0271,
                  0.1006, -0.0211, -0.9947, -0.2647,
                  0.9933, 0.0596, 0.0992, -0.1308,
                  0.0, 0.0, 0.0, 1.0;

        C_Mc << 600.0215532765091, 0.0, 328.4470315774296, 0.0,
                0.0, 600.7558135485723, 236.1876388527033, 0.0,
                0.0, 0.0, 1.0, 0.0;

        L_result = L_Mc * L_RTlc;
        C_result = C_Mc * C_RTlc;
        R_result = R_Mc * R_RTlc;
    }

private:
    void ConePoseCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg) 
    {
        for (const auto& pose : msg->poses)
        {
            float offset = 0;

            bool C_poj = false;
            bool R_poj = false;
            bool L_poj = false;

            bool yellow_matching = false;
            bool blue_matching = false;

            Eigen::Vector4d velodyne_3D;
            velodyne_3D << pose.position.x, pose.position.y, pose.position.z, 1.0;
            Eigen::Vector3d projected_LiDAR_C = C_result * velodyne_3D;
            Eigen::Vector3d projected_LiDAR_R = R_result * velodyne_3D;
            Eigen::Vector3d projected_LiDAR_L = L_result * velodyne_3D;

            float scale_factor_C = projected_LiDAR_C(2);
            float scale_factor_R = projected_LiDAR_R(2);
            float scale_factor_L = projected_LiDAR_L(2);

            projected_LiDAR_C /= scale_factor_C;
            projected_LiDAR_R /= scale_factor_R;
            projected_LiDAR_L /= scale_factor_L;

            if (projected_LiDAR_C(0) >= 0 && projected_LiDAR_C(0) <= 640 && projected_LiDAR_C(1) >= 0 && projected_LiDAR_C(1) <= 480) C_poj = true;
            if (projected_LiDAR_R(0) >= 0 && projected_LiDAR_R(0) <= 640 && projected_LiDAR_R(1) >= 0 && projected_LiDAR_R(1) <= 480) R_poj = true;
            if (projected_LiDAR_L(0) >= 0 && projected_LiDAR_L(0) <= 640 && projected_LiDAR_L(1) >= 0 && projected_LiDAR_L(1) <= 480) L_poj = true;

            if (boxes_ != nullptr)
            {
                if (C_poj) 
                {
                    for (const auto& box : boxes_->boxes)
                    {
                        if (box.x < 640 || box.x >= 1280) continue;  // C 카메라 box만
                        
                        std::cout << "Center_Cam" <<  std::endl;
                        double xmin = box.x;
                        double xmax = xmin + box.width;
                        double ymin = box.y;
                        double ymax = ymin + box.height;


                        // 투영 잘되고 있는지 확인용
                        // int xmin_visu = xmin - 640;
                        // int xmax_visu = xmax - 640;
                        // std::cout << "C_xmin=" << xmin_visu  << ", C_xmax=" << xmax_visu
                        //           << ", C_ymin=" << ymin << ", C_ymax=" << ymax 
                        //           << ", class_name=" << box.class_name 
                        //           << ", x, y" << projected_LiDAR_C(0) << "   space   " << projected_LiDAR_C(1) << std::endl;


                        if (projected_LiDAR_C(0) -30 >= xmin - 640 - offset && projected_LiDAR_C(0) -30 <= xmax - 640 + offset &&
                            projected_LiDAR_C(1) -10 >= ymin - offset && projected_LiDAR_C(1) -10 <= ymax + offset)
                        {
                            if (box.class_name == "blue") blue_matching = true;
                            else if (box.class_name == "yellow") yellow_matching = true;
                        }
                    }
                }
                else if (R_poj)
                {
                    for (const auto& box : boxes_->boxes)
                    {
                        if (box.x < 1280 || box.x >= 1920) continue;  // R 카메라 box만
                        std::cout << "Right_Cam" <<  std::endl;


                        double xmin = box.x;
                        double xmax = xmin + box.width;
                        double ymin = box.y;
                        double ymax = ymin + box.height;

                        // std::cout << "R_xmin=" << xmin << ", R_xmax=" << xmax 
                        //           << ", R_ymin=" << ymin << ", R_ymax=" << ymax 
                        //           << ", class_name=" << box.class_name << std::endl;

                        if (projected_LiDAR_R(0) -5 >= xmin - 1280 - offset && projected_LiDAR_R(0) -5 <= xmax - 1280 + offset &&
                            projected_LiDAR_R(1) +80 >= ymin - offset && projected_LiDAR_R(1) +80 <= ymax + offset)
                        {
                            if (box.class_name == "blue") blue_matching = true;
                            else if (box.class_name == "yellow") yellow_matching = true;
                        }
                    }
                }
                else if (L_poj)
                {
                    for (const auto& box : boxes_->boxes)
                    {
                        if (box.x >= 640) continue;  // L 카메라 box만
                        std::cout << "Left_Cam" <<  std::endl;

                        double xmin = box.x;
                        double xmax = xmin + box.width ;
                        double ymin = box.y;
                        double ymax = ymin + box.height;

                        // std::cout << "L_xmin=" << xmin << ", L_xmax=" << xmax 
                        //           << ", L_ymin=" << ymin << ", L_ymax=" << ymax 
                        //           << ", class_name=" << box.class_name 
                        //           << ", x, y" << projected_LiDAR_L(0) << "   space   " << projected_LiDAR_L(1) << std::endl;

                        // if (projected_LiDAR_L(0) -18 >= xmin - offset && projected_LiDAR_L(0) -18 <= xmax + offset &&
                        //     projected_LiDAR_L(1) +13 >= ymin - offset && projected_LiDAR_L(1) +13 <= ymax + offset)

                        if (projected_LiDAR_L(0) -15 >= xmin - offset && projected_LiDAR_L(0) -15 <= xmax + offset &&
                        projected_LiDAR_L(1) +20 >= ymin - offset && projected_LiDAR_L(1) +20 <= ymax + offset)
    
                        {
                            if (box.class_name == "yellow") yellow_matching = true;
                            else if (box.class_name == "blue") blue_matching = true;
                            std::cout << "\n" << box.class_name <<  std::endl;

                        }
                        else{
                            std::cout << "box_matching_fail"<<  std::endl;
                        }

                    }
                }
                else {
                    continue;
                    // std::cout << "not in camera angle" << std::endl;
                }

                // 매칭 결과 처리
                if (blue_matching && !yellow_matching) {
                    geometry_msgs::msg::PointStamped point;
                    point.header = msg->header;
                    point.point = pose.position;
                    blue_publisher->publish(point);
                }
                else if (!blue_matching && yellow_matching) {
                    geometry_msgs::msg::PointStamped point;
                    point.header = msg->header;
                    point.point = pose.position;
                    yellow_publisher->publish(point);
                }
                else if (blue_matching && yellow_matching) {
                    continue;
                    // std::cout << "duplicate matching" << std::endl;
                }
                else {
                    continue;
                    // std::cout << "not matching" << std::endl;
                }
            }
        }

        boxes_ = nullptr;
    }

    void boundingBoxesCallback(const yolo_msg::msg::BoundingBoxArray::SharedPtr msg) 
    {
        if (boxes_ == nullptr) boxes_ = msg;
    }

    Eigen::Matrix<double, 3, 4> L_Mc;
    Eigen::Matrix<double, 3, 4> R_Mc;
    Eigen::Matrix<double, 3, 4> C_Mc;

    Eigen::Matrix4d R_RTlc;
    Eigen::Matrix4d C_RTlc;
    Eigen::Matrix4d L_RTlc;

    Eigen::Matrix<double, 3, 4> R_result;
    Eigen::Matrix<double, 3, 4> C_result;
    Eigen::Matrix<double, 3, 4> L_result;
    
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr cone_poses_subscription;
    rclcpp::Subscription<yolo_msg::msg::BoundingBoxArray>::SharedPtr bounding_boxes_subscription;

    yolo_msg::msg::BoundingBoxArray::SharedPtr boxes_;

    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr yellow_publisher;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr blue_publisher;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FusionNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
