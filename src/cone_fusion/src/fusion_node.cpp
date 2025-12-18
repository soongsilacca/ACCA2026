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

        R_RTlc << -0.281817835135859,	-0.959159956569860,	0.024308136749690,	-0.225730494890862,
                    -0.245390341164270,	0.047561256307933,	-0.968256942841978,	-0.139140276331998,
                    0.927557161722457,	-0.278837057457066,	-0.248772199262828,	0.019980886445306,
                    0.0,	0.0,	0.0,	1.000000000000000;

        R_Mc << 532.5769782427712,	0.0,	306.8365465098084, 0.0,
                0.0,	531.9526386746585,	251.4871088011509, 0.0,
                0.0,	0.0,	1.0, 0.0,
                0.0,    0.0,    1.0, 0.0;

        L_RTlc <<  0.363452501928163,	-0.931015506651510,	0.033352739266673,	0.218154602864188,
                    -0.298642902093187,	-0.150347366693313,	-0.942447922358443,	-0.088227291598970,
                    0.882448126447962,	0.332574496470819,	-0.332685299383161,	-0.044970737877070,
                    0.0,	0.0,	0.0,	1.000000000000000;

        L_Mc << 511.1833277232166,	0.0,	329.4307342734572, 0.0,
                0.0,	514.3170736290211,	258.4781200652204, 0.0,
                0.0,	0.0,	1.0, 0.0,
                0.0,0.0,0.0,1.0;

        C_RTlc << 0.0572, -0.9980, 0.0270, 0.0271,
                  0.1006, -0.0211, -0.9947, -0.2647,
                  0.9933,  0.0596,  0.0992, -0.1308,
                  0.0, 0.0, 0.0, 1.0;

        C_Mc << 600.0216, 0.0, 328.4470, 0.0,
                0.0, 600.7558, 236.1876, 0.0,
                0.0, 0.0, 1.0, 0.0,
                0.0,0.0,0.0,1.0;

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

            Eigen::Vector4d velodyne_4D;
            velodyne_4D << pose.position.x, pose.position.y, pose.position.z, 1.0;
            Eigen::Vector4d projected_LiDAR_C = C_result * velodyne_4D;
            Eigen::Vector4d projected_LiDAR_R = R_result * velodyne_4D;
            Eigen::Vector4d projected_LiDAR_L = L_result * velodyne_4D;

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

                        double xmin = box.x;
                        double xmax = xmin + box.width;
                        double ymin = box.y;
                        double ymax = ymin + box.height;
                        Eigen::Matrix4d Mc = C_Mc;
                        Eigen::Matrix4d RTlc = C_RTlc;

                        if (projected_LiDAR_C(0) -15 >= xmin - 640 - offset && projected_LiDAR_C(0) -15 <= xmax - 640 + offset &&
                            projected_LiDAR_C(1) >= ymin - offset && projected_LiDAR_C(1) <= ymax + offset)
                        {
                            for (const auto& pose : msg->poses) {
                                for (const auto& box : boxes_->boxes) {
                                    double u = box.pixel_center_x;
                                    double v = box.pixel_center_y;
                                    if (box.pixel_center_x < 640 || box.pixel_center_x >= 1280) continue;  // C 카메라 box만

                                    double depth = scale_factor_C;
                                    double offset_u = u+15;
                                    double offset_v = v;


                                     //이 부분이 잘못됐네. proj_left,ceter,right의 (u,v,w,1)중 w값을 가져와야함.
                            
                                    // u,v,1,1 형태의 동차 좌표 벡터 생성
                                    Eigen::Vector4d pixel_vec(offset_u* depth, offset_v * depth, depth, 1.0);
                            
                                    // Mc 역행렬 계산
                                    Eigen::Matrix4d Mc_inv = Mc.inverse();
                            
                                    // 깊이 곱해서 3D 좌표 얻기 (homogeneous coord.)
                                    Eigen::Vector4d pixel_cam_homo = Mc_inv * pixel_vec;
                            
                                    // LiDAR 좌표계로 변환
                                    Eigen::Vector4d reproj_lidar = RTlc.inverse() * pixel_cam_homo;
                                    Eigen::Vector4d reproj_lidar_real(
                                        reproj_lidar(0) / reproj_lidar(3),
                                        reproj_lidar(1) / reproj_lidar(3),
                                        reproj_lidar(2) / reproj_lidar(3),
                                        1.0
                                    );
                                         
                            
                                    Eigen::Vector4d lidar_point(pose.position.x, pose.position.y, pose.position.z,1.0);
                                    
                                    double dist = (reproj_lidar.head<3>() - lidar_point.head<3>()).norm();
                            
                                    const double THRESH = 0.5;
                                    if (dist < THRESH) {
                                        if (box.class_name == "blue") blue_matching = true;
                                        else if (box.class_name == "yellow") yellow_matching = true;
                                    }
                                }
                            }
                        }
                    }
                }
                else if (R_poj)
                {
                    for (const auto& box : boxes_->boxes)
                    {
                        if (box.x < 1280 || box.x >= 1920) continue;  // R 카메라 box만

                        double xmin = box.x;
                        double xmax = xmin + box.width;
                        double ymin = box.y;
                        double ymax = ymin + box.height;
                        Eigen::Matrix4d Mc = R_Mc;
                        Eigen::Matrix4d RTlc = R_RTlc;


                        if (projected_LiDAR_R(0) +10 >= xmin - 1280 - offset && projected_LiDAR_R(0) +10 <= xmax - 1280 + offset &&
                            projected_LiDAR_R(1) +90 >= ymin - offset && projected_LiDAR_R(1) +90 <= ymax + offset)
                        {
                            for (const auto& pose : msg->poses) {
                                for (const auto& box : boxes_->boxes) {
                                    double u = box.pixel_center_x;
                                    double v = box.pixel_center_y;
                                    if (box.pixel_center_x < 1280 || box.pixel_center_x >= 1920) continue;  // C 카메라 box만

                                    double depth = scale_factor_R;
                                    double offset_u = u-10;
                                    double offset_v = v-90;


                                     //이 부분이 잘못됐네. proj_left,ceter,right의 (u,v,w,1)중 w값을 가져와야함.
                            
                                    // u,v,1,1 형태의 동차 좌표 벡터 생성
                                    Eigen::Vector4d pixel_vec(offset_u* depth, offset_v * depth, depth, 1.0);
                            
                                    // Mc 역행렬 계산
                                    Eigen::Matrix4d Mc_inv = Mc.inverse();
                            
                                    // 깊이 곱해서 3D 좌표 얻기 (homogeneous coord.)
                                    Eigen::Vector4d pixel_cam_homo = Mc_inv * pixel_vec;
                            
                                    // LiDAR 좌표계로 변환
                                    Eigen::Vector4d reproj_lidar = RTlc.inverse() * pixel_cam_homo;
                                    Eigen::Vector4d reproj_lidar_real(
                                        reproj_lidar(0) / reproj_lidar(3),
                                        reproj_lidar(1) / reproj_lidar(3),
                                        reproj_lidar(2) / reproj_lidar(3),
                                        1.0
                                    );
                                         
                            
                                    Eigen::Vector4d lidar_point(pose.position.x, pose.position.y, pose.position.z,1.0);
                                    
                                    double dist = (reproj_lidar.head<3>() - lidar_point.head<3>()).norm();
                            
                                    const double THRESH = 0.5;
                                    if (dist < THRESH) {
                                        if (box.class_name == "blue") blue_matching = true;
                                        else if (box.class_name == "yellow") yellow_matching = true;
                                    }
                                }
                            }
                        }
                    }
                }
                else if (L_poj)
                {
                    for (const auto& box : boxes_->boxes)
                    {
                        if (box.x >= 640) continue;  // L 카메라 box만

                        double xmin = box.x;
                        double xmax = xmin + box.width;
                        double ymin = box.y;
                        double ymax = ymin + box.height;
                        Eigen::Matrix4d Mc = L_Mc;
                        Eigen::Matrix4d RTlc = L_RTlc;


                        if (projected_LiDAR_L(0) -3 >= xmin - offset && projected_LiDAR_L(0) -3 <= xmax + offset &&
                            projected_LiDAR_L(1) >= ymin - offset && projected_LiDAR_L(1) <= ymax + offset)
                        {
                            for (const auto& pose : msg->poses) {
                                for (const auto& box : boxes_->boxes) {
                                    double u = box.pixel_center_x;
                                    double v = box.pixel_center_y;
                                    if (box.pixel_center_x  >= 640) continue;  // C 카메라 box만

                                    double depth = scale_factor_L;
                                    double offset_u = u+3;
                                    double offset_v = v;


                                     //이 부분이 잘못됐네. proj_left,ceter,right의 (u,v,w,1)중 w값을 가져와야함.
                            
                                    // u,v,1,1 형태의 동차 좌표 벡터 생성
                                    Eigen::Vector4d pixel_vec(offset_u* depth, offset_v * depth, depth, 1.0);
                            
                                    // Mc 역행렬 계산
                                    Eigen::Matrix4d Mc_inv = Mc.inverse();
                            
                                    // 깊이 곱해서 3D 좌표 얻기 (homogeneous coord.)
                                    Eigen::Vector4d pixel_cam_homo = Mc_inv * pixel_vec;
                            
                                    // LiDAR 좌표계로 변환
                                    Eigen::Vector4d reproj_lidar = RTlc.inverse() * pixel_cam_homo;
                                    Eigen::Vector4d reproj_lidar_real(
                                        reproj_lidar(0) / reproj_lidar(3),
                                        reproj_lidar(1) / reproj_lidar(3),
                                        reproj_lidar(2) / reproj_lidar(3),
                                        1.0
                                    );
                                         
                            
                                    Eigen::Vector4d lidar_point(pose.position.x, pose.position.y, pose.position.z,1.0);
                                    
                                    double dist = (reproj_lidar.head<3>() - lidar_point.head<3>()).norm();
                            
                                    const double THRESH = 0.5;
                                    if (dist < THRESH) {
                                        if (box.class_name == "yellow") yellow_matching = true;
                                        else if (box.class_name == "blue") blue_matching = true;
                                    }
                                }
                            }
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

    Eigen::Matrix<double, 4, 4> L_Mc;
    Eigen::Matrix<double, 4, 4> R_Mc;
    Eigen::Matrix<double, 4, 4> C_Mc;

    Eigen::Matrix4d R_RTlc;
    Eigen::Matrix4d C_RTlc;
    Eigen::Matrix4d L_RTlc;

    Eigen::Matrix<double, 4, 4> R_result;
    Eigen::Matrix<double, 4, 4> C_result;
    Eigen::Matrix<double, 4, 4> L_result;
    
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
