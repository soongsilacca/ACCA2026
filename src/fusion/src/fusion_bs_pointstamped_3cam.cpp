#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "darknet_ros_msgs/msg/bounding_boxes.hpp"
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

        bounding_boxes_subscription = this->create_subscription<darknet_ros_msgs::msg::BoundingBoxes>(
            "/bounding_boxes", 10,
            std::bind(&FusionNode::boundingBoxesCallback, this, _1));

        yellow_publisher = this->create_publisher<geometry_msgs::msg::PointStamped>("point/yellow", 10);
        blue_publisher = this->create_publisher<geometry_msgs::msg::PointStamped>("point/blue", 10);



        //2025-08-05
        R_RTlc << -0.252011037788250,	-0.967714562342856,	0.004354613928215,	-0.234789978310402,
                  -0.252272068444823,	0.061350790762093,	-0.965709523591559,	-0.063775406669624,
                   0.934264009964774,	-0.244468006705255,	-0.259588430755493,	-0.025615217072694,
                   0.0,	0.0,	 0.0,	1.000000000000000;


        R_Mc <<520.7113714280379,	0.0,	 308.8513561709799, 0.0,
                0.0,	521.4439302987403,	236.2402544548886, 0.0,
                0.0,	0.0,	1.0, 0.0;



        C_RTlc << 0.0219087204896778,	-0.999707258216931,	0.0102667343830515,	-0.0279157166545777,
                   -0.237333870712555,	-0.0151763017066082,	-0.971309586938740,	-0.145387423857104,
                   0.971181055096887,	0.0188435064384779,	-0.237596886524217,	-0.0492311382578551,
                   0., 0., 0., 1.;


        C_Mc << 625.147805669636,	0.0,	329.488400900984, 0.,
                  0.0, 624.496914634015, 	238.751886816586, 0.,
                  0., 0., 1., 0.;


        L_RTlc <<  0.363452501928163,	-0.931015506651510,	0.033352739266673,	0.218154602864188,
                -0.298642902093187,	-0.150347366693313,	-0.942447922358443,	-0.088227291598970,
                0.882448126447962,	0.332574496470819,	-0.332685299383161,	-0.044970737877070,
                0.0,	0.0,	0.0,	1.000000000000000;


        L_Mc << 511.1833277232166,	0.0,	329.4307342734572, 0.0,
                0.0,	514.3170736290211,	258.4781200652204, 0.0,
                0.0,	0.0,	1.0, 0.0;






        


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

            projected_LiDAR_C = projected_LiDAR_C / scale_factor_C;
            projected_LiDAR_R = projected_LiDAR_R / scale_factor_R;
            projected_LiDAR_L = projected_LiDAR_L / scale_factor_L;

            if (projected_LiDAR_C(0) >= 0 && projected_LiDAR_C(0) <= 640 && projected_LiDAR_C(1) >= 0 && projected_LiDAR_C(1) <= 480) C_poj = true;
            if (projected_LiDAR_R(0) >= 0 && projected_LiDAR_R(0) <= 640 && projected_LiDAR_R(1) >= 0 && projected_LiDAR_R(1) <= 480) R_poj = true;
            if (projected_LiDAR_L(0) >= 0 && projected_LiDAR_L(0) <= 640 && projected_LiDAR_L(1) >= 0 && projected_LiDAR_L(1) <= 480) L_poj = true;


            if (boxes_ != nullptr)
            {
                if (C_poj) 
                {
                    for (const auto& box : boxes_->bounding_boxes)
                    {   
                        if (projected_LiDAR_C(0) >= box.xmin - 640 - offset && projected_LiDAR_C(0) <= box.xmax - 640 + offset && projected_LiDAR_C(1) -50 >= box.ymin - offset && projected_LiDAR_C(1) -50 <= box.ymax + offset)
                        {
                            if (box.class_id == "b_cone") blue_matching = true;
                            else yellow_matching = true;
                        }
                    } 

                    if (blue_matching && !yellow_matching) {
                        geometry_msgs::msg::PointStamped point;
                        point.header = msg->header;
                        // point.header.stamp = this->now();
                        point.point = pose.position;
                        blue_publisher->publish(point);
                    }
                    else if (!blue_matching && yellow_matching) {
                        geometry_msgs::msg::PointStamped point;
                        point.header = msg->header;
                        // point.header.stamp = this->now();
                        point.point = pose.position;
                        yellow_publisher->publish(point);
                    }
                    else if (blue_matching && yellow_matching) std::cout << "C: duplicate matching" << std::endl;
                    else std::cout << "C: not matching" << std::endl;
                }

                else if (R_poj) 
                {
                    for (const auto& box : boxes_->bounding_boxes)
                    {
                        if (projected_LiDAR_R(0) >= box.xmin - 1280 - offset && projected_LiDAR_R(0) <= box.xmax - 1280 + offset && projected_LiDAR_R(1) >= box.ymin - offset && projected_LiDAR_R(1) <= box.ymax + offset)
                        {
                            if (box.class_id == "y_cone") yellow_matching = true;
                            else blue_matching = true;
                        }
                    }

                    if (blue_matching && !yellow_matching) {
                        geometry_msgs::msg::PointStamped point;
                        point.header = msg->header;
                        // point.header.stamp = this->now();
                        point.point = pose.position;
                        blue_publisher->publish(point);
                    }
                    else if (!blue_matching && yellow_matching) {
                        geometry_msgs::msg::PointStamped point;
                        point.header = msg->header;
                        // point.header.stamp = this->now();
                        point.point = pose.position;
                        yellow_publisher->publish(point);
                    }
                    else if (blue_matching && yellow_matching) std::cout << "R: duplicate matching" << std::endl;
                    else std::cout << "R: not matching" << std::endl;
                }

                else if (L_poj) 
                {
                    for (const auto& box : boxes_->bounding_boxes)
                    {
                        if (projected_LiDAR_L(0) >= box.xmin - offset && projected_LiDAR_L(0) <= box.xmax + offset && projected_LiDAR_L(1) >= box.ymin - offset && projected_LiDAR_L(1) <= box.ymax + offset)
                        {
                            if (box.class_id == "b_cone") blue_matching = true;
                            else yellow_matching = true;
                        }
                    }

                    if (blue_matching && !yellow_matching) {
                        geometry_msgs::msg::PointStamped point;
                        point.header = msg->header;
                        // point.header.stamp = this->now();
                        point.point = pose.position;
                        blue_publisher->publish(point);
                    }
                    else if (!blue_matching && yellow_matching) {
                        geometry_msgs::msg::PointStamped point;
                        point.header = msg->header;
                        // point.header.stamp = this->now();
                        point.point = pose.position;
                        yellow_publisher->publish(point);
                    }
                    else if (blue_matching && yellow_matching) std::cout << "L: duplicate matching" << std::endl;
                    else std::cout << "L: not matching" << std::endl;
                }

                else std::cout << "not in camera angle" << std::endl;
            }
        }

        boxes_ = nullptr;
    }

    void boundingBoxesCallback(const darknet_ros_msgs::msg::BoundingBoxes::SharedPtr msg) 
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
    rclcpp::Subscription<darknet_ros_msgs::msg::BoundingBoxes>::SharedPtr bounding_boxes_subscription;

    darknet_ros_msgs::msg::BoundingBoxes::SharedPtr boxes_;

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

//TODO 
// '''
// 현재는 for문을 돌리면서 변환된 LiDAR point가 한 종류의 Bounding boxes에 속하는지 확인후 publish 

// 색 섞이는 것을 막는 코드 짜보기
// '''