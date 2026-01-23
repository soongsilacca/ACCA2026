#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.hpp>

#include <pclomp/ndt_omp.h>

class NDTLocalizationNode : public rclcpp::Node
{
public:
    NDTLocalizationNode() : Node("ndt_localization_node")
    {
        // Parameters
        this->declare_parameter("ndt_resolution", 1.0);
        this->declare_parameter("ndt_step_size", 0.1);
        this->declare_parameter("scan_voxel_size", 0.5);
        this->declare_parameter("map_voxel_size", 0.2);
        this->declare_parameter("min_scan_range", 1.0);
        this->declare_parameter("max_scan_range", 100.0);
        
        ndt_resolution_ = this->get_parameter("ndt_resolution").as_double();
        ndt_step_size_ = this->get_parameter("ndt_step_size").as_double();
        scan_voxel_size_ = this->get_parameter("scan_voxel_size").as_double();
        map_voxel_size_ = this->get_parameter("map_voxel_size").as_double();
        min_scan_range_ = this->get_parameter("min_scan_range").as_double();
        max_scan_range_ = this->get_parameter("max_scan_range").as_double();

        // NDT Initialization
        ndt_omp_.reset(new pclomp::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>());
        ndt_omp_->setResolution(ndt_resolution_);
        ndt_omp_->setStepSize(ndt_step_size_);
        ndt_omp_->setTransformationEpsilon(0.01);
        ndt_omp_->setMaximumIterations(32);
        // Optimization method: DIRECT, KDTREE
        ndt_omp_->setNeighborhoodSearchMethod(pclomp::DIRECT7);
        ndt_omp_->setNumThreads(omp_get_max_threads());

        // Subscribers & Publishers
        // Subscribers & Publishers
        map_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/map_cloud", rclcpp::QoS(1).transient_local(),
            std::bind(&NDTLocalizationNode::mapCallback, this, std::placeholders::_1));

        // Using raw velodyne points as they are always available (10Hz)
        scan_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/velodyne_points", 10,
            std::bind(&NDTLocalizationNode::scanCallback, this, std::placeholders::_1));
        
        // Initial Pose Subscriber
        initial_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/initialpose", 1,
            std::bind(&NDTLocalizationNode::initialPoseCallback, this, std::placeholders::_1));

        // Use odometry for initial guess
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odometry/local", 10,
            std::bind(&NDTLocalizationNode::odomCallback, this, std::placeholders::_1));

        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/ndt_pose", 10);
        aligned_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/ndt_debug/aligned_cloud", 10);
        
        // Map Request Client
        map_client_ = this->create_client<std_srvs::srv::Trigger>("/publish_map");

        // TF
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        RCLCPP_INFO(this->get_logger(), "NDT Localization Node Started (OMP threads: %d)", omp_get_max_threads());
        
        // Request Map automatically
        requestMap();
    }
    
    void requestMap()
    {
        if (!map_client_->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_WARN(this->get_logger(), "Map service not available, waiting...");
            return;
        }
        auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
        auto future = map_client_->async_send_request(request);
        // We don't block here, just send request
    }

private:
    void mapCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received Map Cloud (Points: %d)", msg->width * msg->height);
        
        pcl::PointCloud<pcl::PointXYZ>::Ptr map_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::fromROSMsg(*msg, *map_cloud);

        // Downsample Map
        pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled_map(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
        voxel_grid.setLeafSize(map_voxel_size_, map_voxel_size_, map_voxel_size_);
        voxel_grid.setInputCloud(map_cloud);
        voxel_grid.filter(*downsampled_map);

        RCLCPP_INFO(this->get_logger(), "Map Downsampled: %ld points", downsampled_map->size());

        ndt_omp_->setInputTarget(downsampled_map);
        map_initialized_ = true;
    }

    void scanCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        if (!map_initialized_) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Waiting for Map...");
            return;
        }

        pcl::PointCloud<pcl::PointXYZ>::Ptr scan_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::fromROSMsg(*msg, *scan_cloud);

        // Range Filter & Downsample
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_scan(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
        voxel_grid.setLeafSize(scan_voxel_size_, scan_voxel_size_, scan_voxel_size_);
        voxel_grid.setInputCloud(scan_cloud);
        voxel_grid.filter(*filtered_scan);

        // Determine Initial Guess
        Eigen::Matrix4f initial_guess = Eigen::Matrix4f::Identity();
        
        if (pose_initialized_) {
            // Predict next pose using odometry delta
            if (last_odom_received_ && odom_received_) {
                // Calculate delta from last NDT update time to now
                // Ideally we use the delta between scan timestamps, but simplifying:
                // T_pred = T_last_ndt * T_delta_odom
                // Note: This matches the movement since the last successful align
                
                // Get incremental movement from odometry
                Eigen::Matrix4f odom_delta = last_odom_pose_.inverse() * current_odom_pose_;
                initial_guess = current_pose_ * odom_delta;
            } else {
                initial_guess = current_pose_;
            }
        } else {
            // Try to get from TF or wait for reset
             try {
                auto tf = tf_buffer_->lookupTransform("map", "base_link", rclcpp::Time(0));
                initial_guess = tf2::transformToEigen(tf).matrix().cast<float>();
                pose_initialized_ = true;
            } catch (tf2::TransformException &ex) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Waiting for initial pose...");
                return;
            }
        }

        // Perform NDT alignment
        pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        ndt_omp_->setInputSource(filtered_scan);
        ndt_omp_->align(*output_cloud, initial_guess);

        // Check result
        if (ndt_omp_->hasConverged()) {
            double fitness = ndt_omp_->getFitnessScore();
            
            RCLCPP_INFO(this->get_logger(), "NDT Converged. Fitness: %.4f (Limit: %.1f)", fitness, pose_initialized_ ? 2.0 : 5.0);
            
            // Adaptive Covariance Gating
            // If fitness is bad (> threshold), don't update current_pose_ or publish with huge covariance
            // Typical good fitness < 0.5. Bad > 1.0. 
            // Allow looser threshold for first initialization
            
            double threshold = pose_initialized_ ? 2.0 : 5.0;
            
            if (fitness > threshold) {
                RCLCPP_WARN(this->get_logger(), "Fitness Score too high! (Ignored)");
                return; // Ignore bad matches to prevent jumping
            }
            
            current_pose_ = ndt_omp_->getFinalTransformation();
            
            // Publish Pose
            geometry_msgs::msg::PoseWithCovarianceStamped pose_msg;
            pose_msg.header.stamp = msg->header.stamp;
            pose_msg.header.frame_id = "map";
            
            // Get Pose from Matrix
            Eigen::Vector3f pos = current_pose_.block<3, 1>(0, 3);
            Eigen::Quaternionf rot(current_pose_.block<3, 3>(0, 0));
            
            pose_msg.pose.pose.position.x = pos.x();
            pose_msg.pose.pose.position.y = pos.y();
            pose_msg.pose.pose.position.z = pos.z();
            pose_msg.pose.pose.orientation.x = rot.x();
            pose_msg.pose.pose.orientation.y = rot.y();
            pose_msg.pose.pose.orientation.z = rot.z();
            pose_msg.pose.pose.orientation.w = rot.w();
            
            // Covariance Scaling (Exponential)
            // Base: 0.05. Scale factor: 5.0
            // fitness=0.1 -> cov=0.08
            // fitness=0.5 -> cov=0.60
            // fitness=1.0 -> cov=7.4 (High uncertainty)
            double base_cov = 0.05;
            double cov_val = base_cov * std::exp(fitness * 3.0);
            
            pose_msg.pose.covariance[0] = cov_val;  // x
            pose_msg.pose.covariance[7] = cov_val;  // y
            pose_msg.pose.covariance[35] = cov_val * 2.0; // yaw

            pose_pub_->publish(pose_msg);
            
            // Update last valid NDT update time/odom for next delta
            if (odom_received_) {
                last_odom_pose_ = current_odom_pose_;
                last_odom_received_ = true;
            }
            
            // Debug publish aligned cloud
            if(aligned_pub_->get_subscription_count() > 0) {
                sensor_msgs::msg::PointCloud2 aligned_msg;
                pcl::toROSMsg(*output_cloud, aligned_msg);
                aligned_msg.header = pose_msg.header;
                aligned_pub_->publish(aligned_msg);
            }
            
            pose_initialized_ = true;
        } else {
            RCLCPP_WARN(this->get_logger(), "NDT Failed to converge");
        }
    }
    
    void initialPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
    {
        Eigen::Affine3d affine;
        tf2::fromMsg(msg->pose.pose, affine);
        current_pose_ = affine.matrix().cast<float>();
        pose_initialized_ = true;
        // Reset odom tracking
        odom_received_ = false;
        last_odom_received_ = false;
        RCLCPP_INFO(this->get_logger(), "Initial Pose Set: x=%.2f, y=%.2f", current_pose_(0,3), current_pose_(1,3));
    }
    
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        // Store current odometry pose
        Eigen::Affine3d affine;
        tf2::fromMsg(msg->pose.pose, affine);
        current_odom_pose_ = affine.matrix().cast<float>();
        odom_received_ = true;
    }

    // Parameters
    double ndt_resolution_;
    double ndt_step_size_;
    double scan_voxel_size_;
    double map_voxel_size_;
    double min_scan_range_;
    double max_scan_range_;

    // NDT
    pclomp::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>::Ptr ndt_omp_;
    
    // State
    bool map_initialized_ = false;
    bool pose_initialized_ = false;
    Eigen::Matrix4f current_pose_;
    
    // Odometry State (for prediction)
    bool odom_received_ = false;
    bool last_odom_received_ = false;
    Eigen::Matrix4f current_odom_pose_;
    Eigen::Matrix4f last_odom_pose_;

    // ROS
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr map_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr scan_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr aligned_pub_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr map_client_;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<NDTLocalizationNode>());
    rclcpp::shutdown();
    return 0;
}
