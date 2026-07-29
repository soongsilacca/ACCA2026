#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pclomp/ndt_omp.h>
#include <Eigen/Dense>
#include <mutex>
#include <omp.h>

class NdtLocalizationNode {
public:
    NdtLocalizationNode(ros::NodeHandle& nh) : nh_(nh), has_init_guess_(false) {
        // Load parameters
        std::string default_map_pcd = ros::package::getPath("localization") + "/pcd/colored_global_map_intensity.pcd";
        std::string map_pcd_path;
        nh_.param<std::string>("map_pcd_path", map_pcd_path, default_map_pcd);
        
        double ndt_resolution;
        nh_.param<double>("ndt_resolution", ndt_resolution, 1.0);
        double ndt_step_size;
        nh_.param<double>("ndt_step_size", ndt_step_size, 0.1);
        double ndt_epsilon;
        nh_.param<double>("ndt_epsilon", ndt_epsilon, 0.01);
        int ndt_max_iter;
        nh_.param<int>("ndt_max_iter", ndt_max_iter, 35);
        
        nh_.param<double>("map_voxel_size", map_voxel_size_, 0.5);
        nh_.param<double>("scan_voxel_size", scan_voxel_size_, 0.5);
        
        ROS_INFO_STREAM("Loading global map PCD from: " << map_pcd_path);
        
        // Load PCD Map
        pcl::PointCloud<pcl::PointXYZ>::Ptr raw_map_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        if (pcl::io::loadPCDFile(map_pcd_path, *raw_map_cloud) == -1) {
            ROS_ERROR_STREAM("Failed to load map PCD from " << map_pcd_path);
            ros::shutdown();
            return;
        }
        ROS_INFO_STREAM("Successfully loaded raw map with " << raw_map_cloud->size() << " points.");
        
        // Downsample Global Map
        pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled_map_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::VoxelGrid<pcl::PointXYZ> voxelgrid_map;
        voxelgrid_map.setLeafSize(map_voxel_size_, map_voxel_size_, map_voxel_size_);
        voxelgrid_map.setInputCloud(raw_map_cloud);
        voxelgrid_map.filter(*downsampled_map_cloud);
        ROS_INFO_STREAM("Downsampled map has " << downsampled_map_cloud->size() << " points.");
        
        // Setup ndt_omp
        ndt_.reset(new pclomp::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>());
        ndt_->setResolution(ndt_resolution);
        ndt_->setStepSize(ndt_step_size);
        ndt_->setTransformationEpsilon(ndt_epsilon);
        ndt_->setMaximumIterations(ndt_max_iter);
        
        // Use multi-threading and DIRECT7 search for speed/robustness
        ndt_->setNumThreads(omp_get_max_threads());
        ndt_->setNeighborhoodSearchMethod(pclomp::DIRECT7);
        ndt_->setInputTarget(downsampled_map_cloud);
        
        ROS_INFO_STREAM("OMP NDT initialized with " << omp_get_max_threads() << " threads.");
        
        // Advertisers and Subscribers
        pub_ndt_pose_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("/ndt_pose", 10);
        pub_ndt_odom_ = nh_.advertise<nav_msgs::Odometry>("/odometry/ndt", 10);
        pub_ndt_map_  = nh_.advertise<sensor_msgs::PointCloud2>("/ndt_map", 1, true); // Latched
        
        // Publish map for visualization
        sensor_msgs::PointCloud2 map_msg;
        pcl::toROSMsg(*downsampled_map_cloud, map_msg);
        map_msg.header.frame_id = "map";
        map_msg.header.stamp = ros::Time::now();
        pub_ndt_map_.publish(map_msg);
        
        sub_fast_lio_odom_ = nh_.subscribe("/odometry/fast_lio", 10, &NdtLocalizationNode::fastLioOdomCallback, this);
        sub_initial_pose_  = nh_.subscribe("/initialpose", 2, &NdtLocalizationNode::initialPoseCallback, this);
        sub_gps_pose_      = nh_.subscribe("/gps_pose", 2, &NdtLocalizationNode::gpsPoseCallback, this);
        sub_velodyne_      = nh_.subscribe("/velodyne_points", 2, &NdtLocalizationNode::velodyneCallback, this);
        
        ROS_INFO("OMP NDT Localization node is ready!");
    }

private:
    void fastLioOdomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        std::lock_guard<std::mutex> lock(mtx_);
        latest_odom_pose_ = msg->pose.pose;
        has_init_guess_ = true;
    }
    
    void initialPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
        std::lock_guard<std::mutex> lock(mtx_);
        latest_odom_pose_ = msg->pose.pose;
        has_init_guess_ = true;
        ROS_INFO("NDT initial pose updated manually from RViz.");
    }
    
    void velodyneCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
        // Read initial guess from FAST_LIO odometry
        geometry_msgs::Pose initial_pose;
        {
            std::lock_guard<std::mutex> lock(mtx_);
            if (!has_init_guess_) {
                ROS_WARN_THROTTLE(5.0, "Waiting for initial guess pose from /odometry/fast_lio or /initialpose...");
                return;
            }
            initial_pose = latest_odom_pose_;
        }
        
        auto start_time = ros::WallTime::now();
        
        // Convert ROS msg to PCL point cloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr raw_scan(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::fromROSMsg(*msg, *raw_scan);
        
        // Downsample incoming scan
        pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled_scan(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::VoxelGrid<pcl::PointXYZ> voxelgrid_scan;
        voxelgrid_scan.setLeafSize(scan_voxel_size_, scan_voxel_size_, scan_voxel_size_);
        voxelgrid_scan.setInputCloud(raw_scan);
        voxelgrid_scan.filter(*downsampled_scan);
        
        ndt_->setInputSource(downsampled_scan);
        
        // Build initial guess transformation matrix
        Eigen::Translation3f init_trans(initial_pose.position.x, initial_pose.position.y, initial_pose.position.z);
        Eigen::Quaternionf init_rot(initial_pose.orientation.w, initial_pose.orientation.x, initial_pose.orientation.y, initial_pose.orientation.z);
        Eigen::Matrix4f init_guess = (init_trans * init_rot).matrix();
        
        // Align
        pcl::PointCloud<pcl::PointXYZ>::Ptr aligned_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        ndt_->align(*aligned_cloud, init_guess);
        
        auto end_time = ros::WallTime::now();
        double elapsed_ms = (end_time - start_time).toSec() * 1000.0;
        
        // Retrieve result
        Eigen::Matrix4f ndt_trans = ndt_->getFinalTransformation();
        Eigen::Matrix3f rotation_matrix = ndt_trans.block<3, 3>(0, 0);
        Eigen::Vector3f translation_vector = ndt_trans.block<3, 1>(0, 3);
        Eigen::Quaternionf final_quat(rotation_matrix);
        
        double fitness_score = ndt_->getFitnessScore();
        int final_iterations = ndt_->getFinalNumIteration();
        
        ROS_INFO_THROTTLE(1.0, "NDT align: time: %.2f ms | iter: %d | fitness: %.4f", elapsed_ms, final_iterations, fitness_score);
        
        // Publish ndt_pose (geometry_msgs::PoseWithCovarianceStamped)
        geometry_msgs::PoseWithCovarianceStamped pose_msg;
        pose_msg.header = msg->header;
        pose_msg.header.frame_id = "map";
        pose_msg.pose.pose.position.x = translation_vector.x();
        pose_msg.pose.pose.position.y = translation_vector.y();
        pose_msg.pose.pose.position.z = translation_vector.z();
        pose_msg.pose.pose.orientation.x = final_quat.x();
        pose_msg.pose.pose.orientation.y = final_quat.y();
        pose_msg.pose.pose.orientation.z = final_quat.z();
        pose_msg.pose.pose.orientation.w = final_quat.w();

        // Populate the covariance matrix dynamically scaled by the NDT fitness score (Autoware style).
        // This lets EKF trust NDT more when fitness_score is low (good match).
        for (int i = 0; i < 36; ++i) {
            pose_msg.pose.covariance[i] = 0.0;
        }
        
        // Floor the covariance to prevent extremely small values (minimum threshold)
        double cov_val = fitness_score;
        if (cov_val < 0.005) cov_val = 0.005;
        
        pose_msg.pose.covariance[0] = cov_val;        // X
        pose_msg.pose.covariance[7] = cov_val;        // Y
        pose_msg.pose.covariance[14] = 99999.0;       // Z
        pose_msg.pose.covariance[21] = 99999.0;       // Roll
        pose_msg.pose.covariance[28] = 99999.0;       // Pitch
        pose_msg.pose.covariance[35] = cov_val * 2.0; // Yaw (usually slightly less certain than position)
        
        pub_ndt_pose_.publish(pose_msg);
        
        // Publish ndt_odom (nav_msgs::Odometry)
        nav_msgs::Odometry odom_msg;
        odom_msg.header = pose_msg.header;
        odom_msg.child_frame_id = "base_link";
        odom_msg.pose.pose = pose_msg.pose.pose;
        odom_msg.pose.covariance = pose_msg.pose.covariance;
        pub_ndt_odom_.publish(odom_msg);
    }
    
    void gpsPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        std::lock_guard<std::mutex> lock(mtx_);
        if (!has_init_guess_) {
            latest_odom_pose_ = msg->pose;
            has_init_guess_ = true;
            ROS_INFO("NDT initial pose automatically initialized from GPS pose.");
        }
    }
    
    ros::NodeHandle nh_;
    ros::Subscriber sub_velodyne_;
    ros::Subscriber sub_fast_lio_odom_;
    ros::Subscriber sub_initial_pose_;
    ros::Subscriber sub_gps_pose_;
    ros::Publisher pub_ndt_pose_;
    ros::Publisher pub_ndt_odom_;
    ros::Publisher pub_ndt_map_;
    
    pclomp::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>::Ptr ndt_;
    
    double map_voxel_size_;
    double scan_voxel_size_;
    
    std::mutex mtx_;
    geometry_msgs::Pose latest_odom_pose_;
    bool has_init_guess_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "ndt_localization_node");
    ros::NodeHandle nh("~");
    NdtLocalizationNode node(nh);
    ros::spin();
    return 0;
}
