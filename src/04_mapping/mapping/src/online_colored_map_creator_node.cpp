/**
 * online_colored_map_creator_node.cpp
 * 
 * Online Real-Time Topic-based Point Cloud & Colored Map Creator.
 * Subscribes to live LiDAR and Camera topics, registers scans using TF & FastGICP,
 * colorizes point clouds from camera images, publishes real-time map to RViz,
 * and saves binary PCD files via ROS service or upon node shutdown.
 */

#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/CompressedImage.h>
#include <std_srvs/Trigger.h>
#include <tf2_msgs/TFMessage.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/crop_box.h>
#include <pcl/registration/gicp.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <eigen3/Eigen/Dense>

#include <mutex>
#include <map>
#include <vector>
#include <cmath>
#include <fstream>
#include <string>

class OnlineColoredMapCreator {
public:
    OnlineColoredMapCreator(ros::NodeHandle& nh, ros::NodeHandle& pnh)
        : nh_(nh), pnh_(pnh), tf_listener_(tf_buffer_), saved_map_(false) {
        
        pnh_.param<std::string>("lidar_topic",    lidar_topic_,    "/velodyne_points");
        pnh_.param<std::string>("image_topic",    image_topic_,    "/image_jpeg/compressed");
        pnh_.param<bool>("use_color",             use_color_,      true);
        pnh_.param<std::string>("map_frame",      map_frame_,      "map");
        pnh_.param<std::string>("velodyne_frame", velodyne_frame_, "velodyne");
        pnh_.param<std::string>("camera_frame",   camera_frame_,   "Camera");
        std::string default_save_dir = ros::package::getPath("mapping") + "/pcd";
        pnh_.param<std::string>("save_dir",       save_dir_,       default_save_dir);
        pnh_.param<double>("publish_interval",    publish_interval_sec_, 1.0);
        pnh_.param<std::string>("pub_topic",      pub_topic_,      "/online_map");

        // Camera intrinsics
        double fx, fy, cx, cy;
        pnh_.param<double>("fx", fx, 320.0);
        pnh_.param<double>("fy", fy, 320.0);
        pnh_.param<double>("cx", cx, 320.0);
        pnh_.param<double>("cy", cy, 240.0);
        camera_matrix_ = (cv::Mat_<double>(3, 3) <<
            fx,  0.0, cx,
            0.0, fy,  cy,
            0.0, 0.0, 1.0);

        double k1, k2, p1, p2, k3;
        pnh_.param<double>("k1", k1, 0.0);
        pnh_.param<double>("k2", k2, 0.0);
        pnh_.param<double>("p1", p1, 0.0);
        pnh_.param<double>("p2", p2, 0.0);
        pnh_.param<double>("k3", k3, 0.0);
        dist_coeffs_ = (cv::Mat_<double>(5, 1) << k1, k2, p1, p2, k3);

        // Voxel sizes
        pnh_.param<float>("voxel_size_local",  voxel_size_local_,  0.3f);
        pnh_.param<float>("voxel_size_global", voxel_size_global_, 0.1f);

        // FastGICP settings
        pnh_.param<bool>("use_gicp",          use_gicp_,       false);
        pnh_.param<int>("gicp_threads",       gicp_threads_,   4);
        pnh_.param<int>("gicp_neighbors",     gicp_neighbors_, 10);
        pnh_.param<float>("gicp_max_dist",    gicp_max_dist_,  1.0f);
        pnh_.param<float>("gicp_epsilon",     gicp_epsilon_,   0.01f);
        pnh_.param<int>("gicp_min_map_size",  gicp_min_map_size_, 1000);

        // CropBox settings
        pnh_.param<bool>("use_cropping",      use_cropping_,   true);
        pnh_.param<float>("crop_min_x",       crop_min_x_,     0.5f);
        pnh_.param<float>("crop_max_x",       crop_max_x_,     100.0f);
        pnh_.param<float>("crop_min_y",       crop_min_y_,     -100.0f);
        pnh_.param<float>("crop_max_y",       crop_max_y_,     100.0f);
        pnh_.param<float>("crop_min_z",       crop_min_z_,     -20.0f);
        pnh_.param<float>("crop_max_z",       crop_max_z_,     100.0f);
        pnh_.param<bool>("crop_negative",     crop_negative_,  false);

        // Time sync threshold
        pnh_.param<double>("time_sync_threshold", time_sync_threshold_, 0.03);

        global_map_.reset(new pcl::PointCloud<pcl::PointXYZRGB>());
        global_map_intensity_.reset(new pcl::PointCloud<pcl::PointXYZI>());
        global_map_xyz_.reset(new pcl::PointCloud<pcl::PointXYZ>());

        gicp_.setCorrespondenceRandomness(gicp_neighbors_);
        gicp_.setMaxCorrespondenceDistance(gicp_max_dist_);
        gicp_.setTransformationEpsilon(gicp_epsilon_);
        gicp_.setMaximumIterations(15);

        // Publishers & Service
        pub_map_ = nh_.advertise<sensor_msgs::PointCloud2>(pub_topic_, 1, true);
        save_srv_ = pnh_.advertiseService("save_map", &OnlineColoredMapCreator::saveMapCallback, this);

        // Subscribers
        if (use_color_) {
            sub_image_ = nh_.subscribe(image_topic_, 10, &OnlineColoredMapCreator::imageCallback, this);
            ROS_INFO("[OnlineMapCreator] Subscribed to image topic: %s", image_topic_.c_str());
        }
        sub_lidar_ = nh_.subscribe(lidar_topic_, 5, &OnlineColoredMapCreator::lidarCallback, this);
        ROS_INFO("[OnlineMapCreator] Subscribed to LiDAR topic: %s", lidar_topic_.c_str());

        ROS_INFO("[OnlineMapCreator] Real-Time Online Mapping initialized.");
        ROS_INFO("  map_frame: %s | velodyne_frame: %s | camera_frame: %s",
                 map_frame_.c_str(), velodyne_frame_.c_str(), camera_frame_.c_str());
        ROS_INFO("  publishing online map to '%s' every %.1fs", pub_topic_.c_str(), publish_interval_sec_);
    }

    ~OnlineColoredMapCreator() {
        saveMap();
    }

    void imageCallback(const sensor_msgs::CompressedImage::ConstPtr& msg) {
        std::lock_guard<std::mutex> lock(img_mutex_);
        image_buffer_[msg->header.stamp] = msg;

        // Maintain ring buffer of recent 10 seconds of images
        while (!image_buffer_.empty() && (msg->header.stamp - image_buffer_.begin()->first).toSec() > 10.0) {
            image_buffer_.erase(image_buffer_.begin());
        }
    }

    void lidarCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg) {
        ros::Time lidar_time = cloud_msg->header.stamp;

        // Check TF availability
        if (!tf_buffer_.canTransform(map_frame_, velodyne_frame_, lidar_time, ros::Duration(0.05))) {
            ROS_WARN_THROTTLE(3.0, "[OnlineMapCreator] Waiting for TF transform %s -> %s at time %.3f",
                               map_frame_.c_str(), velodyne_frame_.c_str(), lidar_time.toSec());
            return;
        }

        geometry_msgs::TransformStamped tf_map_vel_lidar;
        try {
            tf_map_vel_lidar = tf_buffer_.lookupTransform(map_frame_, velodyne_frame_, lidar_time);
        } catch (tf2::TransformException& ex) {
            ROS_WARN_THROTTLE(2.0, "[OnlineMapCreator] TF lookup failed: %s", ex.what());
            return;
        }

        // Find closest image if color mode enabled
        sensor_msgs::CompressedImage::ConstPtr closest_img = nullptr;
        if (use_color_) {
            std::lock_guard<std::mutex> lock(img_mutex_);
            if (!image_buffer_.empty()) {
                auto it = image_buffer_.lower_bound(lidar_time);
                ros::Time closest_time;
                if (it == image_buffer_.end()) {
                    auto prev = std::prev(it);
                    closest_time = prev->first;  closest_img = prev->second;
                } else if (it == image_buffer_.begin()) {
                    closest_time = it->first;    closest_img = it->second;
                } else {
                    auto prev = std::prev(it);
                    if ((it->first - lidar_time).toSec() < (lidar_time - prev->first).toSec()) {
                        closest_time = it->first;    closest_img = it->second;
                    } else {
                        closest_time = prev->first;  closest_img = prev->second;
                    }
                }

                if (std::abs((closest_time - lidar_time).toSec()) > time_sync_threshold_) {
                    closest_img = nullptr;
                }
            }
        }

        processFrame(cloud_msg, closest_img, tf_map_vel_lidar);

        // Periodically publish real-time accumulated map for RViz visualization
        ros::Time now = ros::Time::now();
        if ((now - last_pub_time_).toSec() >= publish_interval_sec_) {
            publishOnlineMap();
            last_pub_time_ = now;
        }
    }

    bool saveMapCallback(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {
        if (saveMap()) {
            res.success = true;
            res.message = "Successfully saved PCD map to " + save_dir_;
        } else {
            res.success = false;
            res.message = "Failed to save PCD map (map may be empty).";
        }
        return true;
    }

    bool saveMap() {
        std::lock_guard<std::mutex> lock(map_mutex_);
        if (saved_map_) return true;

        if (global_map_->empty() && global_map_intensity_->empty() && global_map_xyz_->empty()) {
            ROS_WARN("[OnlineMapCreator] Map is empty. Nothing to save.");
            return false;
        }

        std::string dir = save_dir_;
        int ret = system(("mkdir -p " + dir).c_str());
        (void)ret;

        if (!global_map_->empty()) {
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr out(new pcl::PointCloud<pcl::PointXYZRGB>());
            pcl::VoxelGrid<pcl::PointXYZRGB> vg;
            vg.setInputCloud(global_map_);
            vg.setLeafSize(voxel_size_global_, voxel_size_global_, voxel_size_global_);
            vg.filter(*out);

            std::string path = uniquePath(dir + "/online_colored_map_rgb", ".pcd");
            ROS_INFO("[OnlineMapCreator] Saving RGB map → %s  (%lu pts)", path.c_str(), out->size());
            pcl::io::savePCDFileBinary(path, *out);
        }

        if (!global_map_intensity_->empty()) {
            pcl::PointCloud<pcl::PointXYZI>::Ptr out(new pcl::PointCloud<pcl::PointXYZI>());
            pcl::VoxelGrid<pcl::PointXYZI> vg;
            vg.setInputCloud(global_map_intensity_);
            vg.setLeafSize(voxel_size_global_, voxel_size_global_, voxel_size_global_);
            vg.filter(*out);

            std::string path = uniquePath(dir + "/online_colored_map_intensity", ".pcd");
            ROS_INFO("[OnlineMapCreator] Saving Intensity map → %s  (%lu pts)", path.c_str(), out->size());
            pcl::io::savePCDFileBinary(path, *out);
        }

        if (global_map_->empty() && global_map_intensity_->empty() && !global_map_xyz_->empty()) {
            pcl::PointCloud<pcl::PointXYZ>::Ptr out(new pcl::PointCloud<pcl::PointXYZ>());
            pcl::VoxelGrid<pcl::PointXYZ> vg;
            vg.setInputCloud(global_map_xyz_);
            vg.setLeafSize(voxel_size_global_, voxel_size_global_, voxel_size_global_);
            vg.filter(*out);

            std::string path = uniquePath(dir + "/online_colored_map_xyz", ".pcd");
            ROS_INFO("[OnlineMapCreator] Saving XYZ map → %s  (%lu pts)", path.c_str(), out->size());
            pcl::io::savePCDFileBinary(path, *out);
        }

        saved_map_ = true;
        return true;
    }

private:
    void processFrame(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg,
                      const sensor_msgs::CompressedImage::ConstPtr& img_msg,
                      const geometry_msgs::TransformStamped& tf_map_vel_lidar) {

        ros::Time lidar_time = cloud_msg->header.stamp;
        Eigen::Affine3d T_map_vel = tf2::transformToEigen(tf_map_vel_lidar);

        geometry_msgs::TransformStamped tf_cam_vel, tf_map_vel_img;
        Eigen::Affine3d T_cam_vel = Eigen::Affine3d::Identity();
        Eigen::Affine3d T_map_vel_img = Eigen::Affine3d::Identity();

        if (img_msg) {
            ros::Time img_time = img_msg->header.stamp;
            try {
                if (tf_buffer_.canTransform(camera_frame_, velodyne_frame_, img_time, ros::Duration(0.02)) &&
                    tf_buffer_.canTransform(map_frame_, velodyne_frame_, img_time, ros::Duration(0.02))) {
                    tf_cam_vel = tf_buffer_.lookupTransform(camera_frame_, velodyne_frame_, img_time);
                    T_cam_vel = tf2::transformToEigen(tf_cam_vel);
                    tf_map_vel_img = tf_buffer_.lookupTransform(map_frame_, velodyne_frame_, img_time);
                    T_map_vel_img = tf2::transformToEigen(tf_map_vel_img);
                } else {
                    T_map_vel_img = T_map_vel;
                }
            } catch (tf2::TransformException& ex) {
                T_map_vel_img = T_map_vel;
            }
        }

        // Load & filter scan
        pcl::PointCloud<pcl::PointXYZI>::Ptr raw(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::fromROSMsg(*cloud_msg, *raw);

        pcl::PointCloud<pcl::PointXYZI>::Ptr cropped(new pcl::PointCloud<pcl::PointXYZI>());
        if (use_cropping_) {
            pcl::CropBox<pcl::PointXYZI> crop;
            crop.setInputCloud(raw);
            crop.setMin(Eigen::Vector4f(crop_min_x_, crop_min_y_, crop_min_z_, 1.0f));
            crop.setMax(Eigen::Vector4f(crop_max_x_, crop_max_y_, crop_max_z_, 1.0f));
            crop.setNegative(crop_negative_);
            crop.filter(*cropped);
        } else {
            cropped = raw;
        }

        pcl::PointCloud<pcl::PointXYZI>::Ptr scan_vel(new pcl::PointCloud<pcl::PointXYZI>());
        {
            pcl::VoxelGrid<pcl::PointXYZI> vg;
            vg.setInputCloud(cropped);
            vg.setLeafSize(voxel_size_local_, voxel_size_local_, voxel_size_local_);
            vg.filter(*scan_vel);
        }

        std::lock_guard<std::mutex> lock(map_mutex_);

        // FastGICP scan-to-map refinement (optional)
        Eigen::Affine3d T_map_vel_refined = T_map_vel;
        if (use_gicp_ && global_map_xyz_->size() >= (size_t)gicp_min_map_size_) {
            pcl::PointCloud<pcl::PointXYZ>::Ptr scan_xyz(new pcl::PointCloud<pcl::PointXYZ>());
            scan_xyz->reserve(scan_vel->size());
            for (const auto& p : scan_vel->points)
                scan_xyz->emplace_back(p.x, p.y, p.z);

            pcl::PointCloud<pcl::PointXYZ>::Ptr scan_map_guess(new pcl::PointCloud<pcl::PointXYZ>());
            pcl::transformPointCloud(*scan_xyz, *scan_map_guess, T_map_vel.matrix().cast<float>());

            gicp_.setInputSource(scan_map_guess);
            gicp_.setInputTarget(global_map_xyz_);

            pcl::PointCloud<pcl::PointXYZ> aligned;
            gicp_.align(aligned);

            if (gicp_.hasConverged()) {
                Eigen::Matrix4f correction = gicp_.getFinalTransformation();
                Eigen::Affine3d T_correction(correction.cast<double>());
                T_map_vel_refined = T_correction * T_map_vel;
            }
        }

        if (img_msg) {
            cv::Mat image = cv::imdecode(cv::Mat(img_msg->data), cv::IMREAD_COLOR);
            if (!image.empty()) {
                Eigen::Matrix3d R_ros_to_optical;
                R_ros_to_optical <<  0, -1,  0,
                                      0,  0, -1,
                                      1,  0,  0;
                Eigen::Affine3d T_ros_to_optical = Eigen::Affine3d::Identity();
                T_ros_to_optical.linear() = R_ros_to_optical;

                Eigen::Affine3d delta_T = T_map_vel_refined * T_map_vel.inverse();
                Eigen::Affine3d T_map_vel_img_refined = delta_T * T_map_vel_img;

                Eigen::Affine3d T_cam_map_img = T_cam_vel * T_map_vel_img_refined.inverse();
                Eigen::Affine3d T_opt_map_refined = T_ros_to_optical * T_cam_map_img;

                Eigen::Matrix3d R = T_opt_map_refined.rotation();
                Eigen::Vector3d T = T_opt_map_refined.translation();

                cv::Mat R_cv = (cv::Mat_<double>(3, 3) <<
                    R(0,0), R(0,1), R(0,2),
                    R(1,0), R(1,1), R(1,2),
                    R(2,0), R(2,1), R(2,2));
                cv::Mat rvec, tvec = (cv::Mat_<double>(3, 1) << T.x(), T.y(), T.z());
                cv::Rodrigues(R_cv, rvec);

                std::vector<cv::Point3f> obj_pts_map;
                std::vector<Eigen::Vector3d> map_pts;
                std::vector<int> obj_idx;

                obj_pts_map.reserve(scan_vel->size());
                map_pts.reserve(scan_vel->size());
                obj_idx.reserve(scan_vel->size());

                for (int i = 0; i < (int)scan_vel->size(); ++i) {
                    const auto& p = scan_vel->points[i];
                    Eigen::Vector3d p_map = T_map_vel_refined * Eigen::Vector3d(p.x, p.y, p.z);
                    Eigen::Vector3d p_opt = T_opt_map_refined * p_map;

                    if (p_opt.z() > 0) {
                        obj_pts_map.emplace_back((float)p_map.x(), (float)p_map.y(), (float)p_map.z());
                        map_pts.push_back(p_map);
                        obj_idx.push_back(i);
                    }
                }

                if (!obj_pts_map.empty()) {
                    std::vector<cv::Point2f> img_pts;
                    cv::projectPoints(obj_pts_map, rvec, tvec, camera_matrix_, dist_coeffs_, img_pts);

                    for (size_t i = 0; i < img_pts.size(); ++i) {
                        int u = (int)std::round(img_pts[i].x);
                        int v = (int)std::round(img_pts[i].y);
                        if (u < 0 || u >= image.cols || v < 0 || v >= image.rows) continue;

                        const auto& src = scan_vel->points[obj_idx[i]];
                        const Eigen::Vector3d& p_map = map_pts[i];

                        int region_pt = std::min((int)src.intensity / 32, 7);
                        cv::Vec3b best_color = image.at<cv::Vec3b>(v, u);
                        bool match_found = false;

                        uint8_t gray = 0.299 * best_color[2] + 0.587 * best_color[1] + 0.114 * best_color[0];
                        int region_px = std::min(gray / 32, 7);

                        if (region_pt == region_px) {
                            match_found = true;
                        } else {
                            int search_radius = 2;
                            for (int dy = -search_radius; dy <= search_radius; ++dy) {
                                for (int dx = -search_radius; dx <= search_radius; ++dx) {
                                    int nu = u + dx;
                                    int nv = v + dy;
                                    if (nu < 0 || nu >= image.cols || nv < 0 || nv >= image.rows) continue;

                                    cv::Vec3b n_color = image.at<cv::Vec3b>(nv, nu);
                                    uint8_t n_gray = 0.299 * n_color[2] + 0.587 * n_color[1] + 0.114 * n_color[0];
                                    int n_region_px = std::min(n_gray / 32, 7);

                                    if (region_pt == n_region_px) {
                                        best_color = n_color;
                                        match_found = true;
                                        break;
                                    }
                                }
                                if (match_found) break;
                            }
                        }

                        pcl::PointXYZRGB cp;
                        cp.x = p_map.x();  cp.y = p_map.y();  cp.z = p_map.z();
                        cp.r = best_color[2];   cp.g = best_color[1];   cp.b = best_color[0];
                        global_map_->push_back(cp);

                        pcl::PointXYZI ip;
                        ip.x = p_map.x();  ip.y = p_map.y();  ip.z = p_map.z();
                        ip.intensity = src.intensity;
                        global_map_intensity_->push_back(ip);
                    }
                }
            }
        } else {
            for (const auto& p : scan_vel->points) {
                Eigen::Vector3d p_map = T_map_vel_refined * Eigen::Vector3d(p.x, p.y, p.z);
                pcl::PointXYZI ip;
                ip.x = p_map.x();  ip.y = p_map.y();  ip.z = p_map.z();
                ip.intensity = p.intensity;
                global_map_intensity_->push_back(ip);
            }
        }

        // Maintain GICP target cloud
        for (const auto& p : scan_vel->points) {
            Eigen::Vector3d pm = T_map_vel_refined * Eigen::Vector3d(p.x, p.y, p.z);
            global_map_xyz_->emplace_back(pm.x(), pm.y(), pm.z());
        }

        if (global_map_xyz_->size() > 200000) {
            pcl::PointCloud<pcl::PointXYZ>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZ>());
            pcl::VoxelGrid<pcl::PointXYZ> vg;
            vg.setInputCloud(global_map_xyz_);
            vg.setLeafSize(voxel_size_local_, voxel_size_local_, voxel_size_local_);
            vg.filter(*tmp);
            global_map_xyz_ = tmp;
        }
    }

    void publishOnlineMap() {
        std::lock_guard<std::mutex> lock(map_mutex_);
        if (global_map_->empty() && global_map_intensity_->empty()) return;

        sensor_msgs::PointCloud2 map_msg;
        if (!global_map_->empty()) {
            pcl::toROSMsg(*global_map_, map_msg);
        } else {
            pcl::toROSMsg(*global_map_intensity_, map_msg);
        }

        map_msg.header.frame_id = map_frame_;
        map_msg.header.stamp = ros::Time::now();
        pub_map_.publish(map_msg);

        ROS_INFO_THROTTLE(10.0, "[OnlineMapCreator] Published real-time map with %lu RGB pts (%lu Intensity pts)",
                           global_map_->size(), global_map_intensity_->size());
    }

    std::string uniquePath(const std::string& base, const std::string& ext) {
        std::string candidate = base + ext;
        for (int i = 1; std::ifstream(candidate).good(); ++i)
            candidate = base + "_" + std::to_string(i) + ext;
        return candidate;
    }

    ros::NodeHandle nh_, pnh_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    ros::Subscriber sub_lidar_;
    ros::Subscriber sub_image_;
    ros::Publisher  pub_map_;
    ros::ServiceServer save_srv_;

    std::string lidar_topic_, image_topic_, pub_topic_;
    bool use_color_;
    std::string map_frame_, velodyne_frame_, camera_frame_;
    std::string save_dir_;
    double publish_interval_sec_;
    ros::Time last_pub_time_;

    cv::Mat camera_matrix_, dist_coeffs_;

    bool  use_gicp_;
    bool  use_cropping_;
    float crop_min_x_, crop_max_x_;
    float crop_min_y_, crop_max_y_;
    float crop_min_z_, crop_max_z_;
    bool  crop_negative_;

    float voxel_size_local_;
    float voxel_size_global_;
    int   gicp_threads_;
    int   gicp_neighbors_;
    float gicp_max_dist_;
    float gicp_epsilon_;
    int   gicp_min_map_size_;
    double time_sync_threshold_;

    std::mutex map_mutex_;
    std::mutex img_mutex_;

    std::map<ros::Time, sensor_msgs::CompressedImage::ConstPtr> image_buffer_;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr global_map_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr   global_map_intensity_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr    global_map_xyz_;

    pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> gicp_;
    bool saved_map_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "online_colored_map_creator");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    OnlineColoredMapCreator creator(nh, pnh);
    ros::spin();
    return 0;
}
