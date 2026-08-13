/**
 * colored_map_creator_node.cpp
 * 
 * High-Performance Offline Bag-to-Colored-PCD Map Creator
 * with FastGICP Scan-to-Map Refinement and Multi-Camera Batch Projection
 * (Velodyne 16CH + 3 Cameras: Front, Left, Right).
 * 
 * Supports both sensor_msgs/Image (raw) and sensor_msgs/CompressedImage (compressed).
 */

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/image_encodings.h>
#include <tf2_msgs/TFMessage.h>
#include <tf2_ros/buffer.h>
#include <tf2_eigen/tf2_eigen.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/crop_box.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <eigen3/Eigen/Dense>

#include <pcl/registration/gicp.h>

#include <map>
#include <set>
#include <tuple>
#include <vector>
#include <cmath>
#include <fstream>
#include <deque>

struct ImageHolder {
    ros::Time stamp;
    sensor_msgs::Image::ConstPtr raw_msg;
    sensor_msgs::CompressedImage::ConstPtr compressed_msg;

    cv::Mat decode() const {
        if (raw_msg) {
            try {
                return cv_bridge::toCvCopy(raw_msg, sensor_msgs::image_encodings::BGR8)->image;
            } catch (cv_bridge::Exception& e) {
                ROS_ERROR("cv_bridge exception: %s", e.what());
                return cv::Mat();
            }
        } else if (compressed_msg) {
            return cv::imdecode(cv::Mat(compressed_msg->data), cv::IMREAD_COLOR);
        }
        return cv::Mat();
    }
};

struct CameraConfig {
    std::string name;
    std::string topic;
    std::string frame_id;
    cv::Mat camera_matrix;
    cv::Mat dist_coeffs;
    std::map<ros::Time, ImageHolder> image_map;
};

struct ActiveCameraContext {
    std::string name;
    cv::Mat image;
    cv::Mat rvec;
    cv::Mat tvec;
    cv::Mat K;
    cv::Mat D;
    Eigen::Affine3d T_opt_map_refined;
};

class ColoredMapCreator {
public:
    ColoredMapCreator() : nh_("~"), tf_buffer_(ros::Duration(1000.0)) {
        nh_.param<std::string>("bag_path",       bag_path_,       "");
        nh_.param<std::string>("lidar_topic",    lidar_topic_,    "/velodyne_points");
        nh_.param<std::string>("image_topic",    image_topic_,    "/image_jpeg/compressed");
        nh_.param<bool>("use_color", use_color_, true);
        nh_.param<bool>("use_multi_camera", use_multi_camera_, true);
        nh_.param<std::string>("map_frame",      map_frame_,      "map");
        nh_.param<std::string>("velodyne_frame", velodyne_frame_, "velodyne");
        nh_.param<std::string>("camera_frame",   camera_frame_,   "Camera");
        nh_.param<std::string>("save_dir",       save_dir_,       "/home/acca/acca_ws/src/localization/pcd");

        double k1, k2, p1, p2, k3;
        nh_.param<double>("k1", k1, 0.0);
        nh_.param<double>("k2", k2, 0.0);
        nh_.param<double>("p1", p1, 0.0);
        nh_.param<double>("p2", p2, 0.0);
        nh_.param<double>("k3", k3, 0.0);
        cv::Mat dist_coeffs = (cv::Mat_<double>(5, 1) << k1, k2, p1, p2, k3);

        if (use_multi_camera_) {
            // Front Camera (1280x720, FOV 90 deg -> fx=640, fy=640, cx=640, cy=360)
            std::string front_topic, front_frame;
            double front_fx, front_fy, front_cx, front_cy;
            nh_.param<std::string>("front_image_topic", front_topic, "/camera/front/image_raw");
            nh_.param<std::string>("front_camera_frame", front_frame, "camera_front");
            nh_.param<double>("front_fx", front_fx, 640.0);
            nh_.param<double>("front_fy", front_fy, 640.0);
            nh_.param<double>("front_cx", front_cx, 640.0);
            nh_.param<double>("front_cy", front_cy, 360.0);

            CameraConfig front_cam;
            front_cam.name = "front";
            front_cam.topic = front_topic;
            front_cam.frame_id = front_frame;
            front_cam.camera_matrix = (cv::Mat_<double>(3, 3) <<
                front_fx, 0.0, front_cx,
                0.0, front_fy, front_cy,
                0.0, 0.0, 1.0);
            front_cam.dist_coeffs = dist_coeffs.clone();
            cameras_.push_back(front_cam);

            // Left Camera (640x480, FOV 130 deg -> fx=149.22, fy=149.22, cx=320, cy=240)
            std::string left_topic, left_frame;
            double left_fx, left_fy, left_cx, left_cy;
            nh_.param<std::string>("left_image_topic", left_topic, "/camera/left/image_raw");
            nh_.param<std::string>("left_camera_frame", left_frame, "camera_left");
            nh_.param<double>("left_fx", left_fx, 149.22);
            nh_.param<double>("left_fy", left_fy, 149.22);
            nh_.param<double>("left_cx", left_cx, 320.0);
            nh_.param<double>("left_cy", left_cy, 240.0);

            CameraConfig left_cam;
            left_cam.name = "left";
            left_cam.topic = left_topic;
            left_cam.frame_id = left_frame;
            left_cam.camera_matrix = (cv::Mat_<double>(3, 3) <<
                left_fx, 0.0, left_cx,
                0.0, left_fy, left_cy,
                0.0, 0.0, 1.0);
            left_cam.dist_coeffs = dist_coeffs.clone();
            cameras_.push_back(left_cam);

            // Right Camera (640x480, FOV 130 deg -> fx=149.22, fy=149.22, cx=320, cy=240)
            std::string right_topic, right_frame;
            double right_fx, right_fy, right_cx, right_cy;
            nh_.param<std::string>("right_image_topic", right_topic, "/camera/right/image_raw");
            nh_.param<std::string>("right_camera_frame", right_frame, "camera_right");
            nh_.param<double>("right_fx", right_fx, 149.22);
            nh_.param<double>("right_fy", right_fy, 149.22);
            nh_.param<double>("right_cx", right_cx, 320.0);
            nh_.param<double>("right_cy", right_cy, 240.0);

            CameraConfig right_cam;
            right_cam.name = "right";
            right_cam.topic = right_topic;
            right_cam.frame_id = right_frame;
            right_cam.camera_matrix = (cv::Mat_<double>(3, 3) <<
                right_fx, 0.0, right_cx,
                0.0, right_fy, right_cy,
                0.0, 0.0, 1.0);
            right_cam.dist_coeffs = dist_coeffs.clone();
            cameras_.push_back(right_cam);
        } else {
            // Single camera fallback
            double fx, fy, cx, cy;
            nh_.param<double>("fx", fx, 320.0);
            nh_.param<double>("fy", fy, 320.0);
            nh_.param<double>("cx", cx, 320.0);
            nh_.param<double>("cy", cy, 240.0);

            CameraConfig cam;
            cam.name = "single";
            cam.topic = image_topic_;
            cam.frame_id = camera_frame_;
            cam.camera_matrix = (cv::Mat_<double>(3, 3) <<
                fx, 0.0, cx,
                0.0, fy, cy,
                0.0, 0.0, 1.0);
            cam.dist_coeffs = dist_coeffs.clone();
            cameras_.push_back(cam);
        }

        // Voxel sizes
        nh_.param<float>("voxel_size_local",  voxel_size_local_,  0.3f);
        nh_.param<float>("voxel_size_global", voxel_size_global_, 0.1f);

        // FastGICP settings
        nh_.param<bool>("use_gicp",       use_gicp_,       false);
        nh_.param<int>("gicp_threads",    gicp_threads_,   16);
        nh_.param<int>("gicp_neighbors",  gicp_neighbors_, 10);
        nh_.param<float>("gicp_max_dist", gicp_max_dist_,  1.0f);
        nh_.param<float>("gicp_epsilon",  gicp_epsilon_,   0.01f);
        nh_.param<int>("gicp_min_map_size", gicp_min_map_size_, 1000);

        // CropBox settings
        nh_.param<bool>("use_cropping",   use_cropping_,   true);
        nh_.param<float>("crop_min_x",    crop_min_x_,     0.5f);
        nh_.param<float>("crop_max_x",    crop_max_x_,     100.0f);
        nh_.param<float>("crop_min_y",    crop_min_y_,     -100.0f);
        nh_.param<float>("crop_max_y",    crop_max_y_,     100.0f);
        nh_.param<float>("crop_min_z",    crop_min_z_,     -20.0f);
        nh_.param<float>("crop_max_z",    crop_max_z_,     100.0f);
        nh_.param<bool>("crop_negative",  crop_negative_,  false);

        // Time sync threshold
        nh_.param<double>("time_sync_threshold", time_sync_threshold_, 0.03);

        // Progress log interval (frames)
        nh_.param<int>("log_interval", log_interval_, 50);

        // CPU & OpenCV limits
        nh_.param<double>("sleep_delay", sleep_delay_, 0.005);
        nh_.param<int>("opencv_threads", opencv_threads_, 4);
        cv::setNumThreads(opencv_threads_);

        global_map_.reset(new pcl::PointCloud<pcl::PointXYZRGB>());
        global_map_intensity_.reset(new pcl::PointCloud<pcl::PointXYZI>());
        global_map_xyz_.reset(new pcl::PointCloud<pcl::PointXYZ>());

        gicp_.setCorrespondenceRandomness(gicp_neighbors_);
        gicp_.setMaxCorrespondenceDistance(gicp_max_dist_);
        gicp_.setTransformationEpsilon(gicp_epsilon_);
        gicp_.setMaximumIterations(15);

        ROS_INFO("[ColoredMapCreator] Parameters loaded:");
        ROS_INFO("  bag_path      : %s", bag_path_.c_str());
        ROS_INFO("  save_dir      : %s", save_dir_.c_str());
        ROS_INFO("  lidar_topic   : %s", lidar_topic_.c_str());
        ROS_INFO("  map_frame     : %s  vel_frame: %s", map_frame_.c_str(), velodyne_frame_.c_str());
        ROS_INFO("  Num Cameras   : %lu", cameras_.size());
        for (const auto& cam : cameras_) {
            ROS_INFO("    - Camera [%s]: topic=%s, frame=%s", cam.name.c_str(), cam.topic.c_str(), cam.frame_id.c_str());
        }
    }

    // ─────────────────────────────────────────
    //  Main pipeline
    // ─────────────────────────────────────────
    void process() {
        if (bag_path_.empty()) {
            ROS_ERROR("Bag path is empty!");
            return;
        }

        rosbag::Bag bag;
        try {
            bag.open(bag_path_, rosbag::bagmode::Read);
        } catch (rosbag::BagException& e) {
            ROS_ERROR("Failed to open bag: %s", e.what());
            return;
        }

        // ── Check if multi-camera topics exist in the bag ────────
        if (use_color_ && use_multi_camera_) {
            bool any_cam_topics_in_bag = false;
            rosbag::View all_view(bag);
            for (const rosbag::ConnectionInfo* connection : all_view.getConnections()) {
                for (const auto& cam : cameras_) {
                    if (connection->topic == cam.topic) {
                        any_cam_topics_in_bag = true;
                        break;
                    }
                }
                if (any_cam_topics_in_bag) break;
            }

            if (!any_cam_topics_in_bag) {
                ROS_WARN("No multi-camera topics found in bag. Trying fallback single image_topic: %s", image_topic_.c_str());
                CameraConfig fallback_cam;
                fallback_cam.name = "fallback";
                fallback_cam.topic = image_topic_;
                fallback_cam.frame_id = camera_frame_;
                double fx, fy, cx, cy;
                nh_.param<double>("fx", fx, 320.0);
                nh_.param<double>("fy", fy, 320.0);
                nh_.param<double>("cx", cx, 320.0);
                nh_.param<double>("cy", cy, 240.0);
                fallback_cam.camera_matrix = (cv::Mat_<double>(3, 3) <<
                    fx, 0.0, cx,
                    0.0, fy, cy,
                    0.0, 0.0, 1.0);
                double k1, k2, p1, p2, k3;
                nh_.param<double>("k1", k1, 0.0);
                nh_.param<double>("k2", k2, 0.0);
                nh_.param<double>("p1", p1, 0.0);
                nh_.param<double>("p2", p2, 0.0);
                nh_.param<double>("k3", k3, 0.0);
                fallback_cam.dist_coeffs = (cv::Mat_<double>(5, 1) << k1, k2, p1, p2, k3);
                cameras_ = {fallback_cam};
            }
        }

        // ── PASS 1: Reading ONLY TFs to Populate TF Buffer ────────
        ROS_INFO("Pass 1: Reading TFs to populate TF buffer...");
        std::vector<std::string> pass1_topics = {"/tf", "/tf_static", "tf", "tf_static"};
        rosbag::View pass1_view(bag, rosbag::TopicQuery(pass1_topics));
        std::set<std::tuple<ros::Time, std::string, std::string>> seen_tf;

        for (const rosbag::MessageInstance& m : pass1_view) {
            if (m.isType<tf2_msgs::TFMessage>()) {
                auto tf_msg = m.instantiate<tf2_msgs::TFMessage>();
                if (tf_msg) {
                    for (const auto& t : tf_msg->transforms) {
                        auto key = std::make_tuple(t.header.stamp, t.header.frame_id, t.child_frame_id);
                        if (seen_tf.find(key) == seen_tf.end()) {
                            bool is_static = (m.getTopic() == "/tf_static" || m.getTopic() == "tf_static");
                            tf_buffer_.setTransform(t, "bag_source", is_static);
                            seen_tf.insert(key);
                        }
                    }
                }
            }
        }
        ROS_INFO("Pass 1 complete. TF Buffer populated.");

        // ── PASS 2: Processing LiDAR frames + Images chronologically ──
        ROS_INFO("Pass 2: Processing LiDAR and camera images chronologically...");
        std::vector<std::string> pass2_topics = {lidar_topic_};
        std::map<std::string, CameraConfig*> topic_to_cam;

        if (use_color_) {
            for (auto& cam : cameras_) {
                pass2_topics.push_back(cam.topic);
                topic_to_cam[cam.topic] = &cam;
            }
        }

        rosbag::View lidar_count_view(bag, rosbag::TopicQuery(std::vector<std::string>{lidar_topic_}));
        size_t total_frames = lidar_count_view.size();
        int processed_frames = 0;

        rosbag::View pass2_view(bag, rosbag::TopicQuery(pass2_topics));
        std::deque<sensor_msgs::PointCloud2::ConstPtr> lidar_queue;
        ros::Time current_max_time = ros::Time(0);

        for (const rosbag::MessageInstance& m : pass2_view) {
            const std::string& topic = m.getTopic();

            if (topic == lidar_topic_) {
                auto cloud_msg = m.instantiate<sensor_msgs::PointCloud2>();
                if (cloud_msg) {
                    if (cloud_msg->header.stamp > current_max_time) {
                        current_max_time = cloud_msg->header.stamp;
                    }
                    lidar_queue.push_back(cloud_msg);
                }
            } else if (use_color_) {
                auto it = topic_to_cam.find(topic);
                if (it != topic_to_cam.end()) {
                    if (m.isType<sensor_msgs::Image>()) {
                        auto raw_msg = m.instantiate<sensor_msgs::Image>();
                        if (raw_msg) {
                            if (raw_msg->header.stamp > current_max_time) {
                                current_max_time = raw_msg->header.stamp;
                            }
                            ImageHolder holder;
                            holder.stamp = raw_msg->header.stamp;
                            holder.raw_msg = raw_msg;
                            it->second->image_map[holder.stamp] = holder;
                        }
                    } else if (m.isType<sensor_msgs::CompressedImage>()) {
                        auto comp_msg = m.instantiate<sensor_msgs::CompressedImage>();
                        if (comp_msg) {
                            if (comp_msg->header.stamp > current_max_time) {
                                current_max_time = comp_msg->header.stamp;
                            }
                            ImageHolder holder;
                            holder.stamp = comp_msg->header.stamp;
                            holder.compressed_msg = comp_msg;
                            it->second->image_map[holder.stamp] = holder;
                        }
                    }
                }
            }

            // Process any queued LiDAR frames whose sync window is fully read
            while (!lidar_queue.empty()) {
                ros::Time T_front = lidar_queue.front()->header.stamp;
                if (current_max_time > T_front + ros::Duration(time_sync_threshold_)) {
                    processFrame(lidar_queue.front());

                    // Clean up older images from memory
                    if (use_color_) {
                        for (auto& cam : cameras_) {
                            auto img_it = cam.image_map.begin();
                            while (img_it != cam.image_map.end() && img_it->first < T_front - ros::Duration(time_sync_threshold_)) {
                                img_it = cam.image_map.erase(img_it);
                            }
                        }
                    }

                    lidar_queue.pop_front();
                    ++processed_frames;

                    if (processed_frames % log_interval_ == 0 || (size_t)processed_frames == total_frames) {
                        ROS_INFO("Progress: %.1f%% (%d/%lu)  RGB: %lu pts, Intensity: %lu pts",
                                 100.0 * processed_frames / total_frames,
                                 processed_frames, total_frames, global_map_->size(), global_map_intensity_->size());
                    }

                    if (sleep_delay_ > 0.0) {
                        ros::WallDuration(sleep_delay_).sleep();
                    }
                } else {
                    break;
                }
            }
        }

        // Process any remaining frames in the queue
        while (!lidar_queue.empty()) {
            processFrame(lidar_queue.front());

            ros::Time T_front = lidar_queue.front()->header.stamp;
            if (use_color_) {
                for (auto& cam : cameras_) {
                    auto img_it = cam.image_map.begin();
                    while (img_it != cam.image_map.end() && img_it->first < T_front - ros::Duration(time_sync_threshold_)) {
                        img_it = cam.image_map.erase(img_it);
                    }
                }
            }

            lidar_queue.pop_front();
            ++processed_frames;

            if (processed_frames % log_interval_ == 0 || (size_t)processed_frames == total_frames) {
                ROS_INFO("Progress: %.1f%% (%d/%lu)  RGB: %lu pts, Intensity: %lu pts",
                         100.0 * processed_frames / total_frames,
                         processed_frames, total_frames, global_map_->size(), global_map_intensity_->size());
            }

            if (sleep_delay_ > 0.0) {
                ros::WallDuration(sleep_delay_).sleep();
            }
        }

        bag.close();

        // ── Final save ───────────────────────────────────────────
        if (global_map_->empty() && global_map_intensity_->empty() && global_map_xyz_->empty()) {
            ROS_WARN("All maps are empty. Nothing saved.");
            return;
        }

        std::string dir = save_dir_;
        system(("mkdir -p " + dir).c_str());

        if (!global_map_->empty()) {
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr out(new pcl::PointCloud<pcl::PointXYZRGB>());
            pcl::VoxelGrid<pcl::PointXYZRGB> vg;
            vg.setInputCloud(global_map_);
            vg.setLeafSize(voxel_size_global_, voxel_size_global_, voxel_size_global_);
            vg.filter(*out);

            std::string path = uniquePath(dir + "/colored_global_map_rgb", ".pcd");
            ROS_INFO("Saving RGB map → %s  (%lu pts)", path.c_str(), out->size());
            pcl::io::savePCDFileBinary(path, *out);
        }

        if (!global_map_intensity_->empty()) {
            pcl::PointCloud<pcl::PointXYZI>::Ptr out(new pcl::PointCloud<pcl::PointXYZI>());
            pcl::VoxelGrid<pcl::PointXYZI> vg;
            vg.setInputCloud(global_map_intensity_);
            vg.setLeafSize(voxel_size_global_, voxel_size_global_, voxel_size_global_);
            vg.filter(*out);

            std::string path = uniquePath(dir + "/colored_global_map_intensity", ".pcd");
            ROS_INFO("Saving Intensity map → %s  (%lu pts)", path.c_str(), out->size());
            pcl::io::savePCDFileBinary(path, *out);
        }

        if (global_map_->empty() && global_map_intensity_->empty() && !global_map_xyz_->empty()) {
            pcl::PointCloud<pcl::PointXYZ>::Ptr out(new pcl::PointCloud<pcl::PointXYZ>());
            pcl::VoxelGrid<pcl::PointXYZ> vg;
            vg.setInputCloud(global_map_xyz_);
            vg.setLeafSize(voxel_size_global_, voxel_size_global_, voxel_size_global_);
            vg.filter(*out);

            std::string path = uniquePath(dir + "/colored_global_map_xyz", ".pcd");
            ROS_INFO("Saving XYZ fallback map → %s  (%lu pts)", path.c_str(), out->size());
            pcl::io::savePCDFileBinary(path, *out);
        }
    }

private:
    // ─────────────────────────────────────────
    //  Per-frame processing with BATCH projection
    // ─────────────────────────────────────────
    void processFrame(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg) {
        ros::Time lidar_time = cloud_msg->header.stamp;

        // ── TF lookup map <- velodyne at lidar_time ───────────────
        geometry_msgs::TransformStamped tf_map_vel_lidar;
        try {
            tf_map_vel_lidar = tf_buffer_.lookupTransform(map_frame_, velodyne_frame_, lidar_time);
        } catch (tf2::TransformException& ex) {
            ROS_WARN_THROTTLE(1, "TF lookup failed for LiDAR frame: %s", ex.what());
            return;
        }

        Eigen::Affine3d T_map_vel = tf2::transformToEigen(tf_map_vel_lidar);

        // ── Load and pre-filter LiDAR scan ───────────────────────
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

        if (scan_vel->empty()) return;

        // ── FastGICP: Refine T_map_vel against global map ────────
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
            } else {
                ROS_WARN_THROTTLE(2, "FastGICP did not converge. Using TF initial guess.");
            }
        }

        // ── Pre-calculate MAP-frame positions for all scan points ─
        std::vector<Eigen::Vector3d> map_pts(scan_vel->size());
        for (size_t i = 0; i < scan_vel->size(); ++i) {
            const auto& p = scan_vel->points[i];
            map_pts[i] = T_map_vel_refined * Eigen::Vector3d(p.x, p.y, p.z);
        }

        // ── Setup active camera contexts for projection ──────────
        std::vector<ActiveCameraContext> active_cameras;

        Eigen::Matrix3d R_ros_to_optical;
        R_ros_to_optical <<  0, -1,  0,
                              0,  0, -1,
                              1,  0,  0;
        Eigen::Affine3d T_ros_to_optical = Eigen::Affine3d::Identity();
        T_ros_to_optical.linear() = R_ros_to_optical;

        if (use_color_) {
            for (const auto& cam : cameras_) {
                if (cam.image_map.empty()) continue;

                // Find time-closest image
                auto it = cam.image_map.lower_bound(lidar_time);
                ros::Time closest_time;
                bool found = false;
                ImageHolder closest_holder;

                if (it == cam.image_map.end()) {
                    auto prev = std::prev(it);
                    closest_time = prev->first; closest_holder = prev->second; found = true;
                } else if (it == cam.image_map.begin()) {
                    closest_time = it->first;   closest_holder = it->second; found = true;
                } else {
                    auto prev = std::prev(it);
                    if ((it->first - lidar_time).toSec() < (lidar_time - prev->first).toSec()) {
                        closest_time = it->first; closest_holder = it->second;
                    } else {
                        closest_time = prev->first; closest_holder = prev->second;
                    }
                    found = true;
                }

                if (!found || std::abs((closest_time - lidar_time).toSec()) > time_sync_threshold_) {
                    continue;
                }

                cv::Mat image = closest_holder.decode();
                if (image.empty()) continue;

                try {
                    geometry_msgs::TransformStamped tf_cam_vel = tf_buffer_.lookupTransform(cam.frame_id, velodyne_frame_, closest_time);
                    geometry_msgs::TransformStamped tf_map_vel_img = tf_buffer_.lookupTransform(map_frame_, velodyne_frame_, closest_time);

                    Eigen::Affine3d T_cam_vel = tf2::transformToEigen(tf_cam_vel);
                    Eigen::Affine3d T_map_vel_img = tf2::transformToEigen(tf_map_vel_img);

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

                    ActiveCameraContext ctx;
                    ctx.name = cam.name;
                    ctx.image = image;
                    ctx.rvec = rvec;
                    ctx.tvec = tvec;
                    ctx.K = cam.camera_matrix;
                    ctx.D = cam.dist_coeffs;
                    ctx.T_opt_map_refined = T_opt_map_refined;
                    active_cameras.push_back(ctx);

                } catch (tf2::TransformException& ex) {
                    ROS_WARN_THROTTLE(2, "TF lookup failed for camera [%s]: %s", cam.name.c_str(), ex.what());
                }
            }
        }

        // ── BATCH Projection & Color Matching ─────────────────────
        std::vector<bool> point_colored(scan_vel->size(), false);

        for (const auto& ctx : active_cameras) {
            std::vector<cv::Point3f> obj_pts;
            std::vector<size_t> candidate_indices;
            obj_pts.reserve(scan_vel->size());
            candidate_indices.reserve(scan_vel->size());

            // Collect all candidate points in front of this camera (Z > 0)
            for (size_t i = 0; i < scan_vel->size(); ++i) {
                if (point_colored[i]) continue; // Already colored by previous camera
                Eigen::Vector3d p_opt = ctx.T_opt_map_refined * map_pts[i];
                if (p_opt.z() > 0.1) {
                    obj_pts.emplace_back((float)map_pts[i].x(), (float)map_pts[i].y(), (float)map_pts[i].z());
                    candidate_indices.push_back(i);
                }
            }

            if (obj_pts.empty()) continue;

            // BATCH projection call for ALL candidate points in ONE single OpenCV call!
            std::vector<cv::Point2f> img_pts;
            cv::projectPoints(obj_pts, ctx.rvec, ctx.tvec, ctx.K, ctx.D, img_pts);

            for (size_t k = 0; k < img_pts.size(); ++k) {
                int u = (int)std::round(img_pts[k].x);
                int v = (int)std::round(img_pts[k].y);

                if (u >= 0 && u < ctx.image.cols && v >= 0 && v < ctx.image.rows) {
                    size_t idx = candidate_indices[k];
                    const auto& src = scan_vel->points[idx];
                    const Eigen::Vector3d& p_map = map_pts[idx];

                    int region_pt = std::min((int)src.intensity / 32, 7);
                    cv::Vec3b best_color = ctx.image.at<cv::Vec3b>(v, u);

                    uint8_t gray = 0.299 * best_color[2] + 0.587 * best_color[1] + 0.114 * best_color[0];
                    int region_px = std::min(gray / 32, 7);

                    if (region_pt != region_px) {
                        int search_radius = 2;
                        bool local_match = false;
                        for (int dy = -search_radius; dy <= search_radius; ++dy) {
                            for (int dx = -search_radius; dx <= search_radius; ++dx) {
                                int nu = u + dx;
                                int nv = v + dy;
                                if (nu < 0 || nu >= ctx.image.cols || nv < 0 || nv >= ctx.image.rows) continue;

                                cv::Vec3b n_color = ctx.image.at<cv::Vec3b>(nv, nu);
                                uint8_t n_gray = 0.299 * n_color[2] + 0.587 * n_color[1] + 0.114 * n_color[0];
                                int n_region_px = std::min(n_gray / 32, 7);

                                if (region_pt == n_region_px) {
                                    best_color = n_color;
                                    local_match = true;
                                    break;
                                }
                            }
                            if (local_match) break;
                        }
                    }

                    pcl::PointXYZRGB cp;
                    cp.x = p_map.x(); cp.y = p_map.y(); cp.z = p_map.z();
                    cp.r = best_color[2]; cp.g = best_color[1]; cp.b = best_color[0];
                    global_map_->push_back(cp);

                    pcl::PointXYZI ip;
                    ip.x = p_map.x(); ip.y = p_map.y(); ip.z = p_map.z();
                    ip.intensity = src.intensity;
                    global_map_intensity_->push_back(ip);

                    point_colored[idx] = true;
                }
            }
        }

        // Add remaining uncolored points to intensity map
        for (size_t i = 0; i < scan_vel->size(); ++i) {
            const Eigen::Vector3d& p_map = map_pts[i];
            if (!point_colored[i]) {
                pcl::PointXYZI ip;
                ip.x = p_map.x(); ip.y = p_map.y(); ip.z = p_map.z();
                ip.intensity = scan_vel->points[i].intensity;
                global_map_intensity_->push_back(ip);
            }
            global_map_xyz_->emplace_back(p_map.x(), p_map.y(), p_map.z());
        }

        // Periodically downsample GICP target
        if (global_map_xyz_->size() > 200000) {
            pcl::PointCloud<pcl::PointXYZ>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZ>());
            pcl::VoxelGrid<pcl::PointXYZ> vg;
            vg.setInputCloud(global_map_xyz_);
            vg.setLeafSize(voxel_size_local_, voxel_size_local_, voxel_size_local_);
            vg.filter(*tmp);
            global_map_xyz_ = tmp;
        }
    }

    // ─────────────────────────────────────────
    //  Helpers
    // ─────────────────────────────────────────
    std::string uniquePath(const std::string& base, const std::string& ext) {
        std::string candidate = base + ext;
        for (int i = 1; std::ifstream(candidate).good(); ++i)
            candidate = base + "_" + std::to_string(i) + ext;
        return candidate;
    }

    // ─────────────────────────────────────────
    //  Members
    // ─────────────────────────────────────────
    ros::NodeHandle nh_;
    tf2_ros::Buffer tf_buffer_;

    std::string bag_path_, lidar_topic_, image_topic_;
    bool use_color_, use_multi_camera_;
    std::string map_frame_, velodyne_frame_, camera_frame_;
    std::string save_dir_;

    std::vector<CameraConfig> cameras_;

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
    int   log_interval_;
    double sleep_delay_;
    int   opencv_threads_;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr global_map_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr   global_map_intensity_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr    global_map_xyz_;

    pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> gicp_;
};

// ─────────────────────────────────────────────
//  main
// ─────────────────────────────────────────────
int main(int argc, char** argv) {
    ros::init(argc, argv, "colored_map_creator");
    ColoredMapCreator creator;
    creator.process();
    return 0;
}
