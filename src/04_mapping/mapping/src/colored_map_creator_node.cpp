/**
 * colored_map_creator_node.cpp
 * 
 * Offline Bag-to-Colored-PCD Map Creator with FastGICP Scan-to-Map Refinement.
 * 
 * Pipeline:
 *   Pass 1: Pre-load all /tf and /tf_static into tf2_ros::Buffer.
 *   Pass 2: Cache all camera images into a sorted std::map for fast lookup.
 *   Pass 3: For each LiDAR frame:
 *     (a) Find the time-closest image (within 30ms threshold).
 *     (b) Look up TF-based initial guess (map <- velodyne) and extrinsic (camera <- velodyne).
 *     (c) Use FastGICP to refine the initial guess against the accumulated global map.
 *     (d) Project LiDAR points onto the image using the corrected extrinsic + OpenCV optical frame.
 *     (e) Accumulate colored / intensity points into the global map.
 *   Final: Apply VoxelGrid downsampling and save as binary PCD.
 */

#include <ros/ros.h>
#include <ros/package.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/CompressedImage.h>
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

// PCL GICP for stable scan-to-map registration
#include <pcl/registration/gicp.h>

#include <map>
#include <set>



#include <tuple>
#include <vector>
#include <cmath>
#include <fstream>

// ─────────────────────────────────────────────
//  ColoredMapCreator
// ─────────────────────────────────────────────
class ColoredMapCreator {
public:
    ColoredMapCreator() : nh_("~"), tf_buffer_(ros::Duration(1000.0)) {
        nh_.param<std::string>("bag_path",       bag_path_,       "");
        nh_.param<std::string>("lidar_topic",    lidar_topic_,    "/velodyne_points_cropped");
        nh_.param<std::string>("image_topic",    image_topic_,    "/image_jpeg/compressed");
        nh_.param<bool>("use_color", use_color_, true);
        nh_.param<std::string>("map_frame",      map_frame_,      "map");
        nh_.param<std::string>("velodyne_frame", velodyne_frame_, "velodyne");
        nh_.param<std::string>("camera_frame",   camera_frame_,   "Camera");
        std::string default_save_dir = ros::package::getPath("mapping") + "/pcd";
        nh_.param<std::string>("save_dir",       save_dir_,       default_save_dir);

        // Camera intrinsics
        double fx, fy, cx, cy;
        nh_.param<double>("fx", fx, 320.0);
        nh_.param<double>("fy", fy, 320.0);
        nh_.param<double>("cx", cx, 320.0);
        nh_.param<double>("cy", cy, 240.0);
        camera_matrix_ = (cv::Mat_<double>(3, 3) <<
            fx,  0.0, cx,
            0.0, fy,  cy,
            0.0, 0.0, 1.0);

        double k1, k2, p1, p2, k3;
        nh_.param<double>("k1", k1, 0.0);
        nh_.param<double>("k2", k2, 0.0);
        nh_.param<double>("p1", p1, 0.0);
        nh_.param<double>("p2", p2, 0.0);
        nh_.param<double>("k3", k3, 0.0);
        dist_coeffs_ = (cv::Mat_<double>(5, 1) << k1, k2, p1, p2, k3);

        // Voxel sizes
        nh_.param<float>("voxel_size_local",  voxel_size_local_,  0.2f);
        nh_.param<float>("voxel_size_global", voxel_size_global_, 0.1f);

        // FastGICP settings
        nh_.param<bool>("use_gicp",       use_gicp_,       true);
        nh_.param<int>("gicp_threads",    gicp_threads_,   4);
        nh_.param<int>("gicp_neighbors",  gicp_neighbors_, 20);
        nh_.param<float>("gicp_max_dist", gicp_max_dist_,  1.5f);
        nh_.param<float>("gicp_epsilon",  gicp_epsilon_,   1e-3f);
        nh_.param<int>("gicp_min_map_size", gicp_min_map_size_, 500);

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

        global_map_.reset(new pcl::PointCloud<pcl::PointXYZRGB>());
        global_map_intensity_.reset(new pcl::PointCloud<pcl::PointXYZI>());
        global_map_xyz_.reset(new pcl::PointCloud<pcl::PointXYZ>());


        // gicp_.setNumThreads(gicp_threads_); // Not needed/supported in standard PCL GICP API directly this way
        gicp_.setCorrespondenceRandomness(gicp_neighbors_);
        gicp_.setMaxCorrespondenceDistance(gicp_max_dist_);
        gicp_.setTransformationEpsilon(gicp_epsilon_);
        gicp_.setMaximumIterations(15);  // Limit iterations to speed up convergence significantly

        ROS_INFO("[ColoredMapCreator] Parameters loaded:");
        ROS_INFO("  bag_path      : %s", bag_path_.c_str());
        ROS_INFO("  save_dir      : %s", save_dir_.c_str());
        ROS_INFO("  lidar_topic   : %s", lidar_topic_.c_str());
        ROS_INFO("  image_topic   : %s", image_topic_.c_str());
        ROS_INFO("  map_frame     : %s  vel_frame: %s  cam_frame: %s",
                 map_frame_.c_str(), velodyne_frame_.c_str(), camera_frame_.c_str());
        ROS_INFO("  fx=%.1f fy=%.1f cx=%.1f cy=%.1f", fx, fy, cx, cy);
        ROS_INFO("  voxel_local=%.2f  voxel_global=%.2f", voxel_size_local_, voxel_size_global_);
        ROS_INFO("  GICP threads=%d  neighbors=%d  max_dist=%.2f  min_map=%d",
                 gicp_threads_, gicp_neighbors_, gicp_max_dist_, gicp_min_map_size_);
        ROS_INFO("  time_sync_threshold=%.3f s", time_sync_threshold_);
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

        // ── PASS 1: TF buffering ──────────────────────────────────
        ROS_INFO("Pass 1: Reading TFs...");
        {
            std::vector<std::string> topics = {"/tf", "/tf_static", "tf", "tf_static"};
            rosbag::View view(bag, rosbag::TopicQuery(topics));
            std::set<std::tuple<ros::Time, std::string, std::string>> seen;

            for (const rosbag::MessageInstance& m : view) {
                if (!m.isType<tf2_msgs::TFMessage>()) continue;
                auto tf_msg = m.instantiate<tf2_msgs::TFMessage>();

                for (const auto& t : tf_msg->transforms) {
                    auto key = std::make_tuple(t.header.stamp, t.header.frame_id, t.child_frame_id);
                    if (seen.find(key) == seen.end()) {
                        bool is_static = (m.getTopic() == "/tf_static" || m.getTopic() == "tf_static");
                        tf_buffer_.setTransform(t, "bag_source", is_static);
                        seen.insert(key);
                    }
                }
            }
        }
        ROS_INFO("TF Buffer populated.");

        // ── PASS 2: Image caching ─────────────────────────────────
        std::map<ros::Time, sensor_msgs::CompressedImage::ConstPtr> image_map;
        if (use_color_) {
            ROS_INFO("Pass 2: Reading images...");
            rosbag::View view(bag, rosbag::TopicQuery(std::vector<std::string>{image_topic_}));
            for (const rosbag::MessageInstance& m : view) {
                auto msg = m.instantiate<sensor_msgs::CompressedImage>();
                if (msg) image_map[msg->header.stamp] = msg;
            }
            ROS_INFO("Cached %lu images.", image_map.size());
        } else {
            ROS_INFO("Pass 2: Skipped image caching (use_color is false)");
        }

        // ── PASS 3: LiDAR matching + projection ──────────────────
        ROS_INFO("Pass 3: Processing LiDAR frames...");
        rosbag::View lidar_view(bag, rosbag::TopicQuery(std::vector<std::string>{lidar_topic_}));
        size_t total_frames    = lidar_view.size();
        int    processed_frames = 0;

        for (const rosbag::MessageInstance& m : lidar_view) {
            auto cloud_msg = m.instantiate<sensor_msgs::PointCloud2>();
            if (!cloud_msg) continue;

            ros::Time lidar_time = cloud_msg->header.stamp;

            // ── Find closest image ──
            sensor_msgs::CompressedImage::ConstPtr closest_img;
            if (use_color_) {
                if (image_map.empty()) {
                    // No images available: fall through with null image → intensity-only accumulation
                    ROS_WARN_THROTTLE(10, "No images cached. Accumulating LiDAR intensity points only.");
                } else {
                    auto it = image_map.lower_bound(lidar_time);
                    ros::Time closest_time;

                    if (it == image_map.end()) {
                        auto prev = std::prev(it);
                        closest_time = prev->first;  closest_img = prev->second;
                    } else if (it == image_map.begin()) {
                        closest_time = it->first;    closest_img = it->second;
                    } else {
                        auto prev = std::prev(it);
                        if ((it->first - lidar_time).toSec() < (lidar_time - prev->first).toSec()) {
                            closest_time = it->first;    closest_img = it->second;
                        } else {
                            closest_time = prev->first;  closest_img = prev->second;
                        }
                    }

                    // If time gap too large, drop the image match but keep the LiDAR frame
                    if (std::abs((closest_time - lidar_time).toSec()) > time_sync_threshold_) {
                        closest_img = nullptr;
                    }
                }
            }

            // ── Process frame ──
            processFrame(cloud_msg, closest_img);

            ++processed_frames;

            if (processed_frames % log_interval_ == 0 || (size_t)processed_frames == total_frames) {
                ROS_INFO("Progress: %.1f%% (%d/%lu)  RGB: %lu pts, Intensity: %lu pts",
                         100.0 * processed_frames / total_frames,
                         processed_frames, total_frames, global_map_->size(), global_map_intensity_->size());
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
            // Final global downsampling
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

        // Fallback: if neither RGB nor intensity map was produced, save raw XYZ map
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
    //  Per-frame processing
    // ─────────────────────────────────────────
    void processFrame(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg,
                      const sensor_msgs::CompressedImage::ConstPtr& img_msg) {

        ros::Time lidar_time   = cloud_msg->header.stamp;

        // ── TF lookup ─────────────────────────────────────────────
        geometry_msgs::TransformStamped tf_cam_vel, tf_map_vel_lidar, tf_map_vel_img;
        Eigen::Affine3d T_cam_vel = Eigen::Affine3d::Identity();
        Eigen::Affine3d T_map_vel_img = Eigen::Affine3d::Identity();
        try {
            if (img_msg) {
                ros::Time img_time = img_msg->header.stamp;
                tf_cam_vel = tf_buffer_.lookupTransform(camera_frame_, velodyne_frame_, img_time);
                T_cam_vel = tf2::transformToEigen(tf_cam_vel);

                tf_map_vel_img = tf_buffer_.lookupTransform(map_frame_, velodyne_frame_, img_time);
                T_map_vel_img = tf2::transformToEigen(tf_map_vel_img);
            }
            tf_map_vel_lidar = tf_buffer_.lookupTransform(map_frame_,    velodyne_frame_, lidar_time);
        } catch (tf2::TransformException& ex) {
            ROS_WARN_THROTTLE(1, "TF lookup failed: %s", ex.what());
            return;
        }

        Eigen::Affine3d T_map_vel = tf2::transformToEigen(tf_map_vel_lidar);  // map <- velodyne at lidar time


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

        // ── FastGICP: Refine T_map_vel against global map ────────
        Eigen::Affine3d T_map_vel_refined = T_map_vel;

        if (use_gicp_ && global_map_xyz_->size() >= (size_t)gicp_min_map_size_) {

            // Build xyz-only source cloud from the pre-filtered XYZI scan
            pcl::PointCloud<pcl::PointXYZ>::Ptr scan_xyz(new pcl::PointCloud<pcl::PointXYZ>());
            scan_xyz->reserve(scan_vel->size());
            for (const auto& p : scan_vel->points)
                scan_xyz->emplace_back(p.x, p.y, p.z);

            // Transform the source scan to map frame using TF guess
            pcl::PointCloud<pcl::PointXYZ>::Ptr scan_map_guess(new pcl::PointCloud<pcl::PointXYZ>());
            pcl::transformPointCloud(*scan_xyz, *scan_map_guess,
                                     T_map_vel.matrix().cast<float>());

            // Run FastGICP: source = current scan in map-guess, target = accumulated map
            // By pre-transforming the source, the initial guess is Identity,
            // and the output transform is the small correction to apply.
            gicp_.setInputSource(scan_map_guess);
            gicp_.setInputTarget(global_map_xyz_);

            pcl::PointCloud<pcl::PointXYZ> aligned;
            gicp_.align(aligned);

            if (gicp_.hasConverged()) {
                Eigen::Matrix4f correction = gicp_.getFinalTransformation();
                Eigen::Affine3d T_correction(correction.cast<double>());
                // Final transform: correction * TF_initial
                T_map_vel_refined = T_correction * T_map_vel;
            } else {
                ROS_WARN_THROTTLE(2, "FastGICP did not converge. Using TF initial guess.");
            }
        }

        if (img_msg) {
            // ── Decode image ──────────────────────────────────────────
            cv::Mat image = cv::imdecode(cv::Mat(img_msg->data), cv::IMREAD_COLOR);
            if (image.empty()) { ROS_ERROR("Failed to decode image."); return; }

            // ── [KEY FIX] Derive projection from GICP-refined pose ───
            //
            // Previous (wrong):  project using T_cam_vel (TF-based, consistent with T_map_vel)
            //                     but accumulate positions using T_map_vel_refined (GICP-corrected)
            //                     → mismatch: different reference pose for color vs position
            //
            // Fixed:             convert velodyne points → MAP frame using T_map_vel_refined,
            //                    then project MAP-frame points using the camera pose derived
            //                    from the SAME T_map_vel_refined.
            //
            //  T_map_cam_refined = T_map_vel_refined * T_cam_vel⁻¹
            //  T_cam_map_refined = T_map_cam_refined⁻¹ = T_cam_vel * T_map_vel_refined⁻¹
            //  T_opt_map_refined = T_ros_to_optical * T_cam_map_refined
            //
            // Both position (T_map_vel_refined * p_vel) and color sampling
            // (project T_map_vel_refined * p_vel with T_opt_map_refined) now share
            // exactly the same corrected transform.

            //  ROS camera frame:     X-forward, Y-left,  Z-up
            //  OpenCV optical frame: X-right,   Y-down,  Z-forward
            Eigen::Matrix3d R_ros_to_optical;
            R_ros_to_optical <<  0, -1,  0,
                                  0,  0, -1,
                                  1,  0,  0;
            Eigen::Affine3d T_ros_to_optical = Eigen::Affine3d::Identity();
            T_ros_to_optical.linear() = R_ros_to_optical;

            // Apply GICP correction to camera pose at image time
            Eigen::Affine3d delta_T = T_map_vel_refined * T_map_vel.inverse();
            Eigen::Affine3d T_map_vel_img_refined = delta_T * T_map_vel_img;

            // Camera pose in map at IMAGE time (GICP-corrected)
            // Inverse (camera ← map at image time):
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

            // ── Pre-transform all scan points to MAP frame ────────────
            // Use T_map_vel_refined so projection and position share the same base.
            std::vector<cv::Point3f> obj_pts_map;   // MAP-frame points for projection
            std::vector<Eigen::Vector3d> map_pts;   // same points as Eigen, for accumulation
            std::vector<int>         obj_idx;

            obj_pts_map.reserve(scan_vel->size());
            map_pts.reserve(scan_vel->size());
            obj_idx.reserve(scan_vel->size());

            for (int i = 0; i < (int)scan_vel->size(); ++i) {
                const auto& p = scan_vel->points[i];
                Eigen::Vector3d p_map = T_map_vel_refined * Eigen::Vector3d(p.x, p.y, p.z);

                // Visibility check in corrected optical frame (Z > 0 = in front of camera)
                Eigen::Vector3d p_opt = T_opt_map_refined * p_map;
                if (p_opt.z() > 0) {
                    obj_pts_map.emplace_back((float)p_map.x(), (float)p_map.y(), (float)p_map.z());
                    map_pts.push_back(p_map);
                    obj_idx.push_back(i);
                }
            }

            if (!obj_pts_map.empty()) {
                // ── Project MAP-frame points onto image plane ─────────────
                std::vector<cv::Point2f> img_pts;
                cv::projectPoints(obj_pts_map, rvec, tvec, camera_matrix_, dist_coeffs_, img_pts);

                // ── Accumulate colored / intensity points ─────────────────
                for (size_t i = 0; i < img_pts.size(); ++i) {
                    int u = (int)std::round(img_pts[i].x);
                    int v = (int)std::round(img_pts[i].y);
                    if (u < 0 || u >= image.cols || v < 0 || v >= image.rows) continue;

                    const auto& src = scan_vel->points[obj_idx[i]];
                    const Eigen::Vector3d& p_map = map_pts[i];

                    // 1. 라이다 Intensity 구간 계산 (0~255를 8등분: 32씩)
                    int region_pt = std::min((int)src.intensity / 32, 7);

                    cv::Vec3b best_color = image.at<cv::Vec3b>(v, u);
                    bool match_found = false;

                    // 2. 현재 픽셀의 밝기 구간 계산 (BGR -> Gray 변환)
                    uint8_t gray = 0.299 * best_color[2] + 0.587 * best_color[1] + 0.114 * best_color[0];
                    int region_px = std::min(gray / 32, 7);

                    if (region_pt == region_px) {
                        match_found = true;
                    } else {
                        // 3. 일치하지 않으면 주변 5x5 영역 탐색
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
                    cp.r = best_color[2];   cp.g = best_color[1];   cp.b = best_color[0];  // BGR → RGB
                    global_map_->push_back(cp);

                    pcl::PointXYZI ip;
                    ip.x = p_map.x();  ip.y = p_map.y();  ip.z = p_map.z();
                    ip.intensity = src.intensity;
                    global_map_intensity_->push_back(ip);
                }

            }
        } else {
            // 컬러를 사용하지 않을 경우, 모든 포인트를 강도(Intensity) 지도로 바로 추가
            for (const auto& p : scan_vel->points) {
                Eigen::Vector3d p_map = T_map_vel_refined * Eigen::Vector3d(p.x, p.y, p.z);
                
                pcl::PointXYZI ip;
                ip.x = p_map.x();  ip.y = p_map.y();  ip.z = p_map.z();
                ip.intensity = p.intensity;
                global_map_intensity_->push_back(ip);
            }
        }



        // ── Maintain a lightweight XYZ map for GICP target ───────
        for (const auto& p : scan_vel->points) {
            Eigen::Vector3d pm = T_map_vel_refined * Eigen::Vector3d(p.x, p.y, p.z);
            global_map_xyz_->emplace_back(pm.x(), pm.y(), pm.z());
        }

        // Periodically downsample the GICP target to keep it lean
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
    bool use_color_;
    std::string map_frame_, velodyne_frame_, camera_frame_;
    std::string save_dir_;

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
    int   log_interval_;

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
