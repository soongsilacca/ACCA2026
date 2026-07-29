#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PoseArray.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/common.h>
#include <cmath>
#include <vector>
#include <algorithm>

/**
 * Distance-Adaptive Euclidean Clustering with L-Shape 3D Bounding Box Fitting
 */
class ShapeEstimationClusterNode {
public:
    ShapeEstimationClusterNode() {
        ros::NodeHandle nh("~");

        // Parameters
        nh.param("min_cluster_size", min_cluster_size_, 3);
        nh.param("max_cluster_size", max_cluster_size_, 10000);
        nh.param("base_tolerance", base_tolerance_, 0.45f); // Base cluster distance tolerance (m)
        nh.param("tolerance_slope", tolerance_slope_, 0.02f); // Distance-adaptive slope for sparse LiDAR

        std::string input_topic, marker_topic, pose_topic;
        nh.param<std::string>("input_topic", input_topic, "/velodyne_points_filtered");
        nh.param<std::string>("marker_topic", marker_topic, "/clusters_markers");
        nh.param<std::string>("pose_topic", pose_topic, "/clusters_poses");

        sub_ = nh_.subscribe(input_topic, 1, &ShapeEstimationClusterNode::pointCloudCallback, this);
        pub_markers_ = nh_.advertise<visualization_msgs::MarkerArray>(marker_topic, 1);
        pub_poses_ = nh_.advertise<geometry_msgs::PoseArray>(pose_topic, 1);
        
        ROS_INFO("ShapeEstimationClusterNode initialized.");
        ROS_INFO("Params: base_tolerance=%.2fm, min_size=%d", base_tolerance_, min_cluster_size_);
    }

private:
    void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::fromROSMsg(*msg, *cloud);

        if (cloud->empty()) return;

        std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> clusters;
        extractAdaptiveClusters(cloud, clusters);
        publishClusters(clusters, msg->header);
    }

    void extractAdaptiveClusters(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
                                std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>& all_clusters) {
        if (cloud->empty()) return;

        pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
        tree->setInputCloud(cloud);

        std::vector<bool> processed(cloud->points.size(), false);

        for (size_t i = 0; i < cloud->points.size(); ++i) {
            if (processed[i]) continue;

            std::vector<int> seed_queue;
            int sq_idx = 0;
            seed_queue.push_back(i);
            processed[i] = true;

            while (sq_idx < seed_queue.size()) {
                int curr_idx = seed_queue[sq_idx];
                
                // Distance from LiDAR origin
                float r = std::sqrt(cloud->points[curr_idx].x * cloud->points[curr_idx].x + 
                                    cloud->points[curr_idx].y * cloud->points[curr_idx].y + 
                                    cloud->points[curr_idx].z * cloud->points[curr_idx].z);
                
                // VLP-16 2.0 deg vertical resolution gap scaling
                const float V_RES = 2.0f * M_PI / 180.0f;
                float expected_v_gap = r * std::tan(V_RES);
                
                // Adaptive tolerance scaling with distance
                float tolerance = std::max(base_tolerance_, base_tolerance_ + r * tolerance_slope_ + expected_v_gap * 1.5f);

                std::vector<int> nn_indices;
                std::vector<float> nn_dists;
                tree->radiusSearch(cloud->points[curr_idx], tolerance, nn_indices, nn_dists);

                for (size_t j = 1; j < nn_indices.size(); ++j) {
                    if (!processed[nn_indices[j]]) {
                        seed_queue.push_back(nn_indices[j]);
                        processed[nn_indices[j]] = true;
                    }
                }
                sq_idx++;
            }

            if (seed_queue.size() >= min_cluster_size_ && seed_queue.size() <= max_cluster_size_) {
                pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZI>);
                for (int idx : seed_queue) {
                    cloud_cluster->points.push_back(cloud->points[idx]);
                }
                cloud_cluster->width = cloud_cluster->points.size();
                cloud_cluster->height = 1;
                cloud_cluster->is_dense = true;
                all_clusters.push_back(cloud_cluster);
            }
        }
    }

    /**
     * L-Shape Fitting Algorithm:
     * Fits an optimal Oriented Bounding Box (OBB) by searching orientation theta that minimizes bounding area.
     */
    void fitLShapeBoundingBox(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cluster,
                              geometry_msgs::Pose& pose,
                              geometry_msgs::Vector3& scale) {
        float min_area = 1e9f;
        float best_theta = 0.0f;
        float best_min_x = 0, best_max_x = 0, best_min_y = 0, best_max_y = 0;

        // Search candidate orientation angle theta from 0 to 90 degrees (3 deg step)
        for (float deg = 0.0f; deg < 90.0f; deg += 3.0f) {
            float theta = deg * M_PI / 180.0f;
            float cos_t = std::cos(theta);
            float sin_t = std::sin(theta);

            float min_x = 1e9f, max_x = -1e9f;
            float min_y = 1e9f, max_y = -1e9f;

            for (const auto& pt : cluster->points) {
                float rx = pt.x * cos_t + pt.y * sin_t;
                float ry = -pt.x * sin_t + pt.y * cos_t;
                if (rx < min_x) min_x = rx;
                if (rx > max_x) max_x = rx;
                if (ry < min_y) min_y = ry;
                if (ry > max_y) max_y = ry;
            }

            float area = (max_x - min_x) * (max_y - min_y);
            if (area < min_area) {
                min_area = area;
                best_theta = theta;
                best_min_x = min_x;
                best_max_x = max_x;
                best_min_y = min_y;
                best_max_y = max_y;
            }
        }

        // Z bounds
        float min_z = 1e9f, max_z = -1e9f;
        for (const auto& pt : cluster->points) {
            if (pt.z < min_z) min_z = pt.z;
            if (pt.z > max_z) max_z = pt.z;
        }

        // Center in rotated frame
        float cx_rot = (best_min_x + best_max_x) / 2.0f;
        float cy_rot = (best_min_y + best_max_y) / 2.0f;

        // Rotate center back to global/sensor frame
        float cos_bt = std::cos(best_theta);
        float sin_bt = std::sin(best_theta);

        pose.position.x = cx_rot * cos_bt - cy_rot * sin_bt;
        pose.position.y = cx_rot * sin_bt + cy_rot * cos_bt;
        pose.position.z = (min_z + max_z) / 2.0f;

        // Orientation Quaternion (Yaw angle = best_theta)
        pose.orientation.x = 0.0;
        pose.orientation.y = 0.0;
        pose.orientation.z = std::sin(best_theta / 2.0f);
        pose.orientation.w = std::cos(best_theta / 2.0f);

        scale.x = std::max(0.3f, best_max_x - best_min_x);
        scale.y = std::max(0.3f, best_max_y - best_min_y);
        scale.z = std::max(0.3f, max_z - min_z);
    }

    void publishClusters(const std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>& clusters,
                         const std_msgs::Header& header) {
        visualization_msgs::MarkerArray marker_array;
        geometry_msgs::PoseArray pose_array;
        pose_array.header = header;

        int id = 0;

        // Delete previous markers
        visualization_msgs::Marker delete_marker;
        delete_marker.action = visualization_msgs::Marker::DELETEALL;
        marker_array.markers.push_back(delete_marker);

        for (const auto& cluster : clusters) {
            geometry_msgs::Pose pose;
            geometry_msgs::Vector3 scale;

            // Fit Oriented Bounding Box (OBB)
            fitLShapeBoundingBox(cluster, pose, scale);

            if (scale.x < 0.05 || scale.y < 0.05 || scale.z < 0.05) continue;

            pose_array.poses.push_back(pose);

            // Create translucent cyan oriented bounding box marker
            visualization_msgs::Marker marker;
            marker.header = header;
            marker.ns = "lidar_clusters";
            marker.id = id++;
            marker.type = visualization_msgs::Marker::CUBE;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose = pose;
            marker.scale = scale;

            // Cyan translucent color
            marker.color.r = 0.0f;
            marker.color.g = 1.0f;
            marker.color.b = 0.8f;
            marker.color.a = 0.55f;
            
            marker.lifetime = ros::Duration(0.1);

            marker_array.markers.push_back(marker);
        }

        pub_markers_.publish(marker_array);
        pub_poses_.publish(pose_array);
    }

    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    ros::Publisher pub_markers_;
    ros::Publisher pub_poses_;

    int min_cluster_size_;
    int max_cluster_size_;
    float base_tolerance_;
    float tolerance_slope_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "shape_estimation_cluster_node");
    ShapeEstimationClusterNode node;
    ros::spin();
    return 0;
}
