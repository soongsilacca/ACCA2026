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

class PointCloudClusterNode {
public:
    PointCloudClusterNode() {
        ros::NodeHandle nh("~");

        // Parameters
        nh.param("min_cluster_size", min_cluster_size_, 10);
        nh.param("max_cluster_size", max_cluster_size_, 10000);
        
        // Adaptive clustering regions
        // Region 1: 0 to 15m, Tolerance: 0.5m
        // Region 2: 15 to 30m, Tolerance: 1.0m
        // Region 3: 30 to 45m, Tolerance: 1.5m
        // Region 4: 45m+, Tolerance: 2.0m

        std::string input_topic, marker_topic, pose_topic;
        nh.param<std::string>("input_topic", input_topic, "/velodyne_points_filtered");
        nh.param<std::string>("marker_topic", marker_topic, "/clusters_markers");
        nh.param<std::string>("pose_topic", pose_topic, "/clusters_poses");

        sub_ = nh_.subscribe(input_topic, 1, &PointCloudClusterNode::pointCloudCallback, this);
        pub_markers_ = nh_.advertise<visualization_msgs::MarkerArray>(marker_topic, 1);
        pub_poses_ = nh_.advertise<geometry_msgs::PoseArray>(pose_topic, 1);
        
        ROS_INFO("PointCloudClusterNode initialized with Adaptive Clustering.");
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

    void extractAdaptiveClusters(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>& all_clusters) {
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
                
                // HDL-32E Characteristics
                // Vertical Field of View: +10.67 to -30.67 degrees (41.33 deg total)
                // Lasers: 32 channels -> Vertical Resolution (V_RES) = ~1.33 degrees
                // Horizontal Resolution (H_RES) = ~0.2 degrees (depends on RPM)
                const float V_RES = 1.33f * M_PI / 180.0f; // 0.0232 rad
                
                // 3D Distance from Lidar origin
                float r = std::sqrt(cloud->points[curr_idx].x * cloud->points[curr_idx].x + 
                                    cloud->points[curr_idx].y * cloud->points[curr_idx].y + 
                                    cloud->points[curr_idx].z * cloud->points[curr_idx].z);
                
                // Maximum expected gap between points on the same object is defined by the vertical resolution.
                // Expected vertical gap at distance r: r * tan(V_RES)
                // We apply a multiplier (e.g. 1.5) to account for non-perpendicular surfaces and noise.
                float expected_v_gap = r * std::tan(V_RES);
                float base_tolerance = 0.15f; // minimum physical distance to cluster points (e.g. 15cm)
                
                float tolerance = std::max(base_tolerance, expected_v_gap * 1.5f);

                std::vector<int> nn_indices;
                std::vector<float> nn_dists;
                tree->radiusSearch(cloud->points[curr_idx], tolerance, nn_indices, nn_dists);

                for (size_t j = 1; j < nn_indices.size(); ++j) { // Start from 1 to skip the point itself
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

    void publishClusters(const std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>& clusters, const std_msgs::Header& header) {
        visualization_msgs::MarkerArray marker_array;
        geometry_msgs::PoseArray pose_array;
        pose_array.header = header;

        int id = 0;

        // Delete previous markers
        visualization_msgs::Marker delete_marker;
        delete_marker.action = visualization_msgs::Marker::DELETEALL;
        marker_array.markers.push_back(delete_marker);

        for (const auto& cluster : clusters) {
            pcl::PointXYZI min_pt, max_pt;
            pcl::getMinMax3D(*cluster, min_pt, max_pt);

            // Center of the bounding box
            float cx = (min_pt.x + max_pt.x) / 2.0f;
            float cy = (min_pt.y + max_pt.y) / 2.0f;
            float cz = (min_pt.z + max_pt.z) / 2.0f;

            // Size of the bounding box
            float dx = max_pt.x - min_pt.x;
            float dy = max_pt.y - min_pt.y;
            float dz = max_pt.z - min_pt.z;

            // Filter out unreasonable clusters
            if (dx < 0.1 || dy < 0.1 || dz < 0.1) continue;

            // Add pose
            geometry_msgs::Pose pose;
            pose.position.x = cx;
            pose.position.y = cy;
            pose.position.z = cz;
            pose.orientation.w = 1.0;
            pose_array.poses.push_back(pose);

            // Add marker
            visualization_msgs::Marker marker;
            marker.header = header;
            marker.ns = "clusters";
            marker.id = id++;
            marker.type = visualization_msgs::Marker::CUBE;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose = pose;
            marker.scale.x = dx;
            marker.scale.y = dy;
            marker.scale.z = dz;

            // Translucent color
            marker.color.r = 0.0f;
            marker.color.g = 1.0f;
            marker.color.b = 0.0f;
            marker.color.a = 0.5f; // Translucent
            
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
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "pointcloud_cluster_node");
    PointCloudClusterNode node;
    ros::spin();
    return 0;
}
