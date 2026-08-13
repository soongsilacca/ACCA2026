#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <cmath>
#include <vector>
#include <algorithm>

// Define a struct to hold polar coordinates for each point
struct PointXYZIRT {
    pcl::PointXYZI point;
    float radius;
    float theta; // azimuth in degrees
    size_t radial_div; // index of the ray
};

class PointCloudRayFilterNode {
public:
    PointCloudRayFilterNode() {
        ros::NodeHandle nh("~");

        // Ray Ground Filter parameters
        nh.param("sensor_height", sensor_height_, 1.8f); // Sensor height from ground (meters)
        nh.param("local_max_slope", local_max_slope_, 8.0f); // Max local slope (degrees)
        nh.param("general_max_slope", general_max_slope_, 5.0f); // Max general slope from origin (degrees)
        nh.param("min_height_threshold", min_height_threshold_, 0.1f); // Min height diff to consider as obstacle (meters)
        nh.param("radial_divider_angle", radial_divider_angle_, 0.18f); // Angular resolution for rays (degrees)
        nh.param("publish_ground", publish_ground_, false);

        radial_dividers_num_ = std::ceil(360.0f / radial_divider_angle_);

        std::string input_topic, output_ground_topic, output_no_ground_topic;
        nh.param<std::string>("input_topic", input_topic, "/velodyne_points");
        // Output for ground points
        nh.param<std::string>("output_ground_topic", output_ground_topic, "/velodyne_points_ground");
        // Output for non-ground (obstacle) points - this is usually the main filtered output
        nh.param<std::string>("output_no_ground_topic", output_no_ground_topic, "/velodyne_points_filtered");

        sub_ = nh_.subscribe(input_topic, 1, &PointCloudRayFilterNode::pointCloudCallback, this);
        if (publish_ground_) {
            pub_ground_ = nh_.advertise<sensor_msgs::PointCloud2>(output_ground_topic, 1);
        }
        pub_no_ground_ = nh_.advertise<sensor_msgs::PointCloud2>(output_no_ground_topic, 1);
        
        ROS_INFO("PointCloudRayFilterNode initialized with true Ray Ground Filtering.");
        ROS_INFO("Parameters: sensor_height=%.2f, radial_divider_angle=%.2f", sensor_height_, radial_divider_angle_);
    }

private:
    void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::fromROSMsg(*msg, *cloud);

        std::vector<std::vector<PointXYZIRT>> radial_divisions(radial_dividers_num_);

        // 1. Organize points into rays based on azimuth angle
        for (const auto& pt : cloud->points) {
            float radius = std::sqrt(pt.x * pt.x + pt.y * pt.y);
            if (radius < 0.1f) continue; // ignore points too close to origin (lidar self-reflection)
            
            float theta = std::atan2(pt.y, pt.x) * 180.0f / M_PI;
            if (theta < 0.0f) theta += 360.0f;
            
            size_t radial_div = std::floor(theta / radial_divider_angle_);
            if (radial_div >= radial_dividers_num_) radial_div = radial_dividers_num_ - 1;

            PointXYZIRT p;
            p.point = pt;
            p.radius = radius;
            p.theta = theta;
            p.radial_div = radial_div;

            radial_divisions[radial_div].push_back(p);
        }

        pcl::PointCloud<pcl::PointXYZI>::Ptr ground_cloud(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::PointCloud<pcl::PointXYZI>::Ptr no_ground_cloud(new pcl::PointCloud<pcl::PointXYZI>());

        // 2. Process each ray individually
        for (size_t i = 0; i < radial_dividers_num_; ++i) {
            auto& ray = radial_divisions[i];
            if (ray.empty()) continue;

            // Sort points in the ray by radius (distance from sensor)
            std::sort(ray.begin(), ray.end(), [](const PointXYZIRT& a, const PointXYZIRT& b) {
                return a.radius < b.radius;
            });

            // Initialize reference height (ground level at sensor origin)
            float prev_radius = 0.0f;
            float prev_height = -sensor_height_;

            for (size_t j = 0; j < ray.size(); ++j) {
                float current_radius = ray[j].radius;
                float current_height = ray[j].point.z;

                float dr = current_radius - prev_radius;
                float dz = current_height - prev_height;

                bool is_ground = false;

                // Handle points that are vertically stacked (same radius)
                if (dr < 0.05f) {
                    if (dz < min_height_threshold_) {
                        is_ground = true;
                    }
                } else {
                    float local_slope = std::atan2(dz, dr) * 180.0f / M_PI;
                    float general_slope = std::atan2(current_height + sensor_height_, current_radius) * 180.0f / M_PI;

                    if (local_slope < local_max_slope_ && general_slope < general_max_slope_) {
                        is_ground = true;
                    } else if (dz < min_height_threshold_) {
                        // Negative slope (downhill) or very small height difference
                        is_ground = true;
                    }
                }

                if (is_ground) {
                    if (publish_ground_) {
                        ground_cloud->points.push_back(ray[j].point);
                    }
                    // Update reference ground point
                    prev_radius = current_radius;
                    prev_height = current_height;
                } else {
                    no_ground_cloud->points.push_back(ray[j].point);
                }
            }
        }

        // 3. Publish results
        if (publish_ground_ && !ground_cloud->points.empty()) {
            ground_cloud->width = ground_cloud->points.size();
            ground_cloud->height = 1;
            ground_cloud->is_dense = true;
            sensor_msgs::PointCloud2 ground_msg;
            pcl::toROSMsg(*ground_cloud, ground_msg);
            ground_msg.header = msg->header;
            pub_ground_.publish(ground_msg);
        }

        // Publish every input frame, including an empty obstacle cloud.  Downstream
        // synchronizers use this topic as a sensor clock; dropping an all-ground
        // frame stalls the complete camera/LiDAR/ego inference pipeline.
        no_ground_cloud->width = no_ground_cloud->points.size();
        no_ground_cloud->height = 1;
        no_ground_cloud->is_dense = true;
        sensor_msgs::PointCloud2 no_ground_msg;
        pcl::toROSMsg(*no_ground_cloud, no_ground_msg);
        no_ground_msg.header = msg->header;
        pub_no_ground_.publish(no_ground_msg);
    }

    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    ros::Publisher pub_ground_;
    ros::Publisher pub_no_ground_;

    float sensor_height_;
    float local_max_slope_;
    float general_max_slope_;
    float min_height_threshold_;
    float radial_divider_angle_;
    bool publish_ground_;
    size_t radial_dividers_num_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "pointcloud_ray_filter_node");
    PointCloudRayFilterNode node;
    ros::spin();
    return 0;
}
