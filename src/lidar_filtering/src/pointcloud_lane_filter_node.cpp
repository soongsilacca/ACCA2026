#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class PointCloudLaneFilterNode {
public:
    PointCloudLaneFilterNode() {
        ros::NodeHandle nh("~");

        // Parameters
        nh.param("min_intensity", min_intensity_, 20.0f); // Default intensity threshold

        std::string input_topic, output_topic;
        nh.param<std::string>("input_topic", input_topic, "/velodyne_points_ground");
        nh.param<std::string>("output_topic", output_topic, "/velodyne_points_lane");

        sub_ = nh_.subscribe(input_topic, 1, &PointCloudLaneFilterNode::pointCloudCallback, this);
        pub_ = nh_.advertise<sensor_msgs::PointCloud2>(output_topic, 1);
        
        ROS_INFO("PointCloudLaneFilterNode initialized.");
        ROS_INFO("Filtering points with intensity >= %.2f to extract lane/road features.", min_intensity_);
    }

private:
    void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::fromROSMsg(*msg, *cloud);

        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>());
        
        for (const auto& point : cloud->points) {
            // Keep points that have high intensity (e.g., lane markings on the road)
            if (point.intensity >= min_intensity_) {
                cloud_filtered->points.push_back(point);
            }
        }
        
        cloud_filtered->width = cloud_filtered->points.size();
        cloud_filtered->height = 1;
        cloud_filtered->is_dense = true;

        sensor_msgs::PointCloud2 output_msg;
        pcl::toROSMsg(*cloud_filtered, output_msg);
        output_msg.header = msg->header; // keep the original header

        pub_.publish(output_msg);
    }

    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    ros::Publisher pub_;

    float min_intensity_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "pointcloud_lane_filter_node");
    PointCloudLaneFilterNode node;
    ros::spin();
    return 0;
}
