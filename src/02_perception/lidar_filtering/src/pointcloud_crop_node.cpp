#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/crop_box.h>
#include <vector>

// Custom Velodyne Point Type matching FAST_LIO and the raw bag data
namespace velodyne_ros {
  struct EIGEN_ALIGN16 Point {
      PCL_ADD_POINT4D;
      float intensity;
      float time;
      uint16_t ring;
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
}  // namespace velodyne_ros

POINT_CLOUD_REGISTER_POINT_STRUCT(velodyne_ros::Point,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (float, time, time)
    (uint16_t, ring, ring)
)

class PointCloudCropNode {
public:
    PointCloudCropNode() {
        ros::NodeHandle nh("~");

        // Parameters for CropBox filter
        nh.param("min_x", min_x_, -10.0f);
        nh.param("max_x", max_x_, 10.0f);
        nh.param("min_y", min_y_, -10.0f);
        nh.param("max_y", max_y_, 10.0f);
        nh.param("min_z", min_z_, -2.0f);
        nh.param("max_z", max_z_, 3.0f);
        nh.param("negative", negative_, false);

        std::string input_topic, output_topic;
        nh.param<std::string>("input_topic", input_topic, "/velodyne_points");
        nh.param<std::string>("output_topic", output_topic, "/velodyne_points_cropped");

        sub_ = nh_.subscribe(input_topic, 1, &PointCloudCropNode::pointCloudCallback, this);
        pub_ = nh_.advertise<sensor_msgs::PointCloud2>(output_topic, 1);
        
        ROS_INFO("PointCloudCropNode initialized. Preserving all original fields using index filtering.");
    }

private:
    void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
        // 1. Read as custom type to preserve all fields (intensity, ring, time)
        pcl::PointCloud<velodyne_ros::Point>::Ptr cloud_custom(new pcl::PointCloud<velodyne_ros::Point>());
        pcl::fromROSMsg(*msg, *cloud_custom);

        // 2. Read as standard type just for CropBox filtering
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_standard(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::fromROSMsg(*msg, *cloud_standard);

        // 3. Filter using CropBox on standard type to get indices
        std::vector<int> indices;
        pcl::CropBox<pcl::PointXYZI> cropFilter;
        cropFilter.setInputCloud(cloud_standard);
        cropFilter.setMin(Eigen::Vector4f(min_x_, min_y_, min_z_, 1.0f));
        cropFilter.setMax(Eigen::Vector4f(max_x_, max_y_, max_z_, 1.0f));
        cropFilter.setNegative(negative_);
        cropFilter.filter(indices);

        // 4. Create output cloud and copy points from custom cloud using filtered indices
        pcl::PointCloud<velodyne_ros::Point>::Ptr cloud_filtered(new pcl::PointCloud<velodyne_ros::Point>());
        cloud_filtered->header = cloud_custom->header;
        cloud_filtered->reserve(indices.size());

        for (int idx : indices) {
            cloud_filtered->push_back(cloud_custom->points[idx]);
        }

        // 5. Publish
        sensor_msgs::PointCloud2 output_msg;
        pcl::toROSMsg(*cloud_filtered, output_msg);
        output_msg.header = msg->header; // keep the original header
        
        // Fallback for zero timestamp if needed
        if (output_msg.header.stamp.isZero()) {
            output_msg.header.stamp = ros::Time::now();
        }

        pub_.publish(output_msg);
    }

    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    ros::Publisher pub_;

    float min_x_, max_x_, min_y_, max_y_, min_z_, max_z_;
    bool negative_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "pointcloud_crop_node");
    PointCloudCropNode node;
    ros::spin();
    return 0;
}
