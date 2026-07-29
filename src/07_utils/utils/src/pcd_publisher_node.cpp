#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <fstream>
#include <cmath>

int main(int argc, char** argv) {
    ros::init(argc, argv, "pcd_publisher_node");
    ros::NodeHandle nh("~");

    std::string pcd_file;
    std::string frame_id;
    std::string topic_name;
    double publish_rate;

    double anchor_east;
    double anchor_north;

    // Default parameters
    nh.param<std::string>("pcd_file", pcd_file, "my_map.pcd");
    nh.param<std::string>("frame_id", frame_id, "map");
    nh.param<std::string>("topic_name", topic_name, "/pcd_map");
    nh.param<double>("publish_rate", publish_rate, 1.0); // 1 Hz
    nh.param<double>("anchor_east", anchor_east, 302595.0);
    nh.param<double>("anchor_north", anchor_north, 4124145.0);

    // Latch is set to true so new subscribers immediately get the latest map
    ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2>(topic_name, 1, true);

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
    
    // Attempt to load the PCD file
    if (pcl::io::loadPCDFile<pcl::PointXYZI>(pcd_file, *cloud) == -1) {
        ROS_ERROR("Couldn't read file %s. Please check the path.", pcd_file.c_str());
        return -1;
    }
    
    ROS_INFO("Successfully loaded %lu data points from %s", cloud->points.size(), pcd_file.c_str());

    // Check if there is a .utm file right next to it
    double utm_x = 0.0, utm_y = 0.0, utm_z = 0.0;
    bool has_utm = false;
    std::string utm_file = pcd_file + ".utm";
    std::ifstream ifs(utm_file);
    if (ifs.is_open()) {
        if (ifs >> utm_x >> utm_y >> utm_z) {
            has_utm = true;
            ROS_INFO("Found UTM coordinate file: %s", utm_file.c_str());
            ROS_INFO("UTM Origin: Easting=%.6f, Northing=%.6f, Altitude=%.6f", utm_x, utm_y, utm_z);
        }
        ifs.close();
    }

    bool points_in_absolute_utm = false;
    if (!cloud->points.empty()) {
        // If points already have large coordinates (e.g. > 100000m), they are in absolute UTM frame
        if (std::abs(cloud->points[0].x) > 100000.0) {
            points_in_absolute_utm = true;
        }
    }

    if (points_in_absolute_utm) {
        ROS_INFO("Point cloud is in absolute UTM frame. Converting to anchor-local frame...");
        for (auto& pt : cloud->points) {
            pt.x -= anchor_east;
            pt.y -= anchor_north;
        }
        ROS_INFO("Successfully converted points to anchor-local frame.");
    } else if (has_utm) {
        ROS_INFO("Point cloud is in local frame. Shifting to anchor-local frame using UTM origin and anchor...");
        for (auto& pt : cloud->points) {
            pt.x += (utm_x - anchor_east);
            pt.y += (utm_y - anchor_north);
            pt.z += utm_z;
        }
        ROS_INFO("Successfully shifted points to anchor-local frame.");
    } else {
        ROS_WARN("Point cloud is in local frame, but no .utm file was found. Publishing as-is (unaligned).");
    }

    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*cloud, output);
    output.header.frame_id = frame_id;

    ros::Rate loop_rate(publish_rate);
    while (ros::ok()) {
        output.header.stamp = ros::Time::now(); // Update timestamp
        pub.publish(output);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
