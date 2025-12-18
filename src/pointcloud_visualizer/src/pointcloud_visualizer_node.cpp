#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>

class PointCloudVisualizer : public rclcpp::Node {
public:
  PointCloudVisualizer()
  : Node("pointcloud_visualizer")
  {
    publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("pointcloud", 10);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>("/home/acca/ys_map.pcd", *cloud) == -1) {
      RCLCPP_ERROR(this->get_logger(), "Couldn't read PCD file");
      return;
    }

    // /home/hovin/hovin_ws/src/route_planner/resource/2023_kcity.pcd
    // /home/hovin/hovin_ws/src/route_planner/resource/school_map.pcd

    // Voxel Grid Downsampling
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud);
    sor.setLeafSize(0.5f, 0.5f, 0.5f); // leaf size 설정 (0.1m x 0.1m x 0.1m)
    sor.filter(*cloud_filtered);

    sensor_msgs::msg::PointCloud2 output;
    pcl::toROSMsg(*cloud_filtered, output);
    output.header.frame_id = "map";
    publisher_->publish(output);

    RCLCPP_INFO(this->get_logger(), "Published downsampled PointCloud2 message");
  }

private:
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PointCloudVisualizer>());
  rclcpp::shutdown();
  return 0;
}
