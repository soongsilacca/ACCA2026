#include <cmath>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class PointCloudCropNode {
public:
    PointCloudCropNode() : private_nh_("~") {
        private_nh_.param("min_x", min_x_, -20.0);
        private_nh_.param("max_x", max_x_, 50.0);
        private_nh_.param("min_y", min_y_, -30.0);
        private_nh_.param("max_y", max_y_, 30.0);
        private_nh_.param("min_z", min_z_, -3.0);
        private_nh_.param("max_z", max_z_, 3.0);
        private_nh_.param("footprint_min_z", footprint_min_z_, -10.0);
        private_nh_.param("footprint_max_z", footprint_max_z_, 10.0);
        private_nh_.param("remove_footprint", remove_footprint_, true);

        private_nh_.param<std::string>("input_topic", input_topic_, "/velodyne_points");
        private_nh_.param<std::string>("output_topic", output_topic_, "/velodyne_points_cropped");

        std::vector<double> footprint_x;
        std::vector<double> footprint_y;
        if (!private_nh_.getParam("footprint_x", footprint_x) ||
            !private_nh_.getParam("footprint_y", footprint_y) ||
            footprint_x.size() < 3 || footprint_x.size() != footprint_y.size()) {
            footprint_x = {0.053, 0.066, -4.925, -4.947};
            footprint_y = {-1.318, 1.330, 1.310, -1.040};
            ROS_WARN("Invalid footprint parameters; using the built-in four-point vehicle footprint.");
        }
        for (std::size_t i = 0; i < footprint_x.size(); ++i) {
            footprint_.push_back({footprint_x[i], footprint_y[i]});
        }

        sub_ = nh_.subscribe(input_topic_, 1, &PointCloudCropNode::pointCloudCallback, this);
        pub_ = nh_.advertise<sensor_msgs::PointCloud2>(output_topic_, 1);

        ROS_INFO("PointCloudCropNode: %s -> %s", input_topic_.c_str(), output_topic_.c_str());
        ROS_INFO("ROI X[%.2f, %.2f], Y[%.2f, %.2f], Z[%.2f, %.2f]", min_x_, max_x_, min_y_, max_y_, min_z_, max_z_);
        ROS_INFO("Removing %zu-point vehicle footprint in the input cloud frame", footprint_.size());
    }

private:
    struct Point2D {
        double x;
        double y;
    };

    bool isInsideFootprint(double x, double y) const {
        bool inside = false;
        for (std::size_t i = 0, j = footprint_.size() - 1; i < footprint_.size(); j = i++) {
            const Point2D& a = footprint_[i];
            const Point2D& b = footprint_[j];
            const bool crosses = ((a.y > y) != (b.y > y));
            if (crosses) {
                const double intersection_x =
                    (b.x - a.x) * (y - a.y) / (b.y - a.y) + a.x;
                if (x < intersection_x) {
                    inside = !inside;
                }
            }
        }
        return inside;
    }

    void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
        pcl::PointCloud<pcl::PointXYZI> cloud;
        pcl::fromROSMsg(*msg, cloud);

        pcl::PointCloud<pcl::PointXYZI> filtered;
        filtered.header = cloud.header;
        filtered.is_dense = false;
        filtered.points.reserve(cloud.points.size());

        for (const auto& point : cloud.points) {
            if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
                continue;
            }
            if (point.x < min_x_ || point.x > max_x_ ||
                point.y < min_y_ || point.y > max_y_ ||
                point.z < min_z_ || point.z > max_z_) {
                continue;
            }
            const bool in_footprint_height =
                point.z >= footprint_min_z_ && point.z <= footprint_max_z_;
            if (remove_footprint_ && in_footprint_height && isInsideFootprint(point.x, point.y)) {
                continue;
            }
            filtered.points.push_back(point);
        }

        filtered.width = static_cast<std::uint32_t>(filtered.points.size());
        filtered.height = 1;

        sensor_msgs::PointCloud2 output_msg;
        pcl::toROSMsg(filtered, output_msg);
        output_msg.header = msg->header;
        pub_.publish(output_msg);
    }

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::Subscriber sub_;
    ros::Publisher pub_;

    std::string input_topic_;
    std::string output_topic_;
    double min_x_;
    double max_x_;
    double min_y_;
    double max_y_;
    double min_z_;
    double max_z_;
    double footprint_min_z_;
    double footprint_max_z_;
    bool remove_footprint_;
    std::vector<Point2D> footprint_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "pointcloud_crop_node");
    PointCloudCropNode node;
    ros::spin();
    return 0;
}
