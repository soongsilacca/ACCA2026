#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <geometry_msgs/Point.h>
#include <morai_msgs/EgoVehicleStatus.h>
#include <nav_msgs/Odometry.h>
#include <pcl/PointIndices.h>
#include <pcl/common/common.h>
#include <pcl/common/point_tests.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/String.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <rear_lidar_perception/TrackedObject.h>
#include <rear_lidar_perception/TrackedObjectArray.h>

namespace {

using PointT = pcl::PointXYZI;
using Cloud = pcl::PointCloud<PointT>;

constexpr double kPi = 3.14159265358979323846;

double clamp(double value, double low, double high) {
  return std::max(low, std::min(value, high));
}

double yawFromQuaternion(const geometry_msgs::Quaternion& q) {
  const double siny = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny, cosy);
}

geometry_msgs::Quaternion yawQuaternion(double yaw) {
  geometry_msgs::Quaternion q;
  q.z = std::sin(0.5 * yaw);
  q.w = std::cos(0.5 * yaw);
  return q;
}

Eigen::Vector2d rotateToWorld(const Eigen::Vector2d& p, double yaw) {
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  return Eigen::Vector2d(c * p.x() - s * p.y(), s * p.x() + c * p.y());
}

Eigen::Vector2d rotateToBody(const Eigen::Vector2d& p, double yaw) {
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  return Eigen::Vector2d(c * p.x() + s * p.y(), -s * p.x() + c * p.y());
}

Eigen::Vector2d integrateBodyVelocity(double vx, double vy, double yaw,
                                      double yaw_rate, double duration) {
  if (std::abs(yaw_rate) < 1e-4) {
    return rotateToWorld(Eigen::Vector2d(vx, vy), yaw) * duration;
  }
  const double future_yaw = yaw + yaw_rate * duration;
  const double dx = (vx * (std::sin(future_yaw) - std::sin(yaw)) +
                     vy * (std::cos(future_yaw) - std::cos(yaw))) /
                    yaw_rate;
  const double dy = (-vx * (std::cos(future_yaw) - std::cos(yaw)) +
                     vy * (std::sin(future_yaw) - std::sin(yaw))) /
                    yaw_rate;
  return Eigen::Vector2d(dx, dy);
}

struct Detection {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Vector2d base_position{0.0, 0.0};
  Eigen::Vector2d tracking_position{0.0, 0.0};
  Eigen::Vector3d dimensions{0.0, 0.0, 0.0};
  double z{0.0};
};

struct Track {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  std::uint32_t id{0};
  Eigen::Vector2d position{0.0, 0.0};
  Eigen::Vector2d velocity{0.0, 0.0};
  Eigen::Vector3d dimensions{0.0, 0.0, 0.0};
  double z{0.0};
  ros::Time created;
  ros::Time state_stamp;
  ros::Time last_seen;
  int hits{0};
  int missed{0};
  int dynamic_score{0};
};

using DetectionVector =
    std::vector<Detection, Eigen::aligned_allocator<Detection>>;
using TrackVector = std::vector<Track, Eigen::aligned_allocator<Track>>;

struct EgoState {
  bool valid{false};
  int source{0};  // 1=primary odom, 2=fallback odom, 3=MORAI ego status
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
  double vx_body{0.0};
  double vy_body{0.0};
  double yaw_rate{0.0};
};

struct OdomSample {
  nav_msgs::Odometry message;
  ros::WallTime receipt_wall_time;
  bool received{false};
};

}  // namespace

class RearLidarPerceptionNode {
 public:
  RearLidarPerceptionNode() : nh_(), pnh_("~") {
    loadParameters();

    const Eigen::Matrix3f rotation =
        (Eigen::AngleAxisf(sensor_yaw_, Eigen::Vector3f::UnitZ()) *
         Eigen::AngleAxisf(sensor_pitch_, Eigen::Vector3f::UnitY()) *
         Eigen::AngleAxisf(sensor_roll_, Eigen::Vector3f::UnitX()))
            .toRotationMatrix();
    sensor_to_base_.setIdentity();
    sensor_to_base_.linear() = rotation;
    sensor_to_base_.translation() = Eigen::Vector3f(sensor_x_, sensor_y_, sensor_z_);

    cloud_sub_ = nh_.subscribe(input_topic_, 1,
                               &RearLidarPerceptionNode::cloudCallback, this);
    odom_sub_ = nh_.subscribe(odom_topic_, 20,
                              &RearLidarPerceptionNode::odomCallback, this);
    if (!fallback_odom_topic_.empty() && fallback_odom_topic_ != odom_topic_) {
      fallback_odom_sub_ = nh_.subscribe(
          fallback_odom_topic_, 20,
          &RearLidarPerceptionNode::fallbackOdomCallback, this);
    }
    if (!ego_status_topic_.empty()) {
      ego_status_sub_ = nh_.subscribe(
          ego_status_topic_, 20,
          &RearLidarPerceptionNode::egoStatusCallback, this);
    }

    obstacle_cloud_pub_ =
        nh_.advertise<sensor_msgs::PointCloud2>("/rear_lidar/obstacles", 1);
    cropped_cloud_pub_ =
        nh_.advertise<sensor_msgs::PointCloud2>("/rear_lidar/debug/cropped", 1);
    ground_cloud_pub_ =
        nh_.advertise<sensor_msgs::PointCloud2>("/rear_lidar/debug/ground", 1);
    wall_cloud_pub_ =
        nh_.advertise<sensor_msgs::PointCloud2>("/rear_lidar/debug/walls", 1);
    tracked_pub_ = nh_.advertise<rear_lidar_perception::TrackedObjectArray>(
        "/rear_lidar/tracked_objects", 1);
    marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(
        "/rear_lidar/markers", 1);
    status_pub_ = nh_.advertise<std_msgs::String>("/rear_lidar/status", 1, true);
    startup_wall_time_ = ros::WallTime::now();
    watchdog_timer_ = nh_.createWallTimer(
        ros::WallDuration(1.0), &RearLidarPerceptionNode::watchdogCallback, this);

    ROS_INFO("Rear LiDAR perception: %s -> /rear_lidar/obstacles", input_topic_.c_str());
    ROS_INFO("Mount base_link->rear_lidar xyz=(%.2f, %.2f, %.2f), rpy=(%.3f, %.3f, %.3f)",
             sensor_x_, sensor_y_, sensor_z_, sensor_roll_, sensor_pitch_, sensor_yaw_);
    ROS_INFO("Tracking motion sources: primary=%s fallback=%s ego_status=%s",
             odom_topic_.c_str(),
             fallback_odom_topic_.empty() ? "disabled" : fallback_odom_topic_.c_str(),
             ego_status_topic_.empty() ? "disabled" : ego_status_topic_.c_str());
  }

 private:
  void loadParameters() {
    pnh_.param<std::string>("input_topic", input_topic_, "/rear_lidar/points_raw");
    pnh_.param<std::string>("odom_topic", odom_topic_, "/rear_lidar/ego_odometry");
    pnh_.param<std::string>("fallback_odom_topic", fallback_odom_topic_,
                            "/localization/kinematic_state");
    pnh_.param<std::string>("ego_status_topic", ego_status_topic_,
                            "/morai/ego_vehicle_status");
    pnh_.param<std::string>("output_frame", output_frame_, "base_link");

    pnh_.param("sensor_x", sensor_x_, -0.73f);
    pnh_.param("sensor_y", sensor_y_, 0.0f);
    pnh_.param("sensor_z", sensor_z_, 0.22f);
    pnh_.param("sensor_roll", sensor_roll_, 0.0f);
    pnh_.param("sensor_pitch", sensor_pitch_, 0.0f);
    pnh_.param("sensor_yaw", sensor_yaw_, 0.0f);

    pnh_.param("roi_min_x", roi_min_x_, -80.0f);
    pnh_.param("roi_max_x", roi_max_x_, 5.0f);
    pnh_.param("roi_min_y", roi_min_y_, -12.0f);
    pnh_.param("roi_max_y", roi_max_y_, 12.0f);
    pnh_.param("roi_min_z", roi_min_z_, -1.0f);
    pnh_.param("roi_max_z", roi_max_z_, 4.0f);
    pnh_.param("vehicle_min_x", vehicle_min_x_, -1.05f);
    pnh_.param("vehicle_max_x", vehicle_max_x_, 4.10f);
    pnh_.param("vehicle_min_y", vehicle_min_y_, -1.15f);
    pnh_.param("vehicle_max_y", vehicle_max_y_, 1.15f);

    pnh_.param("ground_distance_threshold", ground_distance_threshold_, 0.16f);
    pnh_.param("ground_max_tilt_deg", ground_max_tilt_deg_, 12.0f);
    pnh_.param("expected_ground_z", expected_ground_z_, -0.46f);
    pnh_.param("ground_height_tolerance", ground_height_tolerance_, 0.40f);
    pnh_.param("ground_max_iterations", ground_max_iterations_, 120);

    pnh_.param("wall_distance_threshold", wall_distance_threshold_, 0.12f);
    pnh_.param("wall_max_normal_z", wall_max_normal_z_, 0.30f);
    pnh_.param("wall_min_inliers", wall_min_inliers_, 100);
    pnh_.param("wall_min_length", wall_min_length_, 10.0f);
    pnh_.param("wall_min_height", wall_min_height_, 0.70f);
    pnh_.param("wall_bin_size", wall_bin_size_, 0.50f);
    pnh_.param("wall_min_occupancy_ratio", wall_min_occupancy_ratio_, 0.60f);
    pnh_.param("wall_max_planes", wall_max_planes_, 3);

    pnh_.param("voxel_leaf_size", voxel_leaf_size_, 0.20f);
    pnh_.param("cluster_tolerance", cluster_tolerance_, 0.70f);
    pnh_.param("cluster_range_scale", cluster_range_scale_, 0.010f);
    pnh_.param("max_cluster_tolerance", max_cluster_tolerance_, 1.50f);
    pnh_.param("min_cluster_points", min_cluster_points_, 5);
    pnh_.param("max_cluster_points", max_cluster_points_, 12000);
    pnh_.param("min_object_height", min_object_height_, 0.20f);
    pnh_.param("max_object_height", max_object_height_, 4.0f);
    pnh_.param("min_object_planar_size", min_object_planar_size_, 0.20f);
    pnh_.param("max_object_length", max_object_length_, 15.0f);
    pnh_.param("max_object_width", max_object_width_, 6.0f);
    pnh_.param("sparse_detection_range", sparse_detection_range_, 30.0f);
    pnh_.param("sparse_assumed_height", sparse_assumed_height_, 0.80f);

    pnh_.param("association_distance", association_distance_, 2.5);
    pnh_.param("track_alpha", track_alpha_, 0.65);
    pnh_.param("track_beta", track_beta_, 0.20);
    pnh_.param("min_confirmed_hits", min_confirmed_hits_, 2);
    pnh_.param("max_missed_frames", max_missed_frames_, 5);
    pnh_.param("dynamic_speed_threshold", dynamic_speed_threshold_, 0.8);
    pnh_.param("prediction_horizon", prediction_horizon_, 4.0);
    pnh_.param("prediction_dt", prediction_dt_, 0.5);
    pnh_.param("max_odom_age", max_odom_age_, 0.5);
    pnh_.param("enable_ground_filter", enable_ground_filter_, true);
    pnh_.param("enable_wall_filter", enable_wall_filter_, true);
    pnh_.param("publish_debug_clouds", publish_debug_clouds_, true);

    cluster_tolerance_ = std::max(cluster_tolerance_, 0.05f);
    max_cluster_tolerance_ = std::max(max_cluster_tolerance_, cluster_tolerance_);
    min_cluster_points_ = std::max(min_cluster_points_, 1);
    max_cluster_points_ = std::max(max_cluster_points_, min_cluster_points_);
    track_alpha_ = clamp(track_alpha_, 0.0, 1.0);
    track_beta_ = clamp(track_beta_, 0.0, 1.0);
    prediction_dt_ = std::max(prediction_dt_, 0.05);
    prediction_horizon_ = std::max(prediction_horizon_, prediction_dt_);
  }

  void odomCallback(const nav_msgs::OdometryConstPtr& msg) {
    storeOdom(*msg, primary_odom_);
  }

  void fallbackOdomCallback(const nav_msgs::OdometryConstPtr& msg) {
    storeOdom(*msg, fallback_odom_);
  }

  void egoStatusCallback(const morai_msgs::EgoVehicleStatusConstPtr& msg) {
    nav_msgs::Odometry odom;
    odom.header = msg->header;
    odom.header.frame_id = "map";
    odom.child_frame_id = "base_link";
    odom.pose.pose.position.x = msg->position.x;
    odom.pose.pose.position.y = msg->position.y;
    odom.pose.pose.position.z = msg->position.z;
    odom.pose.pose.orientation = yawQuaternion(msg->yaw * kPi / 180.0);

    // MORAI EgoVehicleStatus exposes velocity in km/h and angular velocity in
    // deg/s.  nav_msgs/Odometry uses m/s and rad/s in child_frame_id.
    odom.twist.twist.linear.x = msg->velocity.x / 3.6;
    odom.twist.twist.linear.y = msg->velocity.y / 3.6;
    odom.twist.twist.linear.z = msg->velocity.z / 3.6;
    odom.twist.twist.angular.x = msg->angular_velocity.x * kPi / 180.0;
    odom.twist.twist.angular.y = msg->angular_velocity.y * kPi / 180.0;
    odom.twist.twist.angular.z = msg->angular_velocity.z * kPi / 180.0;

    storeOdom(odom, ego_status_odom_);
  }

  static void storeOdom(const nav_msgs::Odometry& message, OdomSample& sample) {
    sample.message = message;
    sample.receipt_wall_time = ros::WallTime::now();
    sample.received = true;
  }

  EgoState egoAt(const ros::Time& stamp) const {
    EgoState ego;
    // The Velodyne packet stamp and the MORAI UDP bridge stamp can use
    // different clock domains.  Freshness therefore has to be decided from
    // local receipt time; otherwise valid ego motion is rejected and every
    // static object acquires approximately the opposite of the ego speed.
    const ros::WallTime now = ros::WallTime::now();
    const auto isFresh = [this, &now](const OdomSample& sample) {
      return sample.received &&
             (now - sample.receipt_wall_time).toSec() <= max_odom_age_;
    };
    const OdomSample* sample = nullptr;
    if (isFresh(primary_odom_)) {
      sample = &primary_odom_;
      ego.source = 1;
    } else if (isFresh(fallback_odom_)) {
      sample = &fallback_odom_;
      ego.source = 2;
    } else if (isFresh(ego_status_odom_)) {
      sample = &ego_status_odom_;
      ego.source = 3;
    }
    if (sample == nullptr) {
      return ego;
    }

    const nav_msgs::Odometry& odom = sample->message;
    ros::Time odom_stamp = odom.header.stamp;
    double dt = 0.0;
    if (!stamp.isZero() && !odom_stamp.isZero()) {
      const double stamp_dt = (stamp - odom_stamp).toSec();
      if (std::isfinite(stamp_dt) && std::abs(stamp_dt) <= max_odom_age_) {
        dt = stamp_dt;
      } else {
        ROS_WARN_THROTTLE(
            5.0,
            "Rear LiDAR and odometry stamps differ by %.3f s; using fresh latest pose",
            stamp_dt);
      }
    }

    ego.valid = true;
    ego.x = odom.pose.pose.position.x;
    ego.y = odom.pose.pose.position.y;
    ego.yaw = yawFromQuaternion(odom.pose.pose.orientation);
    ego.vx_body = odom.twist.twist.linear.x;
    ego.vy_body = odom.twist.twist.linear.y;
    ego.yaw_rate = odom.twist.twist.angular.z;

    const Eigen::Vector2d displacement = integrateBodyVelocity(
        ego.vx_body, ego.vy_body, ego.yaw, ego.yaw_rate, dt);
    ego.x += displacement.x();
    ego.y += displacement.y();
    ego.yaw += ego.yaw_rate * dt;
    return ego;
  }

  Cloud::Ptr transformAndCrop(const sensor_msgs::PointCloud2& msg) const {
    Cloud::Ptr input(new Cloud());
    Cloud::Ptr output(new Cloud());
    pcl::fromROSMsg(msg, *input);
    output->points.reserve(input->points.size());

    for (const PointT& raw : input->points) {
      if (!pcl::isFinite(raw)) {
        continue;
      }
      const Eigen::Vector3f p = sensor_to_base_ * Eigen::Vector3f(raw.x, raw.y, raw.z);
      if (p.x() < roi_min_x_ || p.x() > roi_max_x_ ||
          p.y() < roi_min_y_ || p.y() > roi_max_y_ ||
          p.z() < roi_min_z_ || p.z() > roi_max_z_) {
        continue;
      }
      const bool on_vehicle = p.x() >= vehicle_min_x_ && p.x() <= vehicle_max_x_ &&
                              p.y() >= vehicle_min_y_ && p.y() <= vehicle_max_y_;
      if (on_vehicle) {
        continue;
      }
      PointT point = raw;
      point.x = p.x();
      point.y = p.y();
      point.z = p.z();
      output->points.push_back(point);
    }
    output->width = static_cast<std::uint32_t>(output->points.size());
    output->height = 1;
    output->is_dense = false;
    return output;
  }

  Cloud::Ptr removeGround(const Cloud::Ptr& input, Cloud::Ptr& ground) const {
    Cloud::Ptr obstacles(new Cloud());
    ground.reset(new Cloud());
    if (input->size() < 20) {
      *obstacles = *input;
      return obstacles;
    }

    pcl::SACSegmentation<PointT> segmentation;
    pcl::PointIndices inliers;
    pcl::ModelCoefficients coefficients;
    segmentation.setOptimizeCoefficients(true);
    segmentation.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
    segmentation.setMethodType(pcl::SAC_RANSAC);
    segmentation.setAxis(Eigen::Vector3f::UnitZ());
    segmentation.setEpsAngle(ground_max_tilt_deg_ * kPi / 180.0);
    segmentation.setDistanceThreshold(ground_distance_threshold_);
    segmentation.setMaxIterations(ground_max_iterations_);
    segmentation.setInputCloud(input);
    segmentation.segment(inliers, coefficients);

    bool plane_valid = coefficients.values.size() >= 4;
    double a = 0.0, b = 0.0, c = 1.0, d = -expected_ground_z_;
    if (plane_valid) {
      a = coefficients.values[0];
      b = coefficients.values[1];
      c = coefficients.values[2];
      d = coefficients.values[3];
      const double norm = std::sqrt(a * a + b * b + c * c);
      if (norm < 1e-6 || std::abs(c) < 1e-3) {
        plane_valid = false;
      } else {
        a /= norm;
        b /= norm;
        c /= norm;
        d /= norm;
        if (c < 0.0) {
          a = -a;
          b = -b;
          c = -c;
          d = -d;
        }
        const double height_at_origin = -d / c;
        plane_valid = std::abs(height_at_origin - expected_ground_z_) <=
                      ground_height_tolerance_;
      }
    }

    obstacles->points.reserve(input->points.size());
    ground->points.reserve(input->points.size());
    for (const PointT& point : input->points) {
      const double height = plane_valid
                                ? (a * point.x + b * point.y + c * point.z + d)
                                : (point.z - expected_ground_z_);
      if (height <= ground_distance_threshold_) {
        ground->points.push_back(point);
      } else {
        obstacles->points.push_back(point);
      }
    }
    if (!plane_valid) {
      ROS_WARN_THROTTLE(2.0,
                        "Rear LiDAR ground plane unavailable; using expected_ground_z fallback");
    }
    finalizeCloud(*obstacles);
    finalizeCloud(*ground);
    return obstacles;
  }

  Cloud::Ptr removeWalls(const Cloud::Ptr& input, Cloud::Ptr& walls) const {
    Cloud::Ptr remaining(new Cloud(*input));
    Cloud::Ptr preserved(new Cloud());
    walls.reset(new Cloud());

    for (int iteration = 0; iteration < wall_max_planes_ &&
                            remaining->size() >= static_cast<std::size_t>(wall_min_inliers_);
         ++iteration) {
      pcl::SACSegmentation<PointT> segmentation;
      pcl::PointIndices inliers;
      pcl::ModelCoefficients coefficients;
      segmentation.setOptimizeCoefficients(true);
      segmentation.setModelType(pcl::SACMODEL_PLANE);
      segmentation.setMethodType(pcl::SAC_RANSAC);
      segmentation.setDistanceThreshold(wall_distance_threshold_);
      segmentation.setMaxIterations(100);
      segmentation.setInputCloud(remaining);
      segmentation.segment(inliers, coefficients);
      if (inliers.indices.size() < static_cast<std::size_t>(wall_min_inliers_) ||
          coefficients.values.size() < 4) {
        break;
      }

      Cloud::Ptr plane(new Cloud());
      Cloud::Ptr next(new Cloud());
      std::vector<bool> selected(remaining->size(), false);
      for (int index : inliers.indices) {
        if (index >= 0 && static_cast<std::size_t>(index) < selected.size()) {
          selected[static_cast<std::size_t>(index)] = true;
          plane->points.push_back(remaining->points[static_cast<std::size_t>(index)]);
        }
      }
      next->points.reserve(remaining->size() - plane->size());
      for (std::size_t i = 0; i < remaining->size(); ++i) {
        if (!selected[i]) {
          next->points.push_back(remaining->points[i]);
        }
      }
      finalizeCloud(*plane);
      finalizeCloud(*next);

      PointT min_point, max_point;
      pcl::getMinMax3D(*plane, min_point, max_point);
      const double nx = coefficients.values[0];
      const double ny = coefficients.values[1];
      const double nz = coefficients.values[2];
      const double normal_norm = std::sqrt(nx * nx + ny * ny + nz * nz);
      const double normalized_abs_nz = normal_norm > 1e-6 ? std::abs(nz) / normal_norm : 1.0;
      double planar_length = 0.0;
      double occupancy_ratio = 0.0;
      const double horizontal_normal_norm = std::hypot(nx, ny);
      if (horizontal_normal_norm > 1e-6) {
        const double tangent_x = -ny / horizontal_normal_norm;
        const double tangent_y = nx / horizontal_normal_norm;
        double min_projection = std::numeric_limits<double>::max();
        double max_projection = std::numeric_limits<double>::lowest();
        for (const PointT& point : plane->points) {
          const double projection = tangent_x * point.x + tangent_y * point.y;
          min_projection = std::min(min_projection, projection);
          max_projection = std::max(max_projection, projection);
        }
        planar_length = max_projection - min_projection;
        const int bin_count = std::max(
            1, static_cast<int>(std::ceil(planar_length / std::max(wall_bin_size_, 0.05f))));
        std::vector<bool> occupied(static_cast<std::size_t>(bin_count), false);
        for (const PointT& point : plane->points) {
          const double projection = tangent_x * point.x + tangent_y * point.y;
          const int bin = std::min(
              bin_count - 1,
              std::max(0, static_cast<int>((projection - min_projection) /
                                           std::max(wall_bin_size_, 0.05f))));
          occupied[static_cast<std::size_t>(bin)] = true;
        }
        const int occupied_count =
            static_cast<int>(std::count(occupied.begin(), occupied.end(), true));
        occupancy_ratio = static_cast<double>(occupied_count) / bin_count;
      }
      const double height = max_point.z - min_point.z;
      const bool is_wall = normalized_abs_nz <= wall_max_normal_z_ &&
                           planar_length >= wall_min_length_ &&
                           height >= wall_min_height_ &&
                           occupancy_ratio >= wall_min_occupancy_ratio_;
      if (is_wall) {
        *walls += *plane;
      } else {
        *preserved += *plane;
      }
      remaining = next;
    }

    *preserved += *remaining;
    finalizeCloud(*preserved);
    finalizeCloud(*walls);
    return preserved;
  }

  Cloud::Ptr downsample(const Cloud::Ptr& input) const {
    if (input->empty() || voxel_leaf_size_ <= 0.0f) {
      return Cloud::Ptr(new Cloud(*input));
    }
    pcl::VoxelGrid<PointT> voxel;
    Cloud::Ptr result(new Cloud());
    voxel.setInputCloud(input);
    voxel.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);
    voxel.filter(*result);
    return result;
  }

  DetectionVector detectClusters(const Cloud::Ptr& cloud,
                                 const EgoState& ego) const {
    DetectionVector detections;
    if (cloud->size() < static_cast<std::size_t>(min_cluster_points_)) {
      return detections;
    }

    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());
    tree->setInputCloud(cloud);
    // VLP-16 vertical beams are widely separated at highway distances.  A
    // fixed Euclidean radius splits a far vehicle into one cluster per ring,
    // so grow the radius with range while keeping it below a lane-width merge.
    std::vector<pcl::PointIndices> cluster_indices;
    std::vector<bool> processed(cloud->size(), false);
    for (std::size_t seed = 0; seed < cloud->size(); ++seed) {
      if (processed[seed]) {
        continue;
      }
      std::vector<int> queue(1, static_cast<int>(seed));
      processed[seed] = true;
      for (std::size_t cursor = 0; cursor < queue.size(); ++cursor) {
        const int index = queue[cursor];
        const PointT& point = cloud->points[static_cast<std::size_t>(index)];
        const double range = std::hypot(point.x, point.y);
        const double radius = std::min<double>(
            max_cluster_tolerance_, cluster_tolerance_ + cluster_range_scale_ * range);
        std::vector<int> neighbors;
        std::vector<float> squared_distances;
        tree->radiusSearch(index, radius, neighbors, squared_distances);
        for (int neighbor : neighbors) {
          if (neighbor >= 0 && static_cast<std::size_t>(neighbor) < processed.size() &&
              !processed[static_cast<std::size_t>(neighbor)]) {
            processed[static_cast<std::size_t>(neighbor)] = true;
            queue.push_back(neighbor);
          }
        }
      }
      if (queue.size() >= static_cast<std::size_t>(min_cluster_points_) &&
          queue.size() <= static_cast<std::size_t>(max_cluster_points_)) {
        pcl::PointIndices cluster;
        cluster.indices.swap(queue);
        cluster_indices.push_back(cluster);
      }
    }

    for (const pcl::PointIndices& indices : cluster_indices) {
      // The indexed getMinMax3D overload in PCL 1.10 returns Eigen vectors,
      // unlike the whole-cloud overload which accepts PointT outputs.
      Eigen::Vector4f min_point;
      Eigen::Vector4f max_point;
      pcl::getMinMax3D(*cloud, indices.indices, min_point, max_point);
      const double dx = max_point.x() - min_point.x();
      const double dy = max_point.y() - min_point.y();
      const double dz = max_point.z() - min_point.z();
      const double length = std::max(dx, dy);
      const double width = std::min(dx, dy);
      const double range = std::hypot(0.5 * (min_point.x() + max_point.x()),
                                      0.5 * (min_point.y() + max_point.y()));
      const bool sparse_far_object = range >= sparse_detection_range_;
      if ((!sparse_far_object && dz < min_object_height_) || dz > max_object_height_ ||
          length < min_object_planar_size_ ||
          (!sparse_far_object && width < min_object_planar_size_) ||
          length > max_object_length_ || width > max_object_width_) {
        continue;
      }

      Detection detection;
      detection.base_position = Eigen::Vector2d(
          0.5 * (min_point.x() + max_point.x()),
          0.5 * (min_point.y() + max_point.y()));
      detection.z = 0.5 * (min_point.z() + max_point.z());
      detection.dimensions = Eigen::Vector3d(
          sparse_far_object ? std::max(dx, 0.50) : dx,
          sparse_far_object ? std::max(dy, 0.50) : dy,
          sparse_far_object ? std::max<double>(dz, sparse_assumed_height_) : dz);
      detection.tracking_position = ego.valid
                                        ? Eigen::Vector2d(ego.x, ego.y) +
                                              rotateToWorld(detection.base_position, ego.yaw)
                                        : detection.base_position;
      detections.push_back(detection);
    }
    return detections;
  }

  void updateTracks(const DetectionVector& detections, const ros::Time& stamp,
                    int motion_source) {
    const bool odometry_compensated = motion_source != 0;
    if (tracking_mode_initialized_ && tracking_motion_source_ != motion_source) {
      tracks_.clear();
      ROS_WARN("Rear LiDAR motion source changed (%d -> %d); tracks were reset",
               tracking_motion_source_, motion_source);
    }
    tracking_mode_initialized_ = true;
    tracking_motion_source_ = motion_source;

    for (Track& track : tracks_) {
      double dt = (stamp - track.state_stamp).toSec();
      if (dt > 0.0 && dt < 2.0) {
        track.position += track.velocity * dt;
      }
      track.state_stamp = stamp;
    }

    struct Pair {
      double distance;
      std::size_t track_index;
      std::size_t detection_index;
    };
    std::vector<Pair> pairs;
    for (std::size_t ti = 0; ti < tracks_.size(); ++ti) {
      for (std::size_t di = 0; di < detections.size(); ++di) {
        const double distance =
            (tracks_[ti].position - detections[di].tracking_position).norm();
        const double gate = association_distance_ +
                            0.25 * tracks_[ti].velocity.norm();
        if (distance <= gate) {
          pairs.push_back(Pair{distance, ti, di});
        }
      }
    }
    std::sort(pairs.begin(), pairs.end(),
              [](const Pair& a, const Pair& b) { return a.distance < b.distance; });

    std::vector<bool> track_used(tracks_.size(), false);
    std::vector<bool> detection_used(detections.size(), false);
    for (const Pair& pair : pairs) {
      if (track_used[pair.track_index] || detection_used[pair.detection_index]) {
        continue;
      }
      Track& track = tracks_[pair.track_index];
      const Detection& detection = detections[pair.detection_index];
      const double observation_dt = clamp((stamp - track.last_seen).toSec(), 0.05, 1.0);
      const Eigen::Vector2d residual = detection.tracking_position - track.position;
      track.position += track_alpha_ * residual;
      track.velocity += (track_beta_ / observation_dt) * residual;
      track.dimensions = 0.7 * track.dimensions + 0.3 * detection.dimensions;
      track.z = 0.7 * track.z + 0.3 * detection.z;
      track.last_seen = stamp;
      ++track.hits;
      track.missed = 0;

      if (!odometry_compensated) {
        // Relative LiDAR motion alone cannot distinguish a moving object from
        // a stationary object passed by the ego vehicle.  Fail closed rather
        // than marking the entire scene dynamic.
        track.dynamic_score = 0;
      } else if (track.velocity.norm() >= dynamic_speed_threshold_) {
        track.dynamic_score = std::min(track.dynamic_score + 1, 5);
      } else {
        track.dynamic_score = std::max(track.dynamic_score - 1, 0);
      }
      track_used[pair.track_index] = true;
      detection_used[pair.detection_index] = true;
    }

    for (std::size_t i = 0; i < tracks_.size(); ++i) {
      if (!track_used[i]) {
        ++tracks_[i].missed;
      }
    }
    for (std::size_t i = 0; i < detections.size(); ++i) {
      if (detection_used[i]) {
        continue;
      }
      Track track;
      track.id = next_track_id_++;
      track.position = detections[i].tracking_position;
      track.dimensions = detections[i].dimensions;
      track.z = detections[i].z;
      track.created = stamp;
      track.state_stamp = stamp;
      track.last_seen = stamp;
      track.hits = 1;
      tracks_.push_back(track);
    }

    tracks_.erase(
        std::remove_if(tracks_.begin(), tracks_.end(), [this](const Track& track) {
          return track.missed > max_missed_frames_;
        }),
        tracks_.end());
  }

  Eigen::Vector2d trackPositionInBase(const Track& track, const EgoState& ego) const {
    if (!ego.valid) {
      return track.position;
    }
    return rotateToBody(track.position - Eigen::Vector2d(ego.x, ego.y), ego.yaw);
  }

  Eigen::Vector2d objectVelocityInBase(const Track& track, const EgoState& ego) const {
    // track.velocity is only a ground-referenced object velocity when ego
    // pose compensation is active.  Without it, expose the measurement via
    // relative_velocity but do not mislabel it as object velocity.
    return ego.valid ? rotateToBody(track.velocity, ego.yaw)
                     : Eigen::Vector2d::Zero();
  }

  Eigen::Vector2d relativeVelocityInBase(const Track& track, const EgoState& ego) const {
    if (!ego.valid) {
      return track.velocity;
    }
    const Eigen::Vector2d ego_velocity_world =
        rotateToWorld(Eigen::Vector2d(ego.vx_body, ego.vy_body), ego.yaw);
    return rotateToBody(track.velocity - ego_velocity_world, ego.yaw);
  }

  Eigen::Vector2d predictedRelativePosition(const Track& track, const EgoState& ego,
                                            double horizon) const {
    if (!ego.valid) {
      // Absolute dead reckoning is unobservable without ego motion.  Keep the
      // prediction anchored instead of drawing an ego-induced false path.
      return track.position;
    }
    const Eigen::Vector2d future_ego = Eigen::Vector2d(ego.x, ego.y) +
        integrateBodyVelocity(ego.vx_body, ego.vy_body, ego.yaw,
                              ego.yaw_rate, horizon);
    const double future_yaw = ego.yaw + ego.yaw_rate * horizon;
    const Eigen::Vector2d future_object = track.position + track.velocity * horizon;
    return rotateToBody(future_object - future_ego, future_yaw);
  }

  void publishTracks(const ros::Time& stamp, const EgoState& ego) {
    rear_lidar_perception::TrackedObjectArray output;
    output.header.stamp = stamp;
    output.header.frame_id = output_frame_;

    visualization_msgs::MarkerArray markers;
    visualization_msgs::Marker clear;
    clear.action = visualization_msgs::Marker::DELETEALL;
    markers.markers.push_back(clear);

    int marker_id = 0;
    for (const Track& track : tracks_) {
      if (track.hits < min_confirmed_hits_) {
        continue;
      }
      const Eigen::Vector2d position = trackPositionInBase(track, ego);
      const Eigen::Vector2d velocity = objectVelocityInBase(track, ego);
      const Eigen::Vector2d relative_velocity = relativeVelocityInBase(track, ego);
      const bool dynamic = ego.valid && track.dynamic_score >= 2;

      rear_lidar_perception::TrackedObject object;
      object.track_id = track.id;
      object.pose.position.x = position.x();
      object.pose.position.y = position.y();
      object.pose.position.z = track.z;
      const double heading = velocity.norm() > 0.2 ? std::atan2(velocity.y(), velocity.x()) : 0.0;
      object.pose.orientation = yawQuaternion(heading);
      object.dimensions.x = track.dimensions.x();
      object.dimensions.y = track.dimensions.y();
      object.dimensions.z = track.dimensions.z();
      object.velocity.x = velocity.x();
      object.velocity.y = velocity.y();
      object.relative_velocity.x = relative_velocity.x();
      object.relative_velocity.y = relative_velocity.y();
      object.is_dynamic = dynamic;
      object.odometry_compensated = ego.valid;
      object.age = std::max(0.0, (stamp - track.created).toSec());
      object.time_since_update = std::max(0.0, (stamp - track.last_seen).toSec());
      object.prediction_dt = prediction_dt_;

      if (dynamic) {
        for (double horizon = prediction_dt_;
             horizon <= prediction_horizon_ + 1e-6; horizon += prediction_dt_) {
          const Eigen::Vector2d predicted = predictedRelativePosition(track, ego, horizon);
          geometry_msgs::Point point;
          point.x = predicted.x();
          point.y = predicted.y();
          point.z = track.z;
          object.predicted_positions.push_back(point);
        }
      }
      output.objects.push_back(object);

      visualization_msgs::Marker box;
      box.header = output.header;
      box.ns = "rear_lidar_boxes";
      box.id = marker_id++;
      box.type = visualization_msgs::Marker::CUBE;
      box.action = visualization_msgs::Marker::ADD;
      box.pose = object.pose;
      box.scale = object.dimensions;
      box.color.r = dynamic ? 1.0f : 0.15f;
      box.color.g = dynamic ? 0.15f : 0.85f;
      box.color.b = 0.10f;
      box.color.a = 0.55f;
      box.lifetime = ros::Duration(0.25);
      markers.markers.push_back(box);

      if (dynamic) {
        visualization_msgs::Marker path;
        path.header = output.header;
        path.ns = "rear_lidar_predictions";
        path.id = marker_id++;
        path.type = visualization_msgs::Marker::LINE_STRIP;
        path.action = visualization_msgs::Marker::ADD;
        path.scale.x = 0.12;
        path.color.r = 1.0f;
        path.color.g = 0.55f;
        path.color.b = 0.0f;
        path.color.a = 1.0f;
        path.lifetime = ros::Duration(0.25);
        geometry_msgs::Point current;
        current.x = position.x();
        current.y = position.y();
        current.z = track.z;
        path.points.push_back(current);
        path.points.insert(path.points.end(), object.predicted_positions.begin(),
                           object.predicted_positions.end());
        markers.markers.push_back(path);
      }

      visualization_msgs::Marker text;
      text.header = output.header;
      text.ns = "rear_lidar_labels";
      text.id = marker_id++;
      text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
      text.action = visualization_msgs::Marker::ADD;
      text.pose.position = object.pose.position;
      text.pose.position.z += 0.5 * object.dimensions.z + 0.5;
      text.pose.orientation.w = 1.0;
      text.scale.z = 0.45;
      text.color.r = text.color.g = text.color.b = text.color.a = 1.0f;
      if (ego.valid) {
        text.text = std::string("ID ") + std::to_string(track.id) +
                    (dynamic ? " DYN " : " ") +
                    std::to_string(static_cast<int>(velocity.norm() * 3.6)) + " km/h";
      } else {
        text.text = std::string("ID ") + std::to_string(track.id) + " NO ODOM";
      }
      text.lifetime = ros::Duration(0.25);
      markers.markers.push_back(text);
    }
    tracked_pub_.publish(output);
    marker_pub_.publish(markers);
  }

  void publishCloud(const Cloud::Ptr& cloud, const ros::Time& stamp,
                    ros::Publisher& publisher) const {
    sensor_msgs::PointCloud2 message;
    pcl::toROSMsg(*cloud, message);
    message.header.stamp = stamp;
    message.header.frame_id = output_frame_;
    publisher.publish(message);
  }

  static void finalizeCloud(Cloud& cloud) {
    cloud.width = static_cast<std::uint32_t>(cloud.points.size());
    cloud.height = 1;
    cloud.is_dense = false;
  }

  void watchdogCallback(const ros::WallTimerEvent&) {
    const double since_start = (ros::WallTime::now() - startup_wall_time_).toSec();
    const double since_cloud = cloud_count_ == 0
                                   ? since_start
                                   : (ros::WallTime::now() - last_cloud_wall_time_).toSec();
    if (since_cloud <= 3.0) {
      return;
    }
    std_msgs::String status;
    status.data =
        "NO_POINTCLOUD: no /rear_lidar/points_raw callback for 3 s. Check MORAI "
        "sensor Connect, Destination Port 2368, UDP firewall, and velodyne poll timeout.";
    status_pub_.publish(status);
    ROS_ERROR_THROTTLE(5.0, "%s", status.data.c_str());
  }

  void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
    ++cloud_count_;
    last_cloud_wall_time_ = ros::WallTime::now();
    const ros::WallTime processing_start = ros::WallTime::now();

    // Publish before PCL processing.  Seeing this latched message proves the
    // perception subscriber is connected even when a later stage is slow.
    std_msgs::String processing_status;
    processing_status.data =
        std::string("PROCESSING raw=") + std::to_string(msg->width * msg->height);
    status_pub_.publish(processing_status);
    ROS_INFO_ONCE("Rear LiDAR perception received its first PointCloud2 frame");

    const ros::Time stamp = msg->header.stamp.isZero() ? ros::Time::now()
                                                       : msg->header.stamp;
    Cloud::Ptr cropped = transformAndCrop(*msg);
    Cloud::Ptr ground;
    Cloud::Ptr without_ground;
    if (enable_ground_filter_) {
      without_ground = removeGround(cropped, ground);
    } else {
      without_ground.reset(new Cloud(*cropped));
      ground.reset(new Cloud());
    }

    // Reduce the scan before wall RANSAC and radius searches.  Running these
    // stages on every dense raw point can block the single ROS callback thread.
    Cloud::Ptr reduced_without_ground = downsample(without_ground);
    Cloud::Ptr walls;
    Cloud::Ptr without_walls;
    if (enable_wall_filter_) {
      without_walls = removeWalls(reduced_without_ground, walls);
    } else {
      without_walls.reset(new Cloud(*reduced_without_ground));
      walls.reset(new Cloud());
    }
    Cloud::Ptr obstacles = without_walls;
    finalizeCloud(*obstacles);

    const EgoState ego = egoAt(stamp);
    if (!ego.valid) {
      ROS_WARN_THROTTLE(
          5.0,
          "Rear LiDAR odometry unavailable/stale: dynamic classification and prediction disabled");
    }
    const DetectionVector detections = detectClusters(obstacles, ego);
    updateTracks(detections, stamp, ego.source);

    publishCloud(obstacles, stamp, obstacle_cloud_pub_);
    if (publish_debug_clouds_) {
      publishCloud(cropped, stamp, cropped_cloud_pub_);
      publishCloud(ground, stamp, ground_cloud_pub_);
      publishCloud(walls, stamp, wall_cloud_pub_);
    }
    publishTracks(stamp, ego);

    std::ostringstream stream;
    stream << "OK raw=" << msg->width * msg->height << " cropped=" << cropped->size()
           << " obstacles=" << obstacles->size() << " detections=" << detections.size()
           << " tracks=" << tracks_.size()
           << " odometry=" << (ego.valid ? "compensated" : "relative")
           << " motion_source="
           << (ego.source == 1 ? "primary_odom"
                               : ego.source == 2 ? "fallback_odom"
                                                 : ego.source == 3 ? "ego_status" : "none")
           << " processing_ms="
           << static_cast<int>((ros::WallTime::now() - processing_start).toSec() * 1000.0);
    std_msgs::String status;
    status.data = stream.str();
    status_pub_.publish(status);

    ROS_INFO_THROTTLE(2.0,
                      "Rear LiDAR: input=%zu obstacle_points=%zu clusters=%zu tracks=%zu odom=%s",
                      cropped->size(), obstacles->size(), detections.size(), tracks_.size(),
                      ego.valid ? "compensated" : "relative");
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber cloud_sub_;
  ros::Subscriber odom_sub_;
  ros::Subscriber fallback_odom_sub_;
  ros::Subscriber ego_status_sub_;
  ros::Publisher obstacle_cloud_pub_;
  ros::Publisher cropped_cloud_pub_;
  ros::Publisher ground_cloud_pub_;
  ros::Publisher wall_cloud_pub_;
  ros::Publisher tracked_pub_;
  ros::Publisher marker_pub_;
  ros::Publisher status_pub_;
  ros::WallTimer watchdog_timer_;

  std::string input_topic_;
  std::string odom_topic_;
  std::string fallback_odom_topic_;
  std::string ego_status_topic_;
  std::string output_frame_;
  Eigen::Affine3f sensor_to_base_{Eigen::Affine3f::Identity()};

  float sensor_x_, sensor_y_, sensor_z_;
  float sensor_roll_, sensor_pitch_, sensor_yaw_;
  float roi_min_x_, roi_max_x_, roi_min_y_, roi_max_y_, roi_min_z_, roi_max_z_;
  float vehicle_min_x_, vehicle_max_x_, vehicle_min_y_, vehicle_max_y_;
  float ground_distance_threshold_, ground_max_tilt_deg_, expected_ground_z_;
  float ground_height_tolerance_;
  int ground_max_iterations_;
  float wall_distance_threshold_, wall_max_normal_z_, wall_min_length_, wall_min_height_;
  float wall_bin_size_, wall_min_occupancy_ratio_;
  int wall_min_inliers_, wall_max_planes_;
  float voxel_leaf_size_, cluster_tolerance_, cluster_range_scale_;
  float max_cluster_tolerance_;
  int min_cluster_points_, max_cluster_points_;
  float min_object_height_, max_object_height_, min_object_planar_size_;
  float max_object_length_, max_object_width_;
  float sparse_detection_range_, sparse_assumed_height_;

  double association_distance_, track_alpha_, track_beta_;
  int min_confirmed_hits_, max_missed_frames_;
  double dynamic_speed_threshold_, prediction_horizon_, prediction_dt_, max_odom_age_;
  bool publish_debug_clouds_;
  bool enable_ground_filter_, enable_wall_filter_;

  OdomSample primary_odom_;
  OdomSample fallback_odom_;
  OdomSample ego_status_odom_;
  bool tracking_mode_initialized_{false};
  int tracking_motion_source_{0};
  TrackVector tracks_;
  std::uint32_t next_track_id_{1};
  std::uint64_t cloud_count_{0};
  ros::WallTime startup_wall_time_;
  ros::WallTime last_cloud_wall_time_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "rear_lidar_perception");
  RearLidarPerceptionNode node;
  ros::spin();
  return 0;
}
