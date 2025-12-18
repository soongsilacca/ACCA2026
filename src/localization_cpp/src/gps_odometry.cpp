#include <memory>
#include <cmath>
#include <vector>
#include <numeric>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <proj.h>

class MapOdomTFPublisherStatic : public rclcpp::Node
{
public:
  MapOdomTFPublisherStatic()
  : Node("map_odom_tf_static_node"),
    P_(proj_create_crs_to_crs(PJ_DEFAULT_CTX, "EPSG:4326", "EPSG:2097", nullptr))
  {
    if (!P_)
      throw std::runtime_error("Failed to create projection");

    /*-------------- 파라미터 ----------------*/
    cov_threshold_ = 0.000196;     // 14 mm 1-σ^2
    samples_needed_ = 100;         // 평균에 쓸 Fix 개수

    /*-------------- 미리 map 앵커를 투영 ----------------*/
    PJ_COORD origin = proj_coord(map_lat_, map_lon_, 0, 0);
    PJ_COORD res    = proj_trans(P_, PJ_FWD, origin);
    map_x_ = res.xy.x;
    map_y_ = res.xy.y;

    /*-------------- ROS 통신 ----------------*/
    
    gps_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
      "ublox_gps_node/fix", 10,
      std::bind(&MapOdomTFPublisherStatic::gpsCallback, this, std::placeholders::_1));

    gps_odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("odometry/gps", 10);
  }

  ~MapOdomTFPublisherStatic() override
  {
    if (P_) proj_destroy(P_);
  }


private:
  /*---------------------- GPS ----------------------*/
  void gpsCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
  {
    
    lat = msg-> latitude;
    lon = msg-> longitude;
    PJ_COORD ll  = proj_coord(lat, lon, 0, 0);
    PJ_COORD xy  = proj_trans(P_, PJ_FWD, ll);
    gps_x_ = xy.xy.x;
    gps_y_ = xy.xy.y;
    odom_msg_ = nav_msgs::msg::Odometry();
    odom_msg_.header.stamp = this->get_clock()->now();
    odom_msg_.header.frame_id = "odom";
    odom_msg_.pose.pose.position.x = gps_x_;
    odom_msg_.pose.pose.position.y = gps_y_;
    odom_msg_.pose.pose.position.z = 0.0;
    odom_msg_.pose.pose.orientation.x = 0.0;
    odom_msg_.pose.pose.orientation.y = 0.0;
    odom_msg_.pose.pose.orientation.z = 0.0;
    odom_msg_.pose.pose.orientation.w = 1.0;

    gps_odom_pub_->publish(odom_msg_);
  
  }

  /* ------------ members ------------ */
  /* ROS */
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;

  /* PROJ */
  PJ* P_;

};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MapOdomTFPublisherStatic>();
  rclcpp::Rate r(10);

  while (rclcpp::ok()) {
    rclcpp::spin_some(node);
    node->publishTF();
    r.sleep();
  }
  rclcpp::shutdown();
  return 0;
}
