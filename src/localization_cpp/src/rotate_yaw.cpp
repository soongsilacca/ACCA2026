#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <nav_msgs/msg/odometry.hpp>  // 추가


#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <deque>
#include <cmath>

class Rotate : public rclcpp::Node
{
public:
  Rotate()
  : Node("rotate_yaw"),
    delta_yaw_(this->declare_parameter<double>("delta_yaw", 0.0))
  {
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
      // History Depth 10 (추정)
    qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    qos.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

    sub_imu_ = create_subscription<sensor_msgs::msg::Imu>(
      "imu/data", qos,
      std::bind(&Rotate::imuCallback, this, std::placeholders::_1));

    sub_gps_vel_ = create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
      "ublox_gps_node/fix_velocity", qos,
      std::bind(&Rotate::gpsVelCallback, this, std::placeholders::_1));

    sub_gps_fix_ = create_subscription<sensor_msgs::msg::NavSatFix>(
      "ublox_gps_node/fix", qos,
      std::bind(&Rotate::gpsFixCallback, this, std::placeholders::_1));
    // ★ 추가: GPS 재밍 오도메트리 구독
    sub_gps_jam_ = create_subscription<nav_msgs::msg::Odometry>(
      "odometry/gps_jamming", qos,
      std::bind(&Rotate::gpsJammingCallback, this, std::placeholders::_1));


    pub_imu_rotated_ = create_publisher<sensor_msgs::msg::Imu>("imu/rotated", qos);
    pub_mean_quat_  = create_publisher<geometry_msgs::msg::Quaternion>("mean", qos);
  }

private:
  /* ---------- 보조 함수 ---------- */
  static void eulerFromQuat(const tf2::Quaternion &q,
                            double &roll, double &pitch, double &yaw)
  {
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
  }

  static tf2::Quaternion quatFromEuler(double roll, double pitch, double yaw)
  {
    tf2::Quaternion q;
    q.setRPY(roll, pitch, yaw);
    return q;
  }

  bool decisionStraight()
  {
    if (forward_.size() < 10) return false;

    double sum = 0.0;
    for (double y : forward_) sum += y;
    double mean = sum / forward_.size();

    for (double y : forward_)
      if (std::abs((mean - y) * 180.0 / M_PI) > 2.0)  // 2 deg 허용
      // if (std::abs((mean - y) * 180.0 / M_PI) > 1.0)  // 2 deg 허용
        return false;

    mean_ = mean;
    return true;
  }

  /* ---------- 콜백 ---------- */
  void gpsFixCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
  {
    cov_ = msg->position_covariance[0];
  }
  void gpsJammingCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    // cov[0] 사용 가정: pose.covariance[0]
    jamming_cov0_ = msg->pose.covariance[0];
  }

  void gpsVelCallback(const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg)
  {
    double vx = msg->twist.twist.linear.x;
    double vy = msg->twist.twist.linear.y;
    v_ = std::hypot(vx, vy);               // 속도 크기
    gps_yaw_ = std::atan2(vy, vx);         // GPS 방향

    forward_.push_back(gps_yaw_);
    if (forward_.size() > 10) forward_.pop_front();
  }

  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    /* ① IMU → (roll,pitch,yaw) */
    double roll, pitch, yaw;
    tf2::Quaternion q_orig(msg->orientation.x,
                           msg->orientation.y,
                           msg->orientation.z,
                           msg->orientation.w);
    eulerFromQuat(q_orig, roll, pitch, yaw);

    /* ② GPS‑IMU 차이 */
    double delta   = gps_yaw_ - yaw;
    double abs_deg = std::abs(delta) * 180.0 / M_PI;

    if (abs_deg > 2.0 && abs_deg < 90.0            // 2° < |Δ| < 90°
        && v_ > 0.50                               // 최소 속도
        && cov_ < 0.0004
        && jamming_cov0_ < 0.0004
      )                          // GPS 분산
    {
      if (decisionStraight()) {
        delta_ = mean_ - yaw;

        // GPS 평균 방향을 별도 topic으로 publish
        tf2::Quaternion q_gps = quatFromEuler(0,0,gps_yaw_);
        geometry_msgs::msg::Quaternion q_msg;
        q_msg.x = q_gps.x(); q_msg.y = q_gps.y();
        q_msg.z = q_gps.z(); q_msg.w = q_gps.w();
        pub_mean_quat_->publish(q_msg);
      }
    }

    /* ③ 보정된 yaw 적용 */
    double yaw_prev = yaw;
    yaw = yaw + delta_yaw_ + delta_;

    tf2::Quaternion q_new = quatFromEuler(0,0,yaw);

    sensor_msgs::msg::Imu imu_out = *msg;   // 원본 복사
    imu_out.orientation.x = q_new.x();
    imu_out.orientation.y = q_new.y();
    imu_out.orientation.z = q_new.z();
    imu_out.orientation.w = q_new.w();
    imu_out.header.stamp = this->get_clock()->now();


    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 100,
        "Yaw prev: %.2f°, new: %.2f°, delta: %.2f°",
        yaw_prev * 180.0 / M_PI,
        yaw      * 180.0 / M_PI,
        delta_   * 180.0 / M_PI);

    pub_imu_rotated_->publish(imu_out);
  }

  /* ---------- 멤버 변수 ---------- */
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr   sub_imu_;
  rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr sub_gps_vel_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr sub_gps_fix_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_gps_jam_; // 추가


  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu_rotated_;
  rclcpp::Publisher<geometry_msgs::msg::Quaternion>::SharedPtr pub_mean_quat_;

  /* 상태 저장 */
  std::deque<double> forward_;   // 최근 10 개의 GPS yaw
  double mean_{0.0};
  double delta_{0.0};

  double delta_yaw_;  // 파라미터
  double gps_yaw_{0.0};
  double v_{0.0};
  double cov_{0.0};
  double jamming_cov0_{0.0};
};

/* ---------- main ---------- */
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Rotate>());
  rclcpp::shutdown();
  return 0;
}