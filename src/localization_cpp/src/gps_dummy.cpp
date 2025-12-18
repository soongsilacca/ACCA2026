// gps_jamming_filter.cpp
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/string.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>

class GpsJammingFilter : public rclcpp::Node
{
public:
  GpsJammingFilter()
  : Node("gps_jamming_filter")
  {
    // ─────────────── Parameter ───────────────
    this->declare_parameter<bool>("gps_jamming_mode", false);
    jamming_active_ = this->get_parameter("gps_jamming_mode").as_bool();

    // ─────────────── QoS ───────────────
    auto qos = rclcpp::SystemDefaultsQoS().keep_last(10);

    // ─────────────── Pub/Sub ───────────────
    sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odometry/gps", qos,
      std::bind(&GpsJammingFilter::odomCallback, this, std::placeholders::_1));

    pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
      "/odometry/gps_jamming", qos);

    mode_sub_ = this->create_subscription<std_msgs::msg::String>(
      "jamming_status", qos,
      std::bind(&GpsJammingFilter::mode_Callback, this, std::placeholders::_1));
  }

private:
  // ------- Mode(subscription) callback -------
  void mode_Callback(const std_msgs::msg::String::SharedPtr msg)
  {
    if (!msg) return;

    if (msg->data == "True") {
      jamming_active_ = true;
    } else if (msg->data == "False") {
      jamming_active_ = false;
    }
    RCLCPP_INFO(this->get_logger(), "jamming_status: %s -> %s",
                msg->data.c_str(), jamming_active_ ? "ON" : "OFF");
  }

  // ------- Odometry subscription callback -------
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    if (!msg) return;

    nav_msgs::msg::Odometry out_msg = *msg;   // copy

    if (jamming_active_)
    {
      // pose.covariance는 6×6 행렬이 1-D 배열(36)로 직렬화되어 있음
      for (size_t i = 0; i < 36; i += 7)      // 0,7,14,21,28,35: 대각 원소
      {
        out_msg.pose.covariance[i] = 9.99999999999e11;
      }
    }
    // 그대로 또는 수정된 메시지를 새 토픽으로 발행
    out_msg.header.stamp = this->get_clock()->now();
    pub_->publish(out_msg);
  }

  // ------- Members -------
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mode_sub_;

  bool jamming_active_{false};
};

// ────────────────────────────────────────────
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GpsJammingFilter>());
  rclcpp::shutdown();
  return 0;
}
