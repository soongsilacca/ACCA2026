#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

class GpsFixSyncNode : public rclcpp::Node
{
public:
  GpsFixSyncNode()
  : Node("gps_fix_sync"),
    recalibrate_on_first_(declare_parameter<bool>("recalibrate_on_first", true)),
    recalibrate_on_every_msg_(declare_parameter<bool>("recalibrate_on_every_msg", false)),
    warn_threshold_ns_(declare_parameter<int64_t>("warn_threshold_ns", 50'000'000)) // 50 ms
  {
    auto qos = rclcpp::SensorDataQoS();

    sub_fix_ = create_subscription<sensor_msgs::msg::NavSatFix>(
      "/ublox_gps_node/fix", qos,
      std::bind(&GpsFixSyncNode::fixCb, this, std::placeholders::_1));

    pub_fix_sync_ = create_publisher<sensor_msgs::msg::NavSatFix>("/ublox_gps_node/fix_syc", qos);

    RCLCPP_INFO(get_logger(), "gps_fix_sync started. Publishing to /ublox_gps_node/fix_syc");
  }

private:
  void fixCb(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
  {
    const rclcpp::Time gps_time(msg->header.stamp);
    const rclcpp::Time now = this->get_clock()->now();

    // 최초(or 매 메시지) 오프셋 계산: offset = now - gps_time
    if (!have_offset_ || recalibrate_on_every_msg_) {
      rclcpp::Duration new_offset = now - gps_time;

      if (have_offset_) {
        // 재보정 시 급격한 변화 경고
        auto diff_ns = std::llabs((new_offset - offset_).nanoseconds());
        if (diff_ns > warn_threshold_ns_) {
          RCLCPP_WARN(get_logger(),
            "Time offset jump detected: old=%.3f ms -> new=%.3f ms (Δ=%.3f ms)",
            offset_.nanoseconds() / 1e6,
            new_offset.nanoseconds() / 1e6,
            (double)diff_ns / 1e6);
        }
      }

      offset_ = new_offset;
      have_offset_ = true;

      if (recalibrate_on_first_) {
        RCLCPP_INFO(get_logger(), "Time offset set: %.3f ms (now - gps)",
                    offset_.nanoseconds() / 1e6);
      }
    }

    // 메시지 복사 및 재타임스탬프
    sensor_msgs::msg::NavSatFix out = *msg;
    if (have_offset_) {
      const rclcpp::Time corrected = gps_time + offset_;
      out.header.stamp = corrected;
    } else {
      // theoretically unreachable (첫 콜백에서 계산함)
      out.header.stamp = now;
    }

    pub_fix_sync_->publish(out);
  }

  // subs & pubs
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr sub_fix_;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr    pub_fix_sync_;

  // time sync
  rclcpp::Duration offset_{0, 0};
  bool have_offset_{false};

  // params
  bool recalibrate_on_first_;
  bool recalibrate_on_every_msg_;
  int64_t warn_threshold_ns_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GpsFixSyncNode>());
  rclcpp::shutdown();
  return 0;
}
