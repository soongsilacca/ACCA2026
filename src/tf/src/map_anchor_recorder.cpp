// map_anchor_recorder.cpp
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <fstream>
#include <vector>
#include <numeric>

class MapAnchorRecorder : public rclcpp::Node
{
public:
  MapAnchorRecorder() : Node("map_anchor_recorder")
  {
    /* ---------- 파라미터 ---------- */
    cov_threshold_ = declare_parameter("cov_threshold", 0.000196);   // 14 mm²
    sample_need_   = declare_parameter("sample_need",   100);
    file_path_     = declare_parameter("output_file",   "anchor.txt");

    gps_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
      "ublox_gps_node/fix", 10,
      std::bind(&MapAnchorRecorder::gpsCallback, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(),
      "Waiting for %zu samples (σ² ≤ %.8f) …", sample_need_, cov_threshold_);
  }

private:
  /* ---- GPS 콜백 ---- */
  void gpsCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
  {
    /* 정확도 필터 */
    if (msg->position_covariance[0] <= cov_threshold_ ||
        msg->position_covariance[4] <= cov_threshold_)
      return;

    lat_buf_.push_back(msg->latitude);
    lon_buf_.push_back(msg->longitude);

    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
      "sample %zu / %zu", lat_buf_.size(), sample_need_);

    /* 목표 개수 도달 → 평균 계산 & 파일 저장 */
    if (lat_buf_.size() == sample_need_)
    {
      double lat_mean = std::accumulate(lat_buf_.begin(), lat_buf_.end(), 0.0) / lat_buf_.size();
      double lon_mean = std::accumulate(lon_buf_.begin(), lon_buf_.end(), 0.0) / lon_buf_.size();

      std::ofstream ofs(file_path_, std::ios::out | std::ios::trunc);
      if (!ofs) {
        RCLCPP_ERROR(get_logger(), "Cannot open file: %s", file_path_.c_str());
        rclcpp::shutdown();
        return;
      }
      ofs << std::setprecision(10) << lat_mean << ' ' << lon_mean << '\n';
      ofs.close();

      RCLCPP_INFO(get_logger(),
        "Anchor saved to %s  (lat: %.10f, lon: %.10f)", file_path_.c_str(),
        lat_mean, lon_mean);

      rclcpp::shutdown();        // 작업 완료 후 종료
    }
  }

  /* ---- 멤버 ---- */
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
  std::vector<double> lat_buf_, lon_buf_;

  double cov_threshold_;
  std::size_t sample_need_;
  std::string file_path_;
};

/* ---------- main ---------- */
int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapAnchorRecorder>());
  rclcpp::shutdown();
  return 0;
}
