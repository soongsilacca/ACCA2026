#ifndef ACCA_BT__ACCA_OSM_NODES_HPP_
#define ACCA_BT__ACCA_OSM_NODES_HPP_

#include "behaviortree_cpp/behavior_tree.h"
#include "rclcpp/rclcpp.hpp"
#include "erp42_msgs/msg/control_message.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/polygon.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include <chrono>
#include <cmath>
#include <vector>
#include <limits>

namespace acca_bt
{

// Base class for OSM Nodes containing common utilities if needed
class OsmNodeBase
{
protected:
    double calculate_distance(const geometry_msgs::msg::Point& p1, const geometry_msgs::msg::Point& p2)
    {
        return std::sqrt(std::pow(p1.x - p2.x, 2) + std::pow(p1.y - p2.y, 2));
    }
};

// ... Action Node Base ...
// Helper to handle erp42 control message publishing
class ControlPublisher
{
public:
  ControlPublisher(rclcpp::Node::SharedPtr node)
  {
    publisher_ = node->create_publisher<erp42_msgs::msg::ControlMessage>("/cmd_msg", 10);
  }

protected:
  rclcpp::Publisher<erp42_msgs::msg::ControlMessage>::SharedPtr publisher_;
  static uint8_t alive_cnt_; // Make static

  void publish_control(int speed, int steer, int brake, int gear=2, int estop=0)
  {
    auto msg = erp42_msgs::msg::ControlMessage();
    msg.mora = 1; // Auto
    msg.estop = estop;
    msg.gear = gear;
    msg.speed = speed * 10; 
    msg.steer = steer;
    msg.brake = brake;
    msg.alive = alive_cnt_++;
    publisher_->publish(msg);
  }
};

// Initialize static member
uint8_t ControlPublisher::alive_cnt_ = 0;

// ... Action Node Base ...
class OsmActionNode : public BT::SyncActionNode, public ControlPublisher
{
public:
  OsmActionNode(const std::string & name, const BT::NodeConfig & config, rclcpp::Node::SharedPtr node)
  : BT::SyncActionNode(name, config), ControlPublisher(node), node_(node)
  {}

  static BT::PortsList providedPorts() { return {}; }

protected:
  rclcpp::Node::SharedPtr node_;
};

class StopVehicle : public OsmActionNode
{
public:
  using OsmActionNode::OsmActionNode;
  
  BT::NodeStatus tick() override
  {
    RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000, "ACTION: StopVehicle");
    publish_control(0, 0, 200); 
    return BT::NodeStatus::SUCCESS;
  }
};

class PrintLog : public BT::SyncActionNode
{
public:
  PrintLog(const std::string & name, const BT::NodeConfig & config, rclcpp::Node::SharedPtr node)
  : BT::SyncActionNode(name, config), node_(node)
  {}

  static BT::PortsList providedPorts()
  {
    return { BT::InputPort<std::string>("message") };
  }

  BT::NodeStatus tick() override
  {
    std::string message;
    if (!getInput("message", message)) message = "No message";
    RCLCPP_INFO(node_->get_logger(), "LOG: %s", message.c_str());
    return BT::NodeStatus::SUCCESS;
  }
protected:
  rclcpp::Node::SharedPtr node_;
};

class IsObstacleDetected : public BT::ConditionNode
{
public:
  IsObstacleDetected(const std::string & name, const BT::NodeConfig & config, rclcpp::Node::SharedPtr node)
  : BT::ConditionNode(name, config), node_(node)
  {
      subscription_ = node_->create_subscription<geometry_msgs::msg::PoseArray>(
      "/cone_pose_map", 10, std::bind(&IsObstacleDetected::topic_callback, this, std::placeholders::_1));
  }

  static BT::PortsList providedPorts()
  {
      return { BT::InputPort<double>("dist_threshold") };
  }

  void topic_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
  {
      last_msg_ = msg;
  }

  BT::NodeStatus tick() override
  {
      bool currently_detected = false;
      if (last_msg_ && !last_msg_->poses.empty()) {
          currently_detected = true;
      }

      auto now = node_->now();

      // If detected now, extend the stop window
      if (currently_detected) {
          stop_until_ = now + rclcpp::Duration::from_seconds(2.0); // Keep stopped for 2s after loss
          RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000, "OBSTACLE DETECTED! (LiDAR)");
          return BT::NodeStatus::SUCCESS;
      }

      // If not currently detected, but within latch window (hysteresis)
      if (now < stop_until_) {
           RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000, "OBSTACLE: Hysteresis Hold...");
           return BT::NodeStatus::SUCCESS;
      }

      return BT::NodeStatus::FAILURE; 
  }
private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr subscription_;
  geometry_msgs::msg::PoseArray::SharedPtr last_msg_;
  rclcpp::Time stop_until_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
};

class IsStopLineDetected : public BT::ConditionNode, public OsmNodeBase
{
public:
  IsStopLineDetected(const std::string & name, const BT::NodeConfig & config, rclcpp::Node::SharedPtr node)
  : BT::ConditionNode(name, config), node_(node)
  {
      sub_stop_lines_ = node_->create_subscription<geometry_msgs::msg::PoseArray>(
      "/stop_lines", 10, [this](const geometry_msgs::msg::PoseArray::SharedPtr msg){ stop_lines_ = msg; });
      
      sub_odom_ = node_->create_subscription<nav_msgs::msg::Odometry>(
      "/odometry/global", 10, [this](const nav_msgs::msg::Odometry::SharedPtr msg){ current_pose_ = msg; });
  }

  static BT::PortsList providedPorts()
  {
      return { BT::InputPort<double>("distance_threshold") };
  }

  BT::NodeStatus tick() override
  {
      if (!stop_lines_ || !current_pose_) return BT::NodeStatus::FAILURE;

      double threshold = 2.0;
      getInput("distance_threshold", threshold);

      double min_dist = std::numeric_limits<double>::max();
      for (const auto& pose : stop_lines_->poses) {
          double dist = calculate_distance(current_pose_->pose.pose.position, pose.position);
          if (dist < min_dist) min_dist = dist;
          if (dist < threshold) {
              RCLCPP_INFO(node_->get_logger(), "Stop Line REACHED! Dist: %.2f (Threshold: %.2f)", dist, threshold);
              return BT::NodeStatus::SUCCESS;
          }
      }
      
      if (min_dist != std::numeric_limits<double>::max()) {
          RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000, "Distance to nearest stop line: %.2f", min_dist);
      }
      
      return BT::NodeStatus::FAILURE; 
  }
private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr sub_stop_lines_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  geometry_msgs::msg::PoseArray::SharedPtr stop_lines_;
  nav_msgs::msg::Odometry::SharedPtr current_pose_;
};

// IsNearParkingZone class definition removed from header
// It is now defined in src/acca_bt_main.cpp to force correct compilation.

class CheckParkingAvailability : public BT::ConditionNode
{
public:
  CheckParkingAvailability(const std::string & name, const BT::NodeConfig & config, rclcpp::Node::SharedPtr node)
  : BT::ConditionNode(name, config), node_(node)
  {
      // Subscribe to all parking spots
      sub_parking_A_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/parking/A", 10, [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg){ parking_spot_A_ = msg; });
      sub_parking_B_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/parking/B", 10, [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg){ parking_spot_B_ = msg; });
      sub_parking_C_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/parking/C", 10, [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg){ parking_spot_C_ = msg; });

      sub_obstacles_ = node_->create_subscription<geometry_msgs::msg::PoseArray>(
      "/cone_pose_map", 10, [this](const geometry_msgs::msg::PoseArray::SharedPtr msg){ obstacles_ = msg; });
  }

  static BT::PortsList providedPorts()
  {
      return { BT::InputPort<std::string>("parking_id") };
  }

  BT::NodeStatus tick() override
  {
      std::string id;
      getInput("parking_id", id);
      
      geometry_msgs::msg::PoseStamped::SharedPtr target_spot;
      if (id == "A") target_spot = parking_spot_A_;
      else if (id == "B") target_spot = parking_spot_B_;
      else if (id == "C") target_spot = parking_spot_C_;
      else return BT::NodeStatus::FAILURE;
      
      if (!target_spot) return BT::NodeStatus::FAILURE; // No info yet

      // Check for obstacles near target_spot
      if (obstacles_) {
          for (const auto& obs : obstacles_->poses) {
             double dist = std::sqrt(std::pow(obs.position.x - target_spot->pose.position.x, 2) + 
                                     std::pow(obs.position.y - target_spot->pose.position.y, 2));
             if (dist < 2.5) { // If obstacle within 2.5m
                 RCLCPP_INFO(node_->get_logger(), "Parking %s is OCCUPIED", id.c_str());
                 return BT::NodeStatus::FAILURE; 
             }
          }
      }
      
      RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000, "Parking %s is AVAILABLE", id.c_str());
      return BT::NodeStatus::SUCCESS;
  }
private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_parking_A_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_parking_B_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_parking_C_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr sub_obstacles_;
  
  geometry_msgs::msg::PoseStamped::SharedPtr parking_spot_A_;
  geometry_msgs::msg::PoseStamped::SharedPtr parking_spot_B_;
  geometry_msgs::msg::PoseStamped::SharedPtr parking_spot_C_;
  geometry_msgs::msg::PoseArray::SharedPtr obstacles_;
};

class DriveForwardDuration : public BT::StatefulActionNode, public ControlPublisher
{
public:
  DriveForwardDuration(const std::string & name, const BT::NodeConfig & config, rclcpp::Node::SharedPtr node)
  : BT::StatefulActionNode(name, config), ControlPublisher(node), node_(node)
  {}

  static BT::PortsList providedPorts() {
    return { BT::InputPort<double>("seconds"), BT::InputPort<double>("speed") };
  }

  BT::NodeStatus onStart() override
  {
    double duration = 1.0;
    double speed = 0.5;
    getInput("seconds", duration);
    getInput("speed", speed);
    
    start_time_ = node_->now();
    duration_ = duration;
    speed_ = speed;
    
    RCLCPP_INFO(node_->get_logger(), "DriveForwardDuration: %.1fs, %.1f", duration, speed);
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning() override
  {
    if ((node_->now() - start_time_).seconds() >= duration_) {
        publish_control(0, 0, 200);
        return BT::NodeStatus::SUCCESS;
    }
    
    publish_control(static_cast<int>(speed_ * 3.6), 0, 0); // speed_ is m/s, convert to KPH
    return BT::NodeStatus::RUNNING;
  }

  void onHalted() override {
      publish_control(0, 0, 200);
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Time start_time_;
  double duration_;
  double speed_;
};


class ExecuteAutoParking : public OsmActionNode
{
public:
  using OsmActionNode::OsmActionNode;

  static BT::PortsList providedPorts() {
    return { BT::InputPort<std::string>("parking_id") };
  }
  
  BT::NodeStatus tick() override
  {
    std::string id = "Unknown";
    getInput("parking_id", id);
    RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000, "ACTION: ExecuteAutoParking in Spot %s (Simulated)", id.c_str());
    return BT::NodeStatus::SUCCESS;
  }
};

class FollowPath : public BT::StatefulActionNode
{
public:
  FollowPath(const std::string & name, const BT::NodeConfig & config, rclcpp::Node::SharedPtr node)
  : BT::StatefulActionNode(name, config), node_(node)
  {
      pub_enable_ = node_->create_publisher<std_msgs::msg::Bool>("/pure_pursuit/enable", 10);
      sub_path_ = node_->create_subscription<nav_msgs::msg::Path>(
          "/global_path", 10, [this](const nav_msgs::msg::Path::SharedPtr msg){ path_ = msg; });
      sub_odom_ = node_->create_subscription<nav_msgs::msg::Odometry>(
          "/odometry/global", 10, [this](const nav_msgs::msg::Odometry::SharedPtr msg){ current_pose_ = msg; });
  }

  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus onStart() override {
      std_msgs::msg::Bool msg;
      msg.data = true;
      pub_enable_->publish(msg);
      RCLCPP_INFO(node_->get_logger(), "Sending Enable to PathFollower/PurePursuit");
      return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning() override
  {
      if (!path_ || path_->poses.empty() || !current_pose_) {
          RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000, "FollowPath: Waiting for path/pose data...");
          return BT::NodeStatus::RUNNING;
      }

      const auto& goal_pos = path_->poses.back().pose.position;
      const auto& curr_pos = current_pose_->pose.pose.position;
      double dist = std::sqrt(std::pow(goal_pos.x - curr_pos.x, 2) + std::pow(goal_pos.y - curr_pos.y, 2));

      RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000, "Distance to goal: %.2f m", dist);

      if (dist < 2.0) { // Increased threshold for reliability
          RCLCPP_WARN(node_->get_logger(), "🎯 GOAL REACHED! Distance: %.2f m", dist);
          return BT::NodeStatus::SUCCESS;
      }
      return BT::NodeStatus::RUNNING;
  }

  void onHalted() override {
      std_msgs::msg::Bool msg;
      msg.data = false;
      pub_enable_->publish(msg);
      RCLCPP_INFO(node_->get_logger(), "Sending Disable to PathFollower/PurePursuit");
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_enable_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr sub_path_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  nav_msgs::msg::Path::SharedPtr path_;
  nav_msgs::msg::Odometry::SharedPtr current_pose_;
};

class WaitDuration : public BT::StatefulActionNode
{
public:
  WaitDuration(const std::string & name, const BT::NodeConfig & config)
  : BT::StatefulActionNode(name, config)
  {}

  static BT::PortsList providedPorts() { return { BT::InputPort<int>("msec") }; }

  BT::NodeStatus onStart() override
  {
      int msec = 1000;
      getInput("msec", msec);
      start_time_ = std::chrono::system_clock::now();
      duration_ = std::chrono::milliseconds(msec);
      return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning() override
  {
      if (std::chrono::system_clock::now() - start_time_ >= duration_) return BT::NodeStatus::SUCCESS;
      return BT::NodeStatus::RUNNING;
  }

  void onHalted() override {}
private:
  std::chrono::system_clock::time_point start_time_;
  std::chrono::milliseconds duration_;
};


class StartPythonParking : public BT::StatefulActionNode
{
public:
  StartPythonParking(const std::string & name, const BT::NodeConfig & config, rclcpp::Node::SharedPtr node)
  : BT::StatefulActionNode(name, config), node_(node)
  {
      // 1. 파이썬 노드에 "주차 시작해!"라고 말할 입구
      pub_cmd_ = node_->create_publisher<std_msgs::msg::String>("/parking_command", 10);
      
      // 2. 파이썬 노드가 "다 했어!"라고 말하면 들을 귀
      sub_status_ = node_->create_subscription<std_msgs::msg::String>(
          "/parking_status", 10, [this](const std_msgs::msg::String::SharedPtr msg) {
              if (msg->data == "SUCCESS") {
                  parking_finished_ = true;
              }
          });
  }

  // XML에서 받을 데이터 (A, B, C 구역 이름)
  static BT::PortsList providedPorts() {
    return { BT::InputPort<std::string>("parking_id") };
  }

  // 노드가 처음 딱 켜질 때 실행
  BT::NodeStatus onStart() override
  {
    std::string id;
    if (!getInput("parking_id", id)) id = "unknown";
    
    parking_finished_ = false;

    // 파이썬 노드에게 주차 구역 ID 전송
    auto msg = std_msgs::msg::String();
    msg.data = id;
    pub_cmd_->publish(msg);
    
    RCLCPP_INFO(node_->get_logger(), "--- [BT -> Python] Start Parking at Spot: %s ---", id.c_str());
    return BT::NodeStatus::RUNNING;
  }

  // 주차가 끝날 때까지 무한 반복 확인
  BT::NodeStatus onRunning() override
  {
    if (parking_finished_) {
        RCLCPP_INFO(node_->get_logger(), "--- [BT] Python Parking SUCCESS! ---");
        return BT::NodeStatus::SUCCESS;
    }
    // 아직 파이썬에서 SUCCESS 안 왔으면 계속 기다림
    return BT::NodeStatus::RUNNING;
  }

  void onHalted() override {
      parking_finished_ = false;
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_cmd_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_status_;
  bool parking_finished_ = false;
};

} // namespace acca_bt

#endif // ACCA_BT__ACCA_OSM_NODES_HPP_