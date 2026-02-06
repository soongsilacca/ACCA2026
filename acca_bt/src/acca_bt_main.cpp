#include "acca_bt/acca_osm_nodes.hpp"
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/loggers/groot2_publisher.h"
#include "rclcpp/rclcpp.hpp"
#include <unistd.h>

// Direct implementation of IsNearParkingZone to force compilation update
class IsNearParkingZone : public BT::ConditionNode, public acca_bt::OsmNodeBase
{
public:
  IsNearParkingZone(const std::string & name, const BT::NodeConfig & config, rclcpp::Node::SharedPtr node)
  : BT::ConditionNode(name, config), node_(node)
  {
      // Subscribing to Polygon Boundary instead of PoseArray centroids
      sub_parking_boundary_ = node_->create_subscription<geometry_msgs::msg::Polygon>(
      "/parking/boundary", rclcpp::QoS(1).transient_local(), 
      [this](const geometry_msgs::msg::Polygon::SharedPtr msg){ parking_boundary_ = msg; });
      
      sub_odom_ = node_->create_subscription<nav_msgs::msg::Odometry>(
      "/odometry/global", 10, [this](const nav_msgs::msg::Odometry::SharedPtr msg){ current_pose_ = msg; });
  }

  static BT::PortsList providedPorts()
  {
      return { BT::InputPort<double>("distance_threshold") };
  }

  // Ray Casting Algorithm to check if point (x, y) is inside polygon
  bool isInside(const geometry_msgs::msg::Polygon& polygon, double x, double y)
  {
      int n = polygon.points.size();
      if (n < 3) return false;
      
      bool inside = false;
      for (int i = 0, j = n - 1; i < n; j = i++) {
          if (((polygon.points[i].y > y) != (polygon.points[j].y > y)) &&
              (x < (polygon.points[j].x - polygon.points[i].x) * (y - polygon.points[i].y) / 
                   (polygon.points[j].y - polygon.points[i].y) + polygon.points[i].x)) {
              inside = !inside;
          }
      }
      return inside;
  }

  BT::NodeStatus tick() override
  {
      // std::cout for absolute debugging
      if (!parking_boundary_) {
          std::cout << "[DEBUG] Waiting for Polygon..." << std::endl;
          return BT::NodeStatus::FAILURE;
      }
      if (!current_pose_) {
          std::cout << "[DEBUG] Waiting for Pose..." << std::endl;
          return BT::NodeStatus::FAILURE;
      }

      double x = current_pose_->pose.pose.position.x;
      double y = current_pose_->pose.pose.position.y;
      
      bool inside = isInside(*parking_boundary_, x, y);

      std::cout << "[DEBUG] Check: Pos(" << x << ", " << y << ") Inside=" << inside 
                << " PolyPts=" << parking_boundary_->points.size() << std::endl;

      if (inside) {
           std::cout << "✅✅✅ VEHICLE IS INSIDE!!! ✅✅✅" << std::endl;
           return BT::NodeStatus::SUCCESS;
      }
      return BT::NodeStatus::FAILURE;
  }
private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<geometry_msgs::msg::Polygon>::SharedPtr sub_parking_boundary_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  geometry_msgs::msg::Polygon::SharedPtr parking_boundary_;
  nav_msgs::msg::Odometry::SharedPtr current_pose_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("acca_bt_node");

  BT::BehaviorTreeFactory factory;

  // Register Nodes
  factory.registerBuilder<acca_bt::IsObstacleDetected>("IsObstacleDetected", 
    [node](const std::string & name, const BT::NodeConfig & config)
    {
      return std::make_unique<acca_bt::IsObstacleDetected>(name, config, node);
    });

  factory.registerBuilder<acca_bt::StopVehicle>("StopVehicle", 
    [node](const std::string & name, const BT::NodeConfig & config)
    {
      return std::make_unique<acca_bt::StopVehicle>(name, config, node);
    });

  factory.registerBuilder<acca_bt::IsStopLineDetected>("IsStopLineDetected", 
    [node](const std::string & name, const BT::NodeConfig & config)
    {
      return std::make_unique<acca_bt::IsStopLineDetected>(name, config, node);
    });

  factory.registerBuilder<IsNearParkingZone>("IsNearParkingZone", 
    [node](const std::string & name, const BT::NodeConfig & config)
    {
      return std::make_unique<IsNearParkingZone>(name, config, node);
    });

  factory.registerBuilder<acca_bt::DriveForwardDuration>("DriveForwardDuration", 
    [node](const std::string & name, const BT::NodeConfig & config)
    {
      return std::make_unique<acca_bt::DriveForwardDuration>(name, config, node);
    });
    
  factory.registerBuilder<acca_bt::CheckParkingAvailability>("CheckParkingAvailability", 
    [node](const std::string & name, const BT::NodeConfig & config)
    {
      return std::make_unique<acca_bt::CheckParkingAvailability>(name, config, node);
    });

  factory.registerBuilder<acca_bt::ExecuteAutoParking>("ExecuteAutoParking", 
    [node](const std::string & name, const BT::NodeConfig & config)
    {
      return std::make_unique<acca_bt::ExecuteAutoParking>(name, config, node);
    });

  factory.registerBuilder<acca_bt::FollowPath>("FollowPath", 
    [node](const std::string & name, const BT::NodeConfig & config)
    {
      return std::make_unique<acca_bt::FollowPath>(name, config, node);
    });

  factory.registerBuilder<acca_bt::PrintLog>("PrintLog",
    [node](const std::string & name, const BT::NodeConfig & config)
    {
      return std::make_unique<acca_bt::PrintLog>(name, config, node);
    });

  factory.registerBuilder<acca_bt::StartPythonParking>("StartPythonParking", 
  [node](const std::string & name, const BT::NodeConfig & config) {
    return std::make_unique<acca_bt::StartPythonParking>(name, config, node);
  });
    
  // WaitDuration is StatefulActionNode
  factory.registerNodeType<acca_bt::WaitDuration>("WaitDuration");


  // Load the Behavior Tree from the XML file
  node->declare_parameter("bt_xml_path", "");
  std::string param_path = node->get_parameter("bt_xml_path").as_string();

  std::string xml_path = "install/acca_bt/share/acca_bt/bt_xml/acca_bt.xml";
  
  if (!param_path.empty()) {
      xml_path = param_path;
  } else {
      // Check local path first for development
      if (access("bt_xml/acca_bt.xml", F_OK) == 0) {
          xml_path = "bt_xml/acca_bt.xml";
      } else if (access("/home/won/BT/ACCA2026/src/acca_bt/bt_xml/acca_bt.xml", F_OK) == 0) { // Check src location too
          xml_path = "/home/won/BT/ACCA2026/src/acca_bt/bt_xml/acca_bt.xml";
      }
  }

  RCLCPP_INFO(node->get_logger(), "Loading Behavior Tree from: %s", xml_path.c_str());

  BT::Tree tree;
  try {
     tree = factory.createTreeFromFile(xml_path);
  } catch (const std::exception& e) {
     RCLCPP_ERROR(node->get_logger(), "Failed to load XML from: %s. Error: %s", xml_path.c_str(), e.what());
     return 1;
  }

  // Connect the Groot2 publisher
  std::unique_ptr<BT::Groot2Publisher> publisher;
  try {
      publisher = std::make_unique<BT::Groot2Publisher>(tree);
      RCLCPP_INFO(node->get_logger(), "Groot2 Publisher started on port 1667");
  } catch (const std::exception& e) {
      RCLCPP_WARN(node->get_logger(), "Failed to start Groot2 Publisher (Port 1667 busy). Autonomous driving will proceed without visualizer.");
  }

  RCLCPP_INFO(node->get_logger(), "Starting ACCA_BT Behavior Tree (OSM based)...");

  rclcpp::Rate rate(50); // Higher rate for responsiveness
  int tick_count = 0;
  while (rclcpp::ok()) {
    BT::NodeStatus status = tree.tickOnce();
    
    if (tick_count++ % 100 == 0) {
        RCLCPP_INFO(node->get_logger(), "BT Heartbeat: Ticking... Status: %s", BT::toStr(status).c_str());
    }

    rclcpp::spin_some(node);
    
    if (status == BT::NodeStatus::SUCCESS) {
        // In this tree, Fallback typically returns RUNNING or FAILURE, but if all done:
        RCLCPP_INFO(node->get_logger(), "Tree finished successfully.");
        // break; // Don't break, maybe restart or just stay
    }
    
    rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}
