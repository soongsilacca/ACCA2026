#ifndef ACCA_BT__DB_HANDLER_HPP_
#define ACCA_BT__DB_HANDLER_HPP_

#include <sqlite3.h>
#include <string>
#include <vector>
#include <iostream>
#include <cmath>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"

namespace acca_bt
{

struct PathPoint {
    double x;
    double y;
    double yaw;
    double speed;
};

class DBHandler {
public:
    DBHandler(const std::string& db_path) : db_(nullptr) {
        if (sqlite3_open(db_path.c_str(), &db_) != SQLITE_OK) {
            std::cerr << "Can't open database: " << sqlite3_errmsg(db_) << std::endl;
        } else {
            std::cout << "Opened database successfully: " << db_path << std::endl;
        }
    }

    ~DBHandler() {
        if (db_) {
            sqlite3_close(db_);
        }
    }

    std::vector<PathPoint> query_from_id(const std::string& path_id) {
        std::vector<PathPoint> points;
        std::string query = "SELECT x, y, yaw, speed FROM Path WHERE path_id = ?;";
        sqlite3_stmt* stmt;

        if (sqlite3_prepare_v2(db_, query.c_str(), -1, &stmt, nullptr) != SQLITE_OK) {
            std::cerr << "Failed to prepare statement: " << sqlite3_errmsg(db_) << std::endl;
            return points;
        }

        sqlite3_bind_text(stmt, 1, path_id.c_str(), -1, SQLITE_STATIC);

        while (sqlite3_step(stmt) == SQLITE_ROW) {
            PathPoint p;
            p.x = sqlite3_column_double(stmt, 0);
            p.y = sqlite3_column_double(stmt, 1);
            p.yaw = sqlite3_column_double(stmt, 2);
            p.speed = sqlite3_column_double(stmt, 3);
            points.push_back(p);
        }

        sqlite3_finalize(stmt);
        return points;
    }

    nav_msgs::msg::Path convert_to_msg(const std::vector<PathPoint>& points, rclcpp::Time now) {
        nav_msgs::msg::Path path_msg;
        path_msg.header.stamp = now;
        path_msg.header.frame_id = "map";

        for (const auto& p : points) {
            geometry_msgs::msg::PoseStamped pose;
            pose.header = path_msg.header;
            pose.pose.position.x = p.x;
            pose.pose.position.y = p.y;
            pose.pose.position.z = 0.0;

            tf2::Quaternion q;
            q.setRPY(0, 0, p.yaw);
            pose.pose.orientation.x = q.x();
            pose.pose.orientation.y = q.y();
            pose.pose.orientation.z = q.z();
            pose.pose.orientation.w = q.w();

            path_msg.poses.push_back(pose);
        }
        return path_msg;
    }

private:
    sqlite3* db_;
};

} // namespace acca_bt

#endif // ACCA_BT__DB_HANDLER_HPP_
