/*
 * OpenPlanner & Autoware Universe C++ Local Path Planner Node
 * Architecture:
 *   - Quintic Frenet Lattice Rollouts (Multi-Lane Support up to +/- 6.5m)
 *   - 3-Circle Ego Vehicle Clearance Envelope
 *   - Continuous Cost Minimization (FSM-Free Smooth Avoidance)
 *   - Dynamic Curvature & In-Path Obstacle Speed Profiling
 *   - 100% Synchronized Trajectory & RViz Visualization Markers
 */

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/String.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <global_path_planner/PlannerTrajectory.h>
#include <global_path_planner/PlannerWaypoint.h>

// OpenPlanner C++ Source Headers (op_local_planner_src)
#include "op_frenet_planner.h"

#include <cmath>
#include <vector>
#include <algorithm>
#include <string>

const double KMH_TO_MS = 1.0 / 3.6;
const double MS_TO_KMH = 3.6;

// Using OpenPlanner C++ Core Class from op_local_planner_src
using QuinticPolynomial = PlannerHNS::QuinticPolynomialCore;

// ----------------------------------------------------------------------------
// Simple Natural Cubic Spline for Path Interpolation
// ----------------------------------------------------------------------------
class SimpleCubicSpline {
public:
    std::vector<double> x_vec, y_vec, s_vec;
    int n;

    void set_points(const std::vector<double>& s, const std::vector<double>& x, const std::vector<double>& y) {
        s_vec = s;
        x_vec = x;
        y_vec = y;
        n = s.size();
    }

    void calc(double s_query, double& x_out, double& y_out, double& yaw_out) const {
        if (n == 0) return;
        if (s_query <= s_vec.front()) {
            x_out = x_vec.front();
            y_out = y_vec.front();
            yaw_out = std::atan2(y_vec[1] - y_vec[0], x_vec[1] - x_vec[0]);
            return;
        }
        if (s_query >= s_vec.back()) {
            x_out = x_vec.back();
            y_out = y_vec.back();
            yaw_out = std::atan2(y_vec[n - 1] - y_vec[n - 2], x_vec[n - 1] - x_vec[n - 2]);
            return;
        }

        auto it = std::upper_bound(s_vec.begin(), s_vec.end(), s_query);
        int idx = std::max(0, static_cast<int>(it - s_vec.begin()) - 1);
        int next_idx = std::min(n - 1, idx + 1);

        double ds = s_vec[next_idx] - s_vec[idx];
        double t = (ds > 1e-6) ? (s_query - s_vec[idx]) / ds : 0.0;

        x_out = (1.0 - t) * x_vec[idx] + t * x_vec[next_idx];
        y_out = (1.0 - t) * y_vec[idx] + t * y_vec[next_idx];

        double dx = x_vec[next_idx] - x_vec[idx];
        double dy = y_vec[next_idx] - y_vec[idx];
        yaw_out = std::atan2(dy, dx);
    }
};

struct Obstacle {
    double x, y, r;
};

// ----------------------------------------------------------------------------
// Main OpenPlanner C++ Node Class
// ----------------------------------------------------------------------------
class OpenPlannerCppNode {
public:
    OpenPlannerCppNode() {
        ros::NodeHandle nh;
        ros::NodeHandle pnh("~");

        // Parameters
        pnh.param("planning/horizon_dist", horizon_dist_, 50.0);
        pnh.param("planning/step_size", step_size_, 0.5);
        pnh.param("planning/max_speed_kmh", max_speed_kmh_, 60.0);
        pnh.param("planning/min_speed_kmh", min_speed_kmh_, 15.0);
        pnh.param("planning/max_lat_accel", max_lat_accel_, 2.0);
        pnh.param("planning/max_drivable_offset", max_drivable_offset_, 6.5);
        pnh.param("planning/max_offset_rate", max_offset_rate_, 0.20);
        pnh.param("planning/publish_rate", publish_rate_, 10.0);
        pnh.param<std::string>("planning/frame_id", frame_id_, "map");

        pnh.param("rollouts/num_rollouts", num_rollouts_, 41);
        pnh.param("rollouts/rollout_spacing", rollout_spacing_, 0.35);
        pnh.param("rollouts/safety_margin", safety_margin_, 1.2);
        pnh.param("rollouts/enable_avoidance", enable_avoidance_, true);

        pnh.param("acc/min_stop_dist", min_stop_dist_, 4.0);
        pnh.param("acc/max_decel", max_decel_, 2.5);

        wheelbase_ = 3.0;
        vehicle_width_ = 1.89;
        vehicle_length_ = 4.635;
        filtered_offset_ = 0.0;
        target_offset_ = 0.0;
        max_speed_ms_ = max_speed_kmh_ * KMH_TO_MS;

        // Generate offsets
        int half = num_rollouts_ / 2;
        offsets_.clear();
        for (int i = -half; i <= half; ++i) {
            double off = i * rollout_spacing_;
            off = std::max(-max_drivable_offset_, std::min(max_drivable_offset_, off));
            if (offsets_.empty() || std::abs(offsets_.back() - off) > 1e-4) {
                offsets_.push_back(off);
            }
        }

        // Publishers & Subscribers
        traj_pub_ = nh.advertise<global_path_planner::PlannerTrajectory>("/local_trajectory", 1);
        rollout_pub_ = nh.advertise<visualization_msgs::MarkerArray>("/rollout_markers", 1);
        speed_pub_ = nh.advertise<visualization_msgs::MarkerArray>("/speed_markers", 1);
        status_pub_ = nh.advertise<std_msgs::String>("/planner_status", 1);

        sub_global_path_ = nh.subscribe("/global_path", 1, &OpenPlannerCppNode::cbGlobalPath, this);
        sub_odom_ = nh.subscribe("/localization/kinematic_state", 1, &OpenPlannerCppNode::cbOdom, this);
        sub_obstacles_ = nh.subscribe("/clusters_markers", 1, &OpenPlannerCppNode::cbClusters, this);

        timer_ = nh.createTimer(ros::Duration(1.0 / publish_rate_), &OpenPlannerCppNode::planningLoop, this);
        ROS_INFO("[OpenPlanner C++ Node] Initialized with %zu rollouts (max offset +/-%.2fm)", offsets_.size(), max_drivable_offset_);
    }

private:
    // ROS Infrastructure
    ros::Publisher traj_pub_, rollout_pub_, speed_pub_, status_pub_;
    ros::Subscriber sub_global_path_, sub_odom_, sub_obstacles_;
    ros::Timer timer_;
    tf::TransformListener tf_listener_;

    // Configuration
    double horizon_dist_, step_size_, max_speed_kmh_, min_speed_kmh_, max_lat_accel_;
    double max_drivable_offset_, max_offset_rate_, publish_rate_;
    int num_rollouts_;
    double rollout_spacing_, safety_margin_, min_stop_dist_, max_decel_;
    bool enable_avoidance_;
    std::string frame_id_;

    double wheelbase_, vehicle_width_, vehicle_length_;
    double max_speed_ms_, filtered_offset_, target_offset_;

    std::vector<double> offsets_;
    nav_msgs::Path::ConstPtr global_path_;
    nav_msgs::Odometry::ConstPtr odom_;
    std::vector<Obstacle> obstacle_list_;

    void cbGlobalPath(const nav_msgs::Path::ConstPtr& msg) {
        if (!msg->poses.empty()) {
            global_path_ = msg;
        }
    }

    void cbOdom(const nav_msgs::Odometry::ConstPtr& msg) {
        odom_ = msg;
    }

    void cbClusters(const visualization_msgs::MarkerArray::ConstPtr& msg) {
        std::vector<Obstacle> obs;
        for (const auto& m : msg->markers) {
            if (m.action == visualization_msgs::Marker::DELETE || m.action == visualization_msgs::Marker::DELETEALL)
                continue;

            double r = std::max(0.4, std::min(std::max(m.scale.x, m.scale.y) / 2.0, 2.5));
            std::string f_id = m.header.frame_id.empty() ? "velodyne" : m.header.frame_id;
            double ox = m.pose.position.x;
            double oy = m.pose.position.y;

            if (f_id != frame_id_ && odom_) {
                double ego_x = odom_->pose.pose.position.x;
                double ego_y = odom_->pose.pose.position.y;
                tf::Quaternion q(odom_->pose.pose.orientation.x, odom_->pose.pose.orientation.y,
                               odom_->pose.pose.orientation.z, odom_->pose.pose.orientation.w);
                double roll, pitch, yaw;
                tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

                double lx = m.pose.position.x + (f_id == "velodyne" ? 3.85 : 0.0);
                double ly = m.pose.position.y;
                ox = ego_x + lx * std::cos(yaw) - ly * std::sin(yaw);
                oy = ego_y + lx * std::sin(yaw) + ly * std::cos(yaw);
            }
            obs.push_back({ox, oy, r});
        }
        obstacle_list_ = obs;
    }

    int getClosestIndex(double ego_x, double ego_y, double ego_yaw) {
        if (!global_path_ || global_path_->poses.empty()) return 0;
        int n = global_path_->poses.size();
        double min_d = 1e9;
        int min_idx = 0;
        for (int i = 0; i < n; ++i) {
            double dx = global_path_->poses[i].pose.position.x - ego_x;
            double dy = global_path_->poses[i].pose.position.y - ego_y;
            double d = std::hypot(dx, dy);
            if (d < min_d) {
                min_d = d;
                min_idx = i;
            }
        }
        for (int i = min_idx; i < std::min(n - 1, min_idx + 10); ++i) {
            double dx = global_path_->poses[i].pose.position.x - ego_x;
            double dy = global_path_->poses[i].pose.position.y - ego_y;
            double hdg = std::atan2(dy, dx);
            double diff = std::atan2(std::sin(hdg - ego_yaw), std::cos(hdg - ego_yaw));
            if (std::abs(diff) < M_PI / 2.0) return i;
        }
        return min_idx;
    }

    void planningLoop(const ros::TimerEvent&) {
        if (!global_path_ || !odom_) return;

        double ego_x = odom_->pose.pose.position.x;
        double ego_y = odom_->pose.pose.position.y;
        tf::Quaternion q(odom_->pose.pose.orientation.x, odom_->pose.pose.orientation.y,
                       odom_->pose.pose.orientation.z, odom_->pose.pose.orientation.w);
        double roll, pitch, yaw;
        tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

        int start_idx = getClosestIndex(ego_x, ego_y, yaw);
        int n_pts = global_path_->poses.size();
        if (start_idx >= n_pts - 5) return;

        double accum = 0.0;
        int end_idx = start_idx;
        while (end_idx < n_pts - 1 && accum < horizon_dist_) {
            double p1x = global_path_->poses[end_idx].pose.position.x;
            double p1y = global_path_->poses[end_idx].pose.position.y;
            double p2x = global_path_->poses[end_idx + 1].pose.position.x;
            double p2y = global_path_->poses[end_idx + 1].pose.position.y;
            accum += std::hypot(p2x - p1x, p2y - p1y);
            end_idx++;
        }

        int sub_len = end_idx - start_idx + 1;
        if (sub_len < 3) return;

        std::vector<double> s_vec(sub_len, 0.0), x_vec(sub_len), y_vec(sub_len);
        for (int i = 0; i < sub_len; ++i) {
            x_vec[i] = global_path_->poses[start_idx + i].pose.position.x;
            y_vec[i] = global_path_->poses[start_idx + i].pose.position.y;
            if (i > 0) {
                s_vec[i] = s_vec[i - 1] + std::hypot(x_vec[i] - x_vec[i - 1], y_vec[i] - y_vec[i - 1]);
            }
        }

        SimpleCubicSpline cs;
        cs.set_points(s_vec, x_vec, y_vec);

        std::vector<double> s_dense, x_base, y_base, yaw_base;
        for (double s = 0.0; s <= s_vec.back(); s += step_size_) {
            double xb, yb, ywb;
            cs.calc(s, xb, yb, ywb);
            s_dense.push_back(s);
            x_base.push_back(xb);
            y_base.push_back(yb);
            yaw_base.push_back(ywb);
        }
        int n_dense = s_dense.size();
        if (n_dense < 3) return;

        // Ego lateral offset calculation
        double dx0 = ego_x - x_base[0];
        double dy0 = ego_y - y_base[0];
        double d_ego = -dx0 * std::sin(yaw_base[0]) + dy0 * std::cos(yaw_base[0]);
        d_ego = std::max(-max_drivable_offset_, std::min(max_drivable_offset_, d_ego));

        // Filter road obstacles
        std::vector<Obstacle> road_obs;
        for (const auto& ob : obstacle_list_) {
            double min_d = 1e9;
            int min_i = 0;
            for (int i = 0; i < n_dense; ++i) {
                double dist = std::hypot(x_base[i] - ob.x, y_base[i] - ob.y);
                if (dist < min_d) { min_d = dist; min_i = i; }
            }
            double lat = -(ob.x - x_base[min_i]) * std::sin(yaw_base[min_i]) + (ob.y - y_base[min_i]) * std::cos(yaw_base[min_i]);
            if (std::abs(lat) - ob.r <= (max_drivable_offset_ + 0.6)) {
                road_obs.push_back(ob);
            }
        }

        double closest_obs_s = 1e6;
        for (const auto& ob : road_obs) {
            for (int i = 0; i < n_dense; ++i) {
                if (std::hypot(x_base[i] - ob.x, y_base[i] - ob.y) - ob.r < 3.5) {
                    closest_obs_s = std::min(closest_obs_s, s_dense[i]);
                    break;
                }
            }
        }

        double trans_s = (closest_obs_s < 1e5) ? std::max(6.0, std::min(20.0, closest_obs_s * 0.75))
                                               : std::max(5.0, std::min(12.0, std::abs(d_ego) * 3.5 + 5.0));

        // Evaluate Rollouts
        double best_offset = 0.0;
        double best_cost = 1e9;
        bool any_valid = false;
        std::vector<std::vector<std::pair<double, double>>> rollout_geometries;
        std::vector<double> foot_offsets = {0.0, wheelbase_ * 0.45, wheelbase_ * 0.9};

        for (double d : offsets_) {
            QuinticPolynomial qp(d_ego, 0.0, 0.0, d, 0.0, 0.0, trans_s);
            std::vector<std::pair<double, double>> rollout(n_dense);
            for (int i = 0; i < n_dense; ++i) {
                double lat = (s_dense[i] < trans_s) ? qp.calc(s_dense[i]) : d;
                rollout[i].first = x_base[i] - std::sin(yaw_base[i]) * lat;
                rollout[i].second = y_base[i] + std::cos(yaw_base[i]) * lat;
            }
            rollout_geometries.push_back(rollout);

            bool collision = false;
            double min_obs_dist = 1e6;

            if (enable_avoidance_ && !road_obs.empty()) {
                for (const auto& ob : road_obs) {
                    for (int i = 0; i < n_dense; ++i) {
                        for (double off : foot_offsets) {
                            double cx = rollout[i].first + off * std::cos(yaw_base[i]);
                            double cy = rollout[i].second + off * std::sin(yaw_base[i]);
                            double dist = std::hypot(cx - ob.x, cy - ob.y) - ob.r;
                            min_obs_dist = std::min(min_obs_dist, dist);
                            if (dist < safety_margin_) {
                                collision = true;
                                break;
                            }
                        }
                        if (collision) break;
                    }
                    if (collision) break;
                }
            }

            if (collision) continue;

            any_valid = true;
            double obs_cost = (min_obs_dist < 3.0) ? (3.0 - min_obs_dist) * (3.0 - min_obs_dist) * 18.0 : 0.0;
            double total_cost = std::pow(std::abs(d), 1.5) * 8.0 + std::abs(d - target_offset_) * 1.5 + obs_cost;
            if (total_cost < best_cost) {
                best_cost = total_cost;
                best_offset = d;
            }
        }

        if (any_valid) {
            target_offset_ = best_offset;
        } else {
            target_offset_ = filtered_offset_;
        }

        double step = std::max(-max_offset_rate_, std::min(max_offset_rate_, target_offset_ - filtered_offset_));
        filtered_offset_ += step;
        filtered_offset_ = std::max(-max_drivable_offset_, std::min(max_drivable_offset_, filtered_offset_));

        // Generate Executed Trajectory
        QuinticPolynomial qp_active(d_ego, 0.0, 0.0, filtered_offset_, 0.0, 0.0, trans_s);
        std::vector<std::pair<double, double>> selected_xy(n_dense);
        std::vector<double> sel_yaw = yaw_base;

        for (int i = 0; i < n_dense; ++i) {
            double lat = (s_dense[i] < trans_s) ? qp_active.calc(s_dense[i]) : filtered_offset_;
            selected_xy[i].first = x_base[i] - std::sin(yaw_base[i]) * lat;
            selected_xy[i].second = y_base[i] + std::cos(yaw_base[i]) * lat;
        }

        for (int i = 0; i < n_dense - 1; ++i) {
            double hdg = std::atan2(selected_xy[i + 1].second - selected_xy[i].second,
                                    selected_xy[i + 1].first - selected_xy[i].first);
            double diff = std::atan2(std::sin(hdg - yaw_base[i]), std::cos(hdg - yaw_base[i]));
            if (std::abs(diff) < M_PI / 3.0) sel_yaw[i] = hdg;
        }
        sel_yaw.back() = sel_yaw[n_dense - 2];

        // Curvature calculation
        std::vector<double> curvature(n_dense, 0.0);
        for (int i = 1; i < n_dense - 1; ++i) {
            double x1 = selected_xy[i - 1].first, y1 = selected_xy[i - 1].second;
            double x2 = selected_xy[i].first, y2 = selected_xy[i].second;
            double x3 = selected_xy[i + 1].first, y3 = selected_xy[i + 1].second;
            double area = 0.5 * (x1 * (y2 - y3) + x2 * (y3 - y1) + x3 * (y1 - y2));
            double a = std::hypot(x2 - x1, y2 - y1);
            double b = std::hypot(x3 - x2, y3 - y2);
            double c = std::hypot(x1 - x3, y1 - y3);
            if (a * b * c > 1e-6) curvature[i] = (4.0 * std::abs(area)) / (a * b * c);
        }

        // Speed Profiling
        double closest_path_obs = 1e6;
        for (const auto& ob : road_obs) {
            for (int i = 0; i < n_dense; ++i) {
                if (std::hypot(selected_xy[i].first - ob.x, selected_xy[i].second - ob.y) - ob.r < (vehicle_width_ / 2.0 + 0.15)) {
                    closest_path_obs = std::min(closest_path_obs, s_dense[i]);
                    break;
                }
            }
        }

        std::vector<double> target_speeds(n_dense);
        for (int i = 0; i < n_dense; ++i) {
            double kappa = curvature[i];
            double v_curve = (kappa > 1e-3) ? std::sqrt(max_lat_accel_ / kappa) : max_speed_ms_;
            double v = std::min(max_speed_ms_, std::max(min_speed_kmh_ * KMH_TO_MS, v_curve));

            if (closest_path_obs < 35.0) {
                double stop_s = std::max(0.0, closest_path_obs - min_stop_dist_);
                v = (s_dense[i] >= stop_s) ? 0.0 : std::min(v, std::sqrt(std::max(0.0, 2.0 * max_decel_ * (stop_s - s_dense[i]))));
            }
            if (!any_valid && closest_path_obs < 5.0) v = 0.0;
            target_speeds[i] = v;
        }

        // Publish Trajectory Message
        ros::Time stamp = ros::Time::now();
        global_path_planner::PlannerTrajectory traj;
        traj.header.frame_id = frame_id_;
        traj.header.stamp = stamp;
        for (int i = 0; i < n_dense; ++i) {
            global_path_planner::PlannerWaypoint wp;
            wp.position.x = selected_xy[i].first;
            wp.position.y = selected_xy[i].second;
            wp.position.z = 0.0;
            wp.yaw = sel_yaw[i];
            wp.target_speed = target_speeds[i];
            wp.curvature = curvature[i];
            traj.waypoints.push_back(wp);
        }
        traj_pub_.publish(traj);

        std_msgs::String status_msg;
        status_msg.data = any_valid ? "TRACKING" : "STOP";
        status_pub_.publish(status_msg);

        // Publish RViz Markers
        publishMarkers(rollout_geometries, selected_xy, sel_yaw, target_speeds, stamp);
    }

    void publishMarkers(const std::vector<std::vector<std::pair<double, double>>>& rollout_geometries,
                        const std::vector<std::pair<double, double>>& selected_xy,
                        const std::vector<double>& sel_yaw,
                        const std::vector<double>& target_speeds,
                        const ros::Time& stamp) {
        visualization_msgs::MarkerArray rollout_array;
        int best_idx = 0;
        double min_diff = 1e9;
        for (size_t i = 0; i < offsets_.size(); ++i) {
            double diff = std::abs(offsets_[i] - filtered_offset_);
            if (diff < min_diff) { min_diff = diff; best_idx = i; }
        }

        for (size_t i = 0; i < rollout_geometries.size(); ++i) {
            visualization_msgs::Marker m;
            m.header.frame_id = frame_id_;
            m.header.stamp = stamp;
            m.ns = "rollouts";
            m.id = i;
            m.type = visualization_msgs::Marker::LINE_STRIP;
            m.action = visualization_msgs::Marker::ADD;
            for (const auto& pt : rollout_geometries[i]) {
                geometry_msgs::Point p;
                p.x = pt.first; p.y = pt.second; p.z = 0.1;
                m.points.push_back(p);
            }
            if (static_cast<int>(i) == best_idx) {
                m.scale.x = 0.15;
                m.color.r = 0.0; m.color.g = 1.0; m.color.b = 0.0; m.color.a = 1.0;
            } else {
                m.scale.x = 0.04;
                m.color.r = 0.6; m.color.g = 0.6; m.color.b = 0.6; m.color.a = 0.3;
            }
            rollout_array.markers.push_back(m);
        }
        rollout_pub_.publish(rollout_array);

        visualization_msgs::MarkerArray speed_array;
        visualization_msgs::Marker del_m;
        del_m.action = visualization_msgs::Marker::DELETEALL;
        speed_array.markers.push_back(del_m);

        int interval = std::max(1, static_cast<int>(2.0 / step_size_));
        int m_id = 0;
        for (size_t i = 0; i < selected_xy.size(); i += interval) {
            double sp_kmh = target_speeds[i] * MS_TO_KMH;
            double yaw_i = sel_yaw[i];

            visualization_msgs::Marker m;
            m.header.frame_id = frame_id_;
            m.header.stamp = stamp;
            m.ns = "speed_footprints";
            m.id = m_id++;
            m.type = visualization_msgs::Marker::CUBE;
            m.action = visualization_msgs::Marker::ADD;
            m.pose.position.x = selected_xy[i].first;
            m.pose.position.y = selected_xy[i].second;
            m.pose.position.z = 0.15;
            m.pose.orientation.z = std::sin(yaw_i / 2.0);
            m.pose.orientation.w = std::cos(yaw_i / 2.0);
            m.scale.x = vehicle_length_; m.scale.y = vehicle_width_; m.scale.z = 0.05;

            if (sp_kmh < 5.0) {
                m.color.r = 1.0; m.color.g = 0.1; m.color.b = 0.1; m.color.a = 0.6;
            } else {
                double norm = std::max(0.0, std::min(1.0, sp_kmh / max_speed_kmh_));
                m.color.r = std::max(0.0, std::min(1.0, 2.0 * (1.0 - norm)));
                m.color.g = std::max(0.0, std::min(1.0, 2.0 * norm));
                m.color.b = 0.15; m.color.a = 0.45;
            }
            speed_array.markers.push_back(m);

            visualization_msgs::Marker tm;
            tm.header.frame_id = frame_id_;
            tm.header.stamp = stamp;
            tm.ns = "speed_text";
            tm.id = m_id++;
            tm.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            tm.action = visualization_msgs::Marker::ADD;
            tm.pose.position.x = selected_xy[i].first;
            tm.pose.position.y = selected_xy[i].second;
            tm.pose.position.z = 0.8;
            tm.scale.z = 0.7;
            tm.color.r = 1.0; tm.color.g = 1.0; tm.color.b = 1.0; tm.color.a = 1.0;
            char buf[32];
            std::snprintf(buf, sizeof(buf), "%.0f km/h", sp_kmh);
            tm.text = buf;
            speed_array.markers.push_back(tm);
        }
        speed_pub_.publish(speed_array);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "op_frenet_local_planner_node");
    OpenPlannerCppNode node;
    ros::spin();
    return 0;
}
