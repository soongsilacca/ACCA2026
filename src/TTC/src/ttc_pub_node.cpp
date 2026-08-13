#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>
#include <autoware_msgs/DetectedObjectArray.h>
#include <tf/transform_listener.h>
#include <algorithm>
#include <cmath>

class TTCPublisher {
public:
    TTCPublisher() : private_nh_("~"), has_velocity_(false), has_pose_(false), filtered_w_(0.0) {
        // 파라미터 로드
        private_nh_.param<double>("ttc_danger_threshold", ttc_danger_threshold_, 1.2); 
        private_nh_.param<double>("ego_width", ego_width_, 1.8);                       
        private_nh_.param<double>("lpf_alpha", lpf_alpha_, 0.2);                      
        private_nh_.param<double>("obstacle_detect_dist", obstacle_detect_dist_, 30.0);
        private_nh_.param<double>("publish_hz", publish_hz_, 10.0);

        
        ttc_pub_ = nh_.advertise<std_msgs::Float32>("/ttc", 1);
        obs_exists_pub_ = nh_.advertise<std_msgs::Bool>("/obstacle/exists", 1);
        obs_front_dist_pub_ = nh_.advertise<std_msgs::Float32>("/obstacle/front_distance", 1);
        obs_in_path_pub_ = nh_.advertise<std_msgs::Bool>("/obstacle/in_ego_path", 1);
        obs_avoid_possible_pub_ = nh_.advertise<std_msgs::Bool>("/obstacle/avoidance_possible", 1);
        
       
        std::string objects_topic, odometry_topic;
        private_nh_.param<std::string>("objects_topic", objects_topic,
                                       "/estimation/objects_with_velocity");
        private_nh_.param<std::string>("odometry_topic", odometry_topic,
                                       "/localization/kinematic_state");

        objects_sub_ = nh_.subscribe(objects_topic, 1, &TTCPublisher::objectsCallback, this);
        odometry_sub_ = nh_.subscribe(odometry_topic, 1, &TTCPublisher::odometryCallback, this);

      
        timer_ = nh_.createTimer(ros::Duration(1.0 / std::max(publish_hz_, 0.1)), &TTCPublisher::timerCallback, this);

        ROS_INFO("TTC Publisher Node Initialized (Visualization-free).");
    }

private:
    void odometryCallback(const nav_msgs::Odometry& msg) {
        current_velocity_ = msg.twist.twist;
        current_pose_ = msg.pose.pose;
        has_velocity_ = true;
        has_pose_ = true;
        filtered_w_ = lpf_alpha_ * msg.twist.twist.angular.z
                    + (1.0 - lpf_alpha_) * filtered_w_;
    }

    void objectsCallback(const autoware_msgs::DetectedObjectArray& msg) {
        latest_objects_ = msg;
    }

    double calculateTTC(const autoware_msgs::DetectedObject& obj, bool& in_path) {
        in_path = false;
        double v = current_velocity_.linear.x;
        if (std::abs(v) < 0.2) return 999.9;

        double dx = obj.pose.position.x - current_pose_.position.x;
        double dy = obj.pose.position.y - current_pose_.position.y;
        double ego_yaw = tf::getYaw(current_pose_.orientation);

        double local_x = dx * std::cos(-ego_yaw) - dy * std::sin(-ego_yaw);
        double local_y = dx * std::sin(-ego_yaw) + dy * std::cos(-ego_yaw);

        if (local_x < 0.0 || local_x > obstacle_detect_dist_) return 999.9;

        double obj_v_map = std::sqrt(std::pow(obj.velocity.linear.x, 2) + std::pow(obj.velocity.linear.y, 2));
        double obj_yaw = std::atan2(obj.velocity.linear.y, obj.velocity.linear.x);
        
        double rel_heading = obj_yaw - ego_yaw;
        while (rel_heading > M_PI) rel_heading -= 2.0 * M_PI;
        while (rel_heading < -M_PI) rel_heading += 2.0 * M_PI;

        // 📐 시나리오 1: 직각 측면 충돌 연산
        if (std::abs(rel_heading) > M_PI/3.0 && std::abs(rel_heading) < 2.0*M_PI/3.0 && obj_v_map > 0.5) {
            double local_obj_vx = obj_v_map * std::cos(rel_heading);
            double local_obj_vy = obj_v_map * std::sin(rel_heading);

            if (std::abs(local_obj_vy) > 0.1) {
                double t_obj = -local_y / local_obj_vy;
                double intersection_x = local_x + local_obj_vx * t_obj;

                if (t_obj > 0.0 && intersection_x > 0.0 && intersection_x <= obstacle_detect_dist_) {
                    double t_ego = intersection_x / v;
                    double time_gap = std::abs(t_ego - t_obj);
                    double margin = (ego_width_ + obj.dimensions.x) / (2.0 * std::max(v, 0.1)) + 1.0;
                    
                    if (time_gap < margin) {
                        in_path = true;
                        return t_ego;
                    }
                }
            }
        }

       
        double arc_distance = 0.0;
        bool lateral_match = false;

        if (std::abs(filtered_w_) < 0.01) {
            arc_distance = local_x;
            if (std::abs(local_y) <= (ego_width_ + obj.dimensions.y) / 2.0 + 0.5) lateral_match = true;
        } else {
            double R = v / filtered_w_;
            double center_y = R;
            double angle_to_obj = std::atan2(local_y - center_y, local_x - 0.0);
            double start_angle = std::atan2(0.0 - center_y, 0.0 - 0.0);
            double delta_theta = angle_to_obj - start_angle;

            while (delta_theta > M_PI) delta_theta -= 2.0 * M_PI;
            while (delta_theta < -M_PI) delta_theta += 2.0 * M_PI;

            arc_distance = R * delta_theta;
            double lat_gap = std::abs(std::sqrt(std::pow(local_x, 2) + std::pow(local_y - center_y, 2)) - std::abs(R));
            if (lat_gap <= (ego_width_ + obj.dimensions.y) / 2.0 + 0.5) lateral_match = true;
        }

        if (lateral_match && arc_distance > 0.0 && arc_distance <= obstacle_detect_dist_) {
            in_path = true;
            double rel_v = v - obj_v_map * std::cos(obj_yaw - ego_yaw);
            if (std::abs(rel_v) < 0.1) return 999.9;
            double ttc = arc_distance / rel_v;
            if (ttc > 0.0) return ttc;
        }
        return 999.9;
    }

    void timerCallback(const ros::TimerEvent& event) {
        if (!has_velocity_ || !has_pose_) return;

        double min_ttc = 999.9;
        bool obstacle_exists = !latest_objects_.objects.empty();
        double min_front_dist = 999.9;
        bool obstacle_in_ego_path = false;
        bool avoidance_possible = true;

        double ego_yaw = tf::getYaw(current_pose_.orientation);

        for (const auto& obj : latest_objects_.objects) {
            double dx = obj.pose.position.x - current_pose_.position.x;
            double dy = obj.pose.position.y - current_pose_.position.y;
            double local_x = dx * std::cos(-ego_yaw) - dy * std::sin(-ego_yaw);
            
            if (local_x > 0.0 && local_x <= obstacle_detect_dist_ && local_x < min_front_dist) {
                min_front_dist = local_x;
            }

            bool is_in_path = false;
            double ttc = calculateTTC(obj, is_in_path);
            if (is_in_path) {
                obstacle_in_ego_path = true;
                if (ttc < min_ttc) min_ttc = ttc;
            }
        }

        if (obstacle_in_ego_path && min_ttc < ttc_danger_threshold_) {
            avoidance_possible = false;
        }

        // 토픽 발행
        std_msgs::Float32 ttc_msg; ttc_msg.data = min_ttc;
        std_msgs::Bool exists_msg; exists_msg.data = obstacle_exists;
        std_msgs::Float32 dist_msg; dist_msg.data = (min_front_dist > 990.0) ? -1.0 : min_front_dist;
        std_msgs::Bool in_path_msg; in_path_msg.data = obstacle_in_ego_path;
        std_msgs::Bool avoid_msg; avoid_msg.data = avoidance_possible;

        ttc_pub_.publish(ttc_msg);
        obs_exists_pub_.publish(exists_msg);
        obs_front_dist_pub_.publish(dist_msg);
        obs_in_path_pub_.publish(in_path_msg);
        obs_avoid_possible_pub_.publish(avoid_msg);
    }

    ros::NodeHandle nh_, private_nh_;
    ros::Timer timer_;
    ros::Subscriber objects_sub_, odometry_sub_;
    ros::Publisher ttc_pub_, obs_exists_pub_, obs_front_dist_pub_, obs_in_path_pub_, obs_avoid_possible_pub_;

    geometry_msgs::Twist current_velocity_;
    geometry_msgs::Pose current_pose_;
    autoware_msgs::DetectedObjectArray latest_objects_;
    bool has_velocity_, has_pose_;
    double filtered_w_, ttc_danger_threshold_, ego_width_, lpf_alpha_, obstacle_detect_dist_, publish_hz_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "ttc_pub_node");
    TTCPublisher pub_node;
    ros::spin();
    return 0;
}
