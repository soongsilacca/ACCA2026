#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <morai_msgs/CtrlCmd.h>

class TTCSubscriber {
public:
    TTCSubscriber() : private_nh_("~"), ttc_(999.9), exists_(false), front_dist_(-1.0), in_path_(false), avoidable_(true) {
        private_nh_.param<double>("ttc_danger_threshold", ttc_danger_threshold_, 1.2);

        sub_ttc_ = nh_.subscribe("/ttc", 10, &TTCSubscriber::ttcCallback, this);
        sub_exists_ = nh_.subscribe("/obstacle/exists", 10, &TTCSubscriber::existsCallback, this);
        sub_dist_ = nh_.subscribe("/obstacle/front_distance", 10, &TTCSubscriber::distCallback, this);
        sub_in_path_ = nh_.subscribe("/obstacle/in_ego_path", 10, &TTCSubscriber::inPathCallback, this);
        sub_avoid_ = nh_.subscribe("/obstacle/avoidance_possible", 10, &TTCSubscriber::avoidCallback, this);

        std::string control_topic;
        private_nh_.param<std::string>("control_topic", control_topic, "/ctrl_cmd");
        pub_ctrl_ = nh_.advertise<morai_msgs::CtrlCmd>(control_topic, 10);
        pub_stop_ = nh_.advertise<std_msgs::Bool>("/behavior/stop_request", 10);
        
        timer_ = nh_.createTimer(ros::Duration(0.1), &TTCSubscriber::displayCallback, this);
        ROS_INFO("TTC Subscriber Node Initialized. Monitoring diagnostics and commanding stop if needed...");
    }

private:
    void ttcCallback(const std_msgs::Float32::ConstPtr& msg) { ttc_ = msg->data; }
    void existsCallback(const std_msgs::Bool::ConstPtr& msg) { exists_ = msg->data; }
    void distCallback(const std_msgs::Float32::ConstPtr& msg) { front_dist_ = msg->data; }
    void inPathCallback(const std_msgs::Bool::ConstPtr& msg) { in_path_ = msg->data; }
    void avoidCallback(const std_msgs::Bool::ConstPtr& msg) { avoidable_ = msg->data; }

    void displayCallback(const ros::TimerEvent& event) {
        if (in_path_ && ttc_ < ttc_danger_threshold_) {
           
            ROS_WARN("[SUB] CRITICAL! TTC: %.2f s | Dist: %.2f m | Avoidable: %s. STOPPING THE VEHICLE!", 
                     ttc_, front_dist_, avoidable_ ? "YES" : "NO");
            
            morai_msgs::CtrlCmd ctrl;
            ctrl.ctrl_mode = 2;
            ctrl.cmd_type = 1;
            ctrl.gear = 4;
            ctrl.steer = 0.0;
            ctrl.accel = 0.0;
            ctrl.brake = 1.0;
            pub_ctrl_.publish(ctrl);

            std_msgs::Bool stop_msg;
            stop_msg.data = true;
            pub_stop_.publish(stop_msg);
            
        } else {
            
            ROS_INFO("[SUB] Exists: %s | InPath: %s | Front Dist: %.2f m | TTC: %.2f s", 
                     exists_ ? "YES" : "NO", in_path_ ? "YES" : "NO", front_dist_, ttc_);
        }
    }

    ros::NodeHandle nh_, private_nh_;
    ros::Timer timer_;
    ros::Subscriber sub_ttc_, sub_exists_, sub_dist_, sub_in_path_, sub_avoid_;
    ros::Publisher pub_ctrl_, pub_stop_;
    double ttc_, front_dist_, ttc_danger_threshold_;
    bool exists_, in_path_, avoidable_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "ttc_sub_node");
    TTCSubscriber sub_node;
    ros::spin();
    return 0;
}
