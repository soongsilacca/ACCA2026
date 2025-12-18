#include <chrono> // 시간 관련 기능
#include <functional> // function, lambda, binding
#include <memory> // 스마트 포인터기능
#include <string> // 문자열 msg type
#include <cstdio>
#include<iostream>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/int16.hpp"

#include <fstream>
#include <vector>

using namespace std;
using namespace std::chrono_literals;  

class Sub_node : public rclcpp::Node
{
    public:
        Sub_node()
        : Node("Sub_node")
        {
            auto qos_profile = rclcpp::SystemDefaultsQoS();

            sub_node_ =this -> create_subscription<std_msgs::msg::String>(
                "topicpp",
                qos_profile,
                std::bind(&Sub_node::subscribe, this, std::placeholders::_1)
                );

            get_set_pub_ = this -> create_publisher<std_msgs::msg::String>(
                "get_set",
                qos_profile
            );
            get_count_pub_ = this -> create_publisher<std_msgs::msg::Int16>(
                "get_count",
                qos_profile
            );
            set_count_pub_ = this -> create_publisher<std_msgs::msg::Int16>(
                "set_count",
                qos_profile
            );

            get_count_ = 0;
            set_count_ = 0;

        }

    private:
        void subscribe(const std_msgs::msg::String & msg) 
        {   
            auto gs_msg = std_msgs::msg::String();

            RCLCPP_INFO(this->get_logger(), "I heard '%s'", msg.data.c_str());
            string ans = msg.data.substr(0,4);
            gs_msg.data = ans;
            get_set_pub_ -> publish(gs_msg);
            string get = "get";
            cout << ans.compare(get) << endl;
            if (ans.compare(get) == 1) {
                get_count_ = get_count_ +1;
                auto get_count = std_msgs::msg::Int16();
                get_count.data = get_count_;
                get_count_pub_ -> publish(get_count);
            }
            else if (ans.compare(get) == 12) {
                set_count_ = set_count_ + 1;
                auto set_count = std_msgs::msg::Int16();
                set_count.data = set_count_;

                set_count_pub_ -> publish(set_count);

            }
            
        }
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_node_;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr get_set_pub_;
        rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr get_count_pub_;
        rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr set_count_pub_;
        int get_count_;
        int set_count_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Sub_node>());
    rclcpp::shutdown();
    return 0;
}