#include <chrono> // 시간 관련 기능
#include <functional> // function, lambda, binding
#include <memory> // 스마트 포인터기능
#include <string> // 문자열 msg type
#include <cstdio>
#include<iostream>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include <fstream>
#include <vector>

using namespace std;
using namespace std::chrono_literals;  //시간 관련 리터럴(표준 라이브러리 문법)

class Pub_node : public rclcpp::Node
{
    public:
        Pub_node()
        : Node("Pub_node")
        {
            auto qos_profile = rclcpp::SystemDefaultsQoS();

            Pub_node_ = this -> create_publisher<std_msgs::msg::String>(
                "topicpp",
                qos_profile);

            timer_ = this -> create_wall_timer(
                500ms, std::bind(&Pub_node::publish, this));
            string s;

            fin_.open("/home/ps/robot_ws/src/hw1_c/query.dat");
            if (fin_.is_open()) {
                cout << "ok" << endl;
            }
        }
            

    
    private:
        void publish()
    
        {
            if (fin_.is_open()) {

            string x;
            getline(fin_,x);
            cout << x << endl;
            auto msg = std_msgs::msg::String();
            msg.data = x;
            Pub_node_->publish(msg);
            }
            else {
                fin_.open("/home/ps/robot_ws/src/hw1_c/query.dat");
                cout << "oh no" <<endl;
            }
        }
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr Pub_node_;
        rclcpp::TimerBase::SharedPtr timer_;
        ifstream fin_;
        

};



int main(int argc, char * argv[])
{
    ifstream fin;
    fin.open("/home/ps/robot_ws/src/hw1_c/query.dat");
    if (fin.is_open()) {
        cout << "ok" << endl;
    }
    cout << "oh" << endl;
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<Pub_node>());
    rclcpp::shutdown();
    return 0;
}