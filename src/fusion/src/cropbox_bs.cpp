#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <iostream>
#include <vector>

using std::placeholders::_1;

class VelodyneSubscriber : public rclcpp::Node
{
public:
    VelodyneSubscriber() : Node("bs_cropbox_filter")
    {
        velodyne_raw_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/velodyne_points", 10,
            std::bind(&VelodyneSubscriber::velodyneCallback, this, _1));

        cropbox_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/cropbox_filtered", 10);

// =============== cropbox 사이즈 바꾸는 부분 ==================
        float_size = 4;

        //Cone Tracking
        // min_x = 0;
        // max_x = 8.0;
        // min_y = -4.0;
        // max_y = 4.0;
        // min_z = -2.0; 
        // max_z = -0.25;
        min_x = 0;
        max_x = 5.0;
        min_y = -3.0;
        max_y = 3.0;
        min_z = -2.0; 
        max_z = -0.25;


        // Parking
        // min_x = 0;
        // max_x = 3.0;
        // min_y = -6.0;
        // max_y = 0.0;
        // min_z = -1.5; 
        // max_z = -0.5;
// =========================================================
    }

private:
    size_t float_size;
    float min_x;
    float max_x;  
    float min_y;   
    float max_y;
    float min_z;  
    float max_z;   

    void velodyneCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        std::vector<uint8_t> filtered_data;

        int x_offset = -1;
        int y_offset = -1;
        int z_offset = -1; 
        for (const auto &field : msg->fields) {
            if (field.name == "x") {
                x_offset = field.offset;
            } else if (field.name == "y") {
                y_offset = field.offset;
            } else if (field.name == "z") {  
                z_offset = field.offset;
            }
        }
        if (x_offset == -1 || y_offset == -1 || z_offset == -1) {
            RCLCPP_ERROR(this->get_logger(), "PointCloud2 message does not have 'x', 'y', or 'z' fields.");
            return;
        }

        for (size_t i = 0; i < msg->data.size(); i += msg->point_step)
        {
            float x, y, z;
            std::memcpy(&x, &msg->data[i + x_offset], float_size);
            std::memcpy(&y, &msg->data[i + y_offset], float_size);
            std::memcpy(&z, &msg->data[i + z_offset], float_size);
            if (x >= min_x && x <= max_x && y >= min_y && y <= max_y && z >= min_z && z <= max_z)
            {
                for (size_t j = 0; j < msg->point_step; ++j)
                {
                    filtered_data.push_back(msg->data[i + j]);
                }
            }
        }

        auto processed_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
        processed_msg->header = msg->header;

        processed_msg->height = 1;
        processed_msg->width = filtered_data.size() / msg->point_step;
        processed_msg->fields = msg->fields;
        processed_msg->is_bigendian = msg->is_bigendian;
        processed_msg->point_step = msg->point_step;
        processed_msg->row_step = processed_msg->point_step * processed_msg->width;
        processed_msg->data = filtered_data;
        processed_msg->is_dense = true;

        if (processed_msg->width * processed_msg->point_step != processed_msg->data.size()) {
            RCLCPP_ERROR(this->get_logger(), "Data size (%zu bytes) does not match width (%u) times point_step (%u).",
                         processed_msg->data.size(), processed_msg->width, processed_msg->point_step);
        } else {
            cropbox_publisher_->publish(*processed_msg);
        }
    }
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cropbox_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr velodyne_raw_subscriber_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<VelodyneSubscriber>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}
