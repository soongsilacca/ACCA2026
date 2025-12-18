#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class ImageConcatNode : public rclcpp::Node {
public:
    ImageConcatNode() : Node("bs_cam_concat") {
        // 두 개의 구독자 설정
        image_sub1_ = this->create_subscription<sensor_msgs::msg::Image>(
            "camera1/image_raw", 10,
            std::bind(&ImageConcatNode::imageCallback1, this, std::placeholders::_1));
        image_sub2_ = this->create_subscription<sensor_msgs::msg::Image>(
            "camera2/image_raw", 10,
            std::bind(&ImageConcatNode::imageCallback2, this, std::placeholders::_1));
        image_sub3_ = this->create_subscription<sensor_msgs::msg::Image>(
        "camera3/image_raw", 10,
        std::bind(&ImageConcatNode::imageCallback3, this, std::placeholders::_1));

        // 퍼블리셔 설정
        image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("concated_cam", 10);
    }

private:
    void imageCallback1(const sensor_msgs::msg::Image::SharedPtr msg) {
        cv_image1_ = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        tryPublish();
    }

    void imageCallback2(const sensor_msgs::msg::Image::SharedPtr msg) {
        cv_image2_ = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        tryPublish();
    }

    void imageCallback3(const sensor_msgs::msg::Image::SharedPtr msg) {
        cv_image3_ = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        tryPublish();
    }

    void tryPublish() {
        if (cv_image1_ && cv_image2_ && cv_image3_) {
            // 이미지 합치기
            cv::Mat concat_image;
            cv::hconcat(cv_image1_->image, cv_image2_->image, concat_image);
            cv::hconcat(concat_image, cv_image3_->image, concat_image);

            // sensor_msgs::msg::Image로 변환하여 퍼블리시
            auto concat_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", concat_image).toImageMsg();
            image_pub_->publish(*concat_msg);
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub1_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub2_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub3_;

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;

    cv_bridge::CvImagePtr cv_image1_;
    cv_bridge::CvImagePtr cv_image2_;
    cv_bridge::CvImagePtr cv_image3_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImageConcatNode>());
    rclcpp::shutdown();
    return 0;
}
