#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class VideoDecoderNode : public rclcpp::Node {
public:
    VideoDecoderNode() : Node("video_decoder_node") {
        this->declare_parameter<std::string>("encoded_video_topic", "/encoded_video");
        std::string encoded_video_topic = this->get_parameter("encoded_video_topic").as_string();

        encoded_sub_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
                encoded_video_topic, 10,
                std::bind(&VideoDecoderNode::decodeCallback, this, std::placeholders::_1));

        image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("decoded_image", 10);

        RCLCPP_INFO(this->get_logger(), "VideoDecoderNode initialized, subscribed to: %s", encoded_video_topic.c_str());
    }

private:
    void decodeCallback(const sensor_msgs::msg::CompressedImage::SharedPtr msg) {
        try {
            cv::Mat decoded_image = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);
            if (decoded_image.empty()) {
                RCLCPP_ERROR(this->get_logger(), "Failed to decode image");
                return; //TODO: Have proper error handling for failed decoding
            }

            // OpenCV image to a ROS Image message
            std_msgs::msg::Header header;
            header.stamp = this->now();
            header.frame_id = "camera_frame";
            sensor_msgs::msg::Image::SharedPtr img_msg =
                    cv_bridge::CvImage(header, "bgr8", decoded_image).toImageMsg();

            // Publish the decoded image
            image_pub_->publish(*img_msg);
        }
        catch (cv::Exception &e) {
            RCLCPP_ERROR(this->get_logger(), "OpenCV exception: %s", e.what());
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr encoded_sub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VideoDecoderNode>());
    rclcpp::shutdown();
    return 0;
}
