#include <cstdio>
#include <iostream>

#include "depthai_bridge/BridgePublisher.hpp"     // Not used but included for ease of access
#include "depthai_bridge/ImageConverter.hpp"
#include "rclcpp/executors.hpp"
#include "rclcpp/node.hpp"
#include "sensor_msgs/msg/image.hpp"              // Not used but included for ease of access
#include "sensor_msgs/msg/compressed_image.hpp"

// Includes common necessary includes for development using depthai library
#include "depthai/device/DataQueue.hpp"
#include "depthai/device/Device.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/node/ColorCamera.hpp"
#include "depthai/pipeline/node/MonoCamera.hpp"   // Not used but included for ease of access
#include <depthai/pipeline/node/VideoEncoder.hpp>
#include "depthai/pipeline/node/XLinkOut.hpp"

dai::Pipeline createPipeline() {
    dai::Pipeline pipeline;

    auto colorCam = pipeline.create<dai::node::ColorCamera>();
    auto xlinkOut = pipeline.create<dai::node::XLinkOut>();
    auto videoEnc = pipeline.create<dai::node::VideoEncoder>();

    xlinkOut->setStreamName("encoded_video");

    colorCam->setFps(5);
    colorCam->setResolution(dai::ColorCameraProperties::SensorResolution::THE_1080_P);
    colorCam->setBoardSocket(dai::CameraBoardSocket::CAM_A);
    colorCam->setInterleaved(true);
    colorCam->setColorOrder(dai::ColorCameraProperties::ColorOrder::BGR);

    videoEnc->setDefaultProfilePreset(colorCam->getFps(), dai::VideoEncoderProperties::Profile::MJPEG);
    videoEnc->setQuality(10);
    videoEnc->setFrameRate(5);

    colorCam->video.link(videoEnc->input);
    videoEnc->bitstream.link(xlinkOut->input);

    return pipeline;
}

class MobileNetPublisherNode : public rclcpp::Node {
public:
    MobileNetPublisherNode() : Node("mobilenet_publisher_node") {
        encoded_pub_ = this->create_publisher<sensor_msgs::msg::CompressedImage>("encoded_video", 10);

        pipeline = createPipeline();
        device = std::make_shared<dai::Device>(pipeline);

        encodedQueue = device->getOutputQueue("encoded_video", 30, false);

        timer = this->create_wall_timer(
                std::chrono::milliseconds(200),
                std::bind(&MobileNetPublisherNode::publishEncodedImage, this));
    }

private:
    void publishEncodedImage() {
        auto frame = encodedQueue->get<dai::ImgFrame>();

        sensor_msgs::msg::CompressedImage::SharedPtr msg = std::make_shared<sensor_msgs::msg::CompressedImage>();

        msg->header.stamp = this->now();
        msg->format = "jpeg";
        msg->data = frame->getData();

        encoded_pub_->publish(*msg);
    }

    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr encoded_pub_;
    rclcpp::TimerBase::SharedPtr timer;
    std::shared_ptr<dai::Device> device;
    std::shared_ptr<dai::DataOutputQueue> encodedQueue;
    dai::Pipeline pipeline;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MobileNetPublisherNode>());
    rclcpp::shutdown();

    return 0;
}