#include <cstdio>
#include <iostream>


#include "camera_info_manager/camera_info_manager.hpp"
#include "depthai_bridge/BridgePublisher.hpp"
#include "depthai_bridge/ImageConverter.hpp"
#include "depthai_bridge/ImgDetectionConverter.hpp"
#include "rclcpp/executors.hpp"
#include "rclcpp/node.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "vision_msgs/msg/detection2_d_array.hpp"

// Includes common necessary includes for development using depthai library
#include "depthai/device/DataQueue.hpp"
#include "depthai/device/Device.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/node/ColorCamera.hpp"
#include "depthai/pipeline/node/MonoCamera.hpp"
#include <depthai/pipeline/node/VideoEncoder.hpp>
#include "depthai/pipeline/node/DetectionNetwork.hpp"
#include "depthai/pipeline/node/XLinkOut.hpp"

dai::Pipeline createPipeline() {
    dai::Pipeline pipeline;

    auto monoCam = pipeline.create<dai::node::MonoCamera>();
    auto xlinkOut = pipeline.create<dai::node::XLinkOut>();

    xlinkOut->setStreamName("jpeg");

    monoCam->setResolution(dai::MonoCameraProperties::SensorResolution::THE_400_P);
    monoCam->setFps(1);
    monoCam->setRawOutputPacked(true);

    auto videoEnc = pipeline.create<dai::node::VideoEncoder>();
    videoEnc->setDefaultProfilePreset(monoCam->getFps(), dai::VideoEncoderProperties::Profile::MJPEG);
    videoEnc->setQuality(25);

    monoCam->out.link(videoEnc->input);
    videoEnc->bitstream.link(xlinkOut->input);

    return pipeline;
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("mono_camera_node");

    std::string tfPrefix, resourceBaseFolder, nnPath;
    std::string cameraParamUri = "package://depthai_examples/params/camera";

    int bad_params = 0;

    node->declare_parameter("tf_prefix", "oak");
    node->declare_parameter("camera_param_uri", cameraParamUri);
    node->declare_parameter("resourceBaseFolder", "");

    node->get_parameter("tf_prefix", tfPrefix);
    node->get_parameter("camera_param_uri", cameraParamUri);
    node->get_parameter("resourceBaseFolder", resourceBaseFolder);

    if(resourceBaseFolder.empty()) {
        throw std::runtime_error("Send the path to the resource folder containing NNBlob in 'resourceBaseFolder'");
    }
    // Uses the path from param if passed or else uses from BLOB_PATH from CMAKE

    bool found_flag = false;
    while(!found_flag) {
        try {
            dai::Pipeline pipeline = createPipeline();
            dai::Device device(pipeline);

            std::shared_ptr<dai::DataOutputQueue> monoQueue = device.getOutputQueue("mono", 30, false);

            std::string mono_uri = cameraParamUri + "/" + "mono.yaml";

            dai::rosBridge::ImageConverter monoConverter(tfPrefix + "_mono_camera_optical_frame", false);
            dai::rosBridge::BridgePublisher<sensor_msgs::msg::Image, dai::ImgFrame> monoPublish(monoQueue,
                                                                                                node,
                                                                                                std::string(
                                                                                                        "mono/image"),
                                                                                                std::bind(
                                                                                                        &dai::rosBridge::ImageConverter::toRosMsg,
                                                                                                        &monoConverter,  // since the converter has the same frame name
                                                                                                        // and image type is also same we can reuse it
                                                                                                        std::placeholders::_1,
                                                                                                        std::placeholders::_2),
                                                                                                30,
                                                                                                mono_uri,
                                                                                                "mono");

            monoPublish.addPublisherCallback();  // addPublisherCallback works only when the dataqueue is non blocking.

            rclcpp::spin(node);

            found_flag = true;
        }
        catch (std::runtime_error e) {
            RCLCPP_WARN(node->get_logger(), "No device found, retrying search...");
        }
    }

    rclcpp::shutdown();

    return 0;
}


