#pragma once
#include "hnurm_camera/Camera.h"
#include <rclcpp/rclcpp.hpp>

#include <camera_info_manager/camera_info_manager.hpp>
#include <image_transport/image_transport.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace hnurm
{
class CameraNode : public rclcpp::Node
{
public:
    explicit CameraNode(const rclcpp::NodeOptions &options)
        : Node("camera_node", options),
          cam_(nullptr),
          logger_(get_logger()) {};

    CameraNode(const CameraNode &)            = delete;
    CameraNode &operator=(const CameraNode &) = delete;
    CameraNode(CameraNode &&)                 = delete;
    CameraNode &operator=(CameraNode &&)      = delete;
    ~CameraNode() override = default;

    void run();

private:
    image_transport::Publisher                              pub_img_;
    std::unique_ptr<camera_info_manager::CameraInfoManager> cam_info_;
    // msgs
    sensor_msgs::msg::Image      img_msg_;
    sensor_msgs::msg::CameraInfo cam_info_msg_;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Logger               logger_;

    std::shared_ptr<HKcam> cam_;
    std::thread            capture_thread_;

private:
    void timer_callback();

protected:
    std::shared_ptr<CameraNode> shared_from_this()
    {
        return std::static_pointer_cast<CameraNode>(Node::shared_from_this());
    }
};
}
