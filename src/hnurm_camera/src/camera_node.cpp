#include "hnurm_camera/camera_node.hpp"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.hpp>

using namespace std::chrono_literals;

namespace hnurm
{
void CameraNode::run()
{
    RCLCPP_INFO(logger_, "CameraNode is running");
    std::string camera_img_topic = this->declare_parameter("camera_img_topic", "image");
    cam_                         = std::make_shared<HKcam>(shared_from_this());

    // using best effort
    pub_img_  = image_transport::create_publisher(this, camera_img_topic, rmw_qos_profile_sensor_data);
    cam_info_ = std::make_unique<camera_info_manager::CameraInfoManager>(this, "camera");

    capture_thread_ = std::thread([this]() {
        while(rclcpp::ok())
        {
            timer_callback();
        }
    });
}

void CameraNode::timer_callback()
{
    std::vector<uint8_t> img;
    static long long     img_cnt = 0;
    static auto          prev    = this->now();
    if(cam_->GetFrame(img))
    {
        img_msg_.data            = img;
        img_msg_.header.stamp    = now();
        img_msg_.header.frame_id = "camera";
        img_msg_.encoding        = sensor_msgs::image_encodings::BGR8;
        img_msg_.height          = cam_->_nHeight;
        img_msg_.width           = cam_->_nWidth;
        img_msg_.is_bigendian    = 0;
        img_msg_.step            = cam_->_nWidth * 3;

        //        pub_->publish(msg);
        pub_img_.publish(img_msg_);

        auto now = this->now();
        RCLCPP_INFO(this->get_logger(), "Capture FPS: %f", 1.0 / (now - prev).seconds());
        prev = now;
    }
}
}