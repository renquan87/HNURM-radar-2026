#include "hnurm_camera/camera_node.hpp"

int main(int argc, char *argv[])
{

    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    options.use_intra_process_comms(true);
    auto node = std::make_shared<hnurm::CameraNode>(options);
    node->run();
    rclcpp::spin(node);
    rclcpp::shutdown();
}