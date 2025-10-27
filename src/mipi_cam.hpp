#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <thread>
#include "common_utils.h"
#include "hbn_isp_api.h"

namespace mipi_cam {

class MipiCamera {
public:
    MipiCamera() = default;
    ~MipiCamera(); // 声明析构函数
};

class MipiNode : public rclcpp::Node {
public:
    explicit MipiNode(const std::string &frame_id = "camera_frame");
    ~MipiNode();

private:
    // 初始化相机和视频流管道
    int init_camera_and_pipeline();
    
    // 释放相机和视频流资源
    void release_camera_and_pipeline();
    
    // 图像发布线程函数
    void publish_image_thread();
    
    // ROS2图像发布者
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
    
    // 视频流上下文
    pipe_contex_t pipe_contex_;
    
    // 配置参数
    int sensor_index_;
    std::string frame_id_;
    
    // 线程控制
    bool running_;
    std::thread publish_thread_;
};

}  // namespace mipi_cam