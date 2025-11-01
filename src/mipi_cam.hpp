#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <thread>
#include <string>
#include "common_utils.h"
#include "hbn_isp_api.h"

// 零拷贝消息类型
#include "hbm_img_msgs/msg/hbm_msg1080_p.hpp"

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
    
    // 零拷贝图像发布线程函数
    void publish_image_thread_zero_copy();
    
    // 发布到/hbmem_img话题的辅助函数
    void publish_to_hbmem_topic(const hbn_vnode_image_t& out_img);
    
    // 检查零拷贝环境
    void check_zero_copy_env();
    
    // ROS2图像发布者 (普通方式)
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
    
    // 零拷贝图像发布者
    rclcpp::Publisher<hbm_img_msgs::msg::HbmMsg1080P>::SharedPtr zero_copy_publisher_;
    
    // 视频流上下文
    pipe_contex_t pipe_contex_;
    
    // 配置参数
    int sensor_index_;
    std::string frame_id_;
    std::string io_method_name_;  // "ros" 或 "shared_mem"
    
    // 线程控制
    bool running_;
    std::thread publish_thread_;
};

}  // namespace mipi_cam