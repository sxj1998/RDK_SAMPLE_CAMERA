#ifndef _MIPI_CAM_HPP_
#define _MIPI_CAM_HPP_


#include <string>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <thread>
#include "common_utils.h"

namespace mipi_cam {

class MipiCamera {
public:
  virtual ~MipiCamera() = default;
};

class MipiNode : public rclcpp::Node
{
public:
  explicit MipiNode(const std::string &frame_id = "camera_frame");
  virtual ~MipiNode();

private:
  // 初始化相机和视频流
  int init_camera_and_pipeline();
  // 释放相机和视频流资源
  void release_camera_and_pipeline();
  // 图像发布线程函数
  void publish_image_thread();
  
  // ROS2图像发布者
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
  
  // 相机和视频流相关变量
  pipe_contex_t pipe_contex_;
  int sensor_index_;
  std::string frame_id_;
  bool running_;
  std::thread publish_thread_;
};


}

#endif