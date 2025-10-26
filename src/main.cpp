#include <cstdio>
#include <rclcpp/rclcpp.hpp>
#include "mipi_cam.hpp"

int main(int argc, char ** argv)
{
  // 初始化ROS2环境
  rclcpp::init(argc, argv);
  
  printf("Starting standalone_vin_project camera node...\n");
  
  // 创建MipiNode节点
  auto node = std::make_shared<mipi_cam::MipiNode>();
  
  try {
    // 运行ROS2节点
    rclcpp::spin(node);
  } catch (const std::exception& e) {
    printf("Exception caught: %s\n", e.what());
  } catch (...) {
    printf("Unknown exception caught\n");
  }
  
  // 关闭ROS2环境
  rclcpp::shutdown();
  
  printf("standalone_vin_project camera node shutdown\n");
  
  return 0;
}
