#include "rclcpp/rclcpp.hpp"

#include "mipi_cam.hpp"

namespace mipi_cam {

// 显式实现MipiCamera类的析构函数
MipiCamera::~MipiCamera() {}

// 实现MipiNode类的构造函数
MipiNode::MipiNode() : Node("mipi_cam_node") 
{
  RCLCPP_INFO(this->get_logger(), "hello world standalone_vin_project package");
}

// 实现MipiNode类的析构函数
MipiNode::~MipiNode()
{
  RCLCPP_INFO(this->get_logger(), "MipiNode destroyed");
}

}