#include <cstdio>
#include <rclcpp/rclcpp.hpp>
#include "mipi_cam.hpp"

int main(int argc, char ** argv)
{
  printf("hello world standalone_vin_project package\n");
  rclcpp::init(argc, argv);
  auto node = std::make_shared<mipi_cam::MipiNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
