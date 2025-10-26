#ifndef _MIPI_CAM_HPP_
#define _MIPI_CAM_HPP_


#include <string>
#include <vector>
#include <rclcpp/rclcpp.hpp>

namespace mipi_cam {

class MipiCamera {
public:
  virtual ~MipiCamera() = default;
};

class MipiNode : public rclcpp::Node
{
public:
  MipiNode();
  virtual ~MipiNode();

};


}

#endif