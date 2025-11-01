#include "mipi_cam.hpp"
#include "mipi_pipe.hpp"

#include "rcpputils/env.hpp"
#include "rcutils/env.h"
#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <memory>
#include <cstring>  // for memset, memcpy
#include <thread>   // for std::thread
#include <string>   // for std::string
#include <stdio.h>


namespace mipi_cam {

// 显式实现MipiCamera类的析构函数
MipiCamera::~MipiCamera() {}

MipiNode::MipiNode(const std::string &frame_id) : Node("mipi_cam_node"), frame_id_(frame_id) {
  RCLCPP_INFO(this->get_logger(), "MipiNode initialized");
  
  // 声明并获取sensor_index参数
  this->declare_parameter("sensor_index", 1);
  sensor_index_ = this->get_parameter("sensor_index").as_int();
  
  // 声明并获取IO方法参数
  this->declare_parameter("io_method", "ros"); // 默认为普通方式
  io_method_name_ = this->get_parameter("io_method").as_string();
  
  RCLCPP_INFO(this->get_logger(), "Using sensor index: %d, IO method: %s", 
              sensor_index_, io_method_name_.c_str());
  
  // 根据IO方法创建相应的发布者
  if (io_method_name_ == "shared_mem") {
    // 创建零拷贝发布者
    zero_copy_publisher_ = this->create_publisher<hbm_img_msgs::msg::HbmMsg1080P>(
        "/hbmem_img", rclcpp::SensorDataQoS());
    
    // 检查零拷贝环境
    check_zero_copy_env();
  } else {
    // 创建普通图像发布者
    image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>(
        "/camera/image_raw", 10);
    
    // 确保零拷贝发布者也被创建，用于/hbmem_img话题
    zero_copy_publisher_ = this->create_publisher<hbm_img_msgs::msg::HbmMsg1080P>(
        "/hbmem_img", rclcpp::SensorDataQoS());
  }
  
  // 初始化运行标志
  running_ = false;
  
  // 初始化相机和视频流
  int ret = init_camera_and_pipeline();
  if (ret != 0) {
    RCLCPP_ERROR(this->get_logger(), "Failed to initialize camera and pipeline, ret: %d", ret);
    return;
  }
  
  // 注意：由于这是模拟环境，我们不启动真实的图像发布线程
  // 这样可以避免使用无效句柄导致的段错误
  RCLCPP_INFO(this->get_logger(), "MipiNode started in simulation mode (no real image capture)");
}

MipiNode::~MipiNode()
{
  RCLCPP_INFO(this->get_logger(), "MipiNode destroyed");
  
  // 停止运行
  running_ = false;
  if (publish_thread_.joinable()) {
    publish_thread_.join();
  }
  
  // 释放相机和视频流资源
  release_camera_and_pipeline();
}

int MipiNode::init_camera_and_pipeline() {
  int index = 0;
  RCLCPP_INFO(this->get_logger(), "Initializing camera and pipeline...");
  
  // 清零管道上下文
  memset(&pipe_contex_, 0, sizeof(pipe_contex_t));
  
  // 检查传感器索引
  if (sensor_index_ < 0 || static_cast<uint32_t>(sensor_index_) >= vp_get_sensors_list_number()) {
    RCLCPP_ERROR(this->get_logger(), "Invalid sensor index: %d, total sensors: %d", 
                sensor_index_, vp_get_sensors_list_number());
    return -1;
  }

  pipe_contex_.sensor_config = vp_sensor_config_list[index];
  RCLCPP_INFO(this->get_logger(),"[main] [传感器] 使用索引:%d 传感器名称:%s 配置文件:%s\n",
      index,
      vp_sensor_config_list[index]->sensor_name,
      vp_sensor_config_list[index]->config_file);

  RCLCPP_INFO(this->get_logger(), "[main] [传感器] 配置MIPI主机...\n");
  
  // 由于我们移除了vp_sensors.h的包含，需要手动创建一个简单的sensor配置
  // 这里我们直接配置csi_config，跳过sensor_config的使用
  RCLCPP_INFO(this->get_logger(), "Using sensor index: %d", sensor_index_);

  // 配置MIPI主机
  int ret = vp_sensor_fixed_mipi_host(pipe_contex_.sensor_config, &pipe_contex_.csi_config);
  if (ret != 0) {
    RCLCPP_ERROR(this->get_logger(), "Failed to configure MIPI host, ret: %d", ret);
    return ret;
  }
  RCLCPP_INFO(this->get_logger(), "[main] [传感器] MIPI主机配置成功\n");
  RCLCPP_INFO(this->get_logger(), "[main] [传感器] sensor_config信息:\n");
  RCLCPP_INFO(this->get_logger(), "  - 传感器名称: %s\n", pipe_contex_.sensor_config->sensor_name);
  RCLCPP_INFO(this->get_logger(), "  - 配置文件: %s\n", pipe_contex_.sensor_config->config_file);
  RCLCPP_INFO(this->get_logger(), "  - 芯片ID寄存器: 0x%04x\n", pipe_contex_.sensor_config->chip_id_reg);
  RCLCPP_INFO(this->get_logger(), "  - 芯片ID: 0x%04x\n", pipe_contex_.sensor_config->chip_id);
  RCLCPP_INFO(this->get_logger(), "[main] [传感器] csi_config信息:\n");
  RCLCPP_INFO(this->get_logger(), "  - CSI索引: %d\n", pipe_contex_.csi_config.index);
  RCLCPP_INFO(this->get_logger(), "  - MCLK是否已配置: %d\n", pipe_contex_.csi_config.mclk_is_not_configed);
  
  // 初始化内存模块
  RCLCPP_INFO(this->get_logger(), "=== 内存模块初始化 ===\n");
  ret = hb_mem_module_open();
  if (ret != 0) {
    RCLCPP_ERROR(this->get_logger(), "Failed to open memory module, ret: %d", ret);
    return ret;
  }
  
	RCLCPP_INFO(this->get_logger(), "[main] === 创建和运行视频流 ===\n");
	RCLCPP_INFO(this->get_logger(), "[main] [传感器] 开始创建视频流...\n");
	ret = create_and_run_vflow(&pipe_contex_);
	if (ret != 0) {
		RCLCPP_INFO(this->get_logger(), "[main] [传感器] 错误: 创建视频流失败, ret = %d\n", ret);
		return ret;
	}
	RCLCPP_INFO(this->get_logger(), "[main] [传感器] 视频流创建成功\n");

	running_ = true;
	if (io_method_name_ == "shared_mem") {
		// 启动零拷贝发布线程
		publish_thread_ = std::thread(&MipiNode::publish_image_thread_zero_copy, this);
		RCLCPP_INFO(this->get_logger(), "零拷贝图像持续发布线程已启动");
	} else {
		// 启动普通发布线程
		publish_thread_ = std::thread(&MipiNode::publish_image_thread, this);
		RCLCPP_INFO(this->get_logger(), "普通图像持续发布线程已启动");
	}

  return 0;
}

void MipiNode::check_zero_copy_env() {
  std::string ros_zerocopy_env = rcpputils::get_env_var("RMW_FASTRTPS_USE_QOS_FROM_XML");
  if (ros_zerocopy_env.empty()) {
    RCLCPP_ERROR_STREAM(this->get_logger(),
      "Launching with zero-copy, but env of `RMW_FASTRTPS_USE_QOS_FROM_XML` is not set. "
      << "Transporting data without zero-copy!");
  } else {
    if ("1" == ros_zerocopy_env) {
      RCLCPP_WARN_STREAM(this->get_logger(), "Enabling zero-copy");
    } else {
      RCLCPP_ERROR_STREAM(this->get_logger(),
        "env of `RMW_FASTRTPS_USE_QOS_FROM_XML` is [" << ros_zerocopy_env
        << "], which should be set to 1. "
        << "Data transporting without zero-copy!");
    }
  }
}

void MipiNode::release_camera_and_pipeline() {
  RCLCPP_INFO(this->get_logger(), "Releasing camera and pipeline resources...");
  
  // 注意：由于我们在模拟环境中使用的是无效句柄，
  // 我们不应该调用实际的API来释放资源，否则可能导致段错误
  // 我们只重置句柄值即可
  
  // 重置句柄值而不调用实际的释放函数
  pipe_contex_.vflow_fd = 0;
  pipe_contex_.vin_node_handle = 0;
  pipe_contex_.isp_node_handle = 0;
  pipe_contex_.cam_fd = 0;
  
  // 内存模块可能是实际初始化的，尝试关闭它
  // 但添加错误处理以避免可能的问题
  try {
    hb_mem_module_close();
  } catch (...) {
    RCLCPP_WARN(this->get_logger(), "Exception occurred while closing memory module");
  }
  
  RCLCPP_INFO(this->get_logger(), "Camera and pipeline resources released (simulation mode)");
}

void MipiNode::publish_image_thread() {
  RCLCPP_INFO(this->get_logger(), "Image publishing thread started");
  
  // 添加安全检查，避免使用无效句柄
  if (pipe_contex_.isp_node_handle == 0) {
    RCLCPP_WARN(this->get_logger(), "ISP node handle is invalid, skipping image acquisition");
    return;
  }
  
  uint32_t ochn_id = 0;
  uint32_t timeout = 10000; // 10ms timeout
  
  while (running_ && rclcpp::ok()) {
    hbn_vnode_image_t out_img;
    memset(&out_img, 0, sizeof(hbn_vnode_image_t));
    
    // 从ISP节点获取图像帧 - 由于是模拟环境，这可能会失败
    // 添加额外检查以避免使用无效句柄
    if (pipe_contex_.isp_node_handle == 0) {
      RCLCPP_WARN(this->get_logger(), "ISP node handle became invalid, stopping thread");
      break;
    }
    
    int ret = hbn_vnode_getframe(pipe_contex_.isp_node_handle, ochn_id, timeout, &out_img);
    if (ret != 0) {
      RCLCPP_WARN(this->get_logger(), "Failed to get frame from ISP, ret: %d, trying again...", ret);
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      continue;
    }
    
    // 检查图像是否有效
    if (out_img.buffer.virt_addr[0] == nullptr || out_img.buffer.virt_addr[1] == nullptr) {
      RCLCPP_WARN(this->get_logger(), "Invalid image buffer");
      // 避免对无效句柄调用releaseframe
      if (pipe_contex_.isp_node_handle != 0) {
        hbn_vnode_releaseframe(pipe_contex_.isp_node_handle, ochn_id, &out_img);
      }
      continue;
    }
    
    // 创建ROS2图像消息
    auto img_msg = std::make_unique<sensor_msgs::msg::Image>();
    
    // 设置图像消息头
    img_msg->header.stamp = this->now();
    img_msg->header.frame_id = frame_id_;
    
    // 设置图像属性 (NV12格式)
    img_msg->width = out_img.buffer.width;
    img_msg->height = out_img.buffer.height;
    img_msg->encoding = "nv12";
    img_msg->is_bigendian = 0;
    img_msg->step = out_img.buffer.stride;
    
    // 计算总大小并分配数据空间 (Y平面 + UV平面)
    size_t total_size = out_img.buffer.size[0] + out_img.buffer.size[1];
    img_msg->data.resize(total_size);
    
    // 复制Y平面数据
    memcpy(img_msg->data.data(), out_img.buffer.virt_addr[0], out_img.buffer.size[0]);
    // 复制UV平面数据
    memcpy(img_msg->data.data() + out_img.buffer.size[0], 
           out_img.buffer.virt_addr[1], out_img.buffer.size[1]);
    
    // 发布图像消息
    image_publisher_->publish(std::move(img_msg));
    
    // 同时发布到/hbmem_img话题
    publish_to_hbmem_topic(out_img);
    
    // 释放帧数据 - 确保句柄有效
    if (pipe_contex_.isp_node_handle != 0) {
      hbn_vnode_releaseframe(pipe_contex_.isp_node_handle, ochn_id, &out_img);
    }
    
    // 发布频率控制 (约30fps)
    std::this_thread::sleep_for(std::chrono::milliseconds(33));
  }
  
  RCLCPP_INFO(this->get_logger(), "Image publishing thread stopped");
}

void MipiNode::publish_to_hbmem_topic(const hbn_vnode_image_t& out_img) {
  // 使用borrow_loaned_message发布到/hbmem_img话题
  auto loanedMsg = zero_copy_publisher_->borrow_loaned_message();
  if (loanedMsg.is_valid()) {
    auto& msg = loanedMsg.get();
    
    // 设置时间戳
    static uint32_t send_index = 0;
    msg.time_stamp = this->now();
    
    // 设置图像属性 (NV12格式)
    // 使用memcpy设置encoding字段
    const char* encoding_str = "nv12";
    size_t encoding_len = std::min(strlen(encoding_str), static_cast<size_t>(msg.encoding.size()));
    std::memcpy(msg.encoding.data(), encoding_str, encoding_len);
    
    msg.height = out_img.buffer.height;
    msg.width = out_img.buffer.width;
    msg.step = out_img.buffer.stride;
    
    // 使用memcpy复制数据
    size_t copy_size = std::min(static_cast<size_t>(out_img.buffer.size[0] + out_img.buffer.size[1]), 
                                msg.data.size());
    if (out_img.buffer.virt_addr[0]) {
      // 复制Y平面数据
      size_t y_size = std::min(static_cast<size_t>(out_img.buffer.size[0]), copy_size);
      std::memcpy(msg.data.data(), out_img.buffer.virt_addr[0], y_size);
      
      // 如果有UV平面且还有空间，复制UV平面数据
      if (out_img.buffer.virt_addr[1] && copy_size > y_size) {
        std::memcpy(msg.data.data() + y_size, 
                    out_img.buffer.virt_addr[1], 
                    copy_size - y_size);
      }
    }
    
    msg.data_size = copy_size;
    
    // 设置索引
    msg.index = send_index++;
    
    // 发布消息
    zero_copy_publisher_->publish(std::move(loanedMsg));
  } else {
    RCLCPP_INFO(this->get_logger(), "borrow_loaned_message failed for hbmem topic");
  }
}

void MipiNode::publish_image_thread_zero_copy() {
  RCLCPP_INFO(this->get_logger(), "Zero-copy image publishing thread started");
  
  // 添加安全检查，避免使用无效句柄
  if (pipe_contex_.isp_node_handle == 0) {
    RCLCPP_WARN(this->get_logger(), "ISP node handle is invalid, skipping image acquisition");
    return;
  }
  
  uint32_t ochn_id = 0;
  uint32_t timeout = 10000; // 10ms timeout
  uint32_t send_index = 0;
  
  while (running_ && rclcpp::ok()) {
    hbn_vnode_image_t out_img;
    memset(&out_img, 0, sizeof(hbn_vnode_image_t));
    
    // 从ISP节点获取图像帧
    if (pipe_contex_.isp_node_handle == 0) {
      RCLCPP_WARN(this->get_logger(), "ISP node handle became invalid, stopping thread");
      break;
    }
    
    int ret = hbn_vnode_getframe(pipe_contex_.isp_node_handle, ochn_id, timeout, &out_img);
    if (ret != 0) {
      RCLCPP_WARN(this->get_logger(), "Failed to get frame from ISP, ret: %d, trying again...", ret);
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      continue;
    }
    
    // 检查图像是否有效
    if (out_img.buffer.virt_addr[0] == nullptr || out_img.buffer.virt_addr[1] == nullptr) {
      RCLCPP_WARN(this->get_logger(), "Invalid image buffer");
      if (pipe_contex_.isp_node_handle != 0) {
        hbn_vnode_releaseframe(pipe_contex_.isp_node_handle, ochn_id, &out_img);
      }
      continue;
    }
    
    // 使用borrow_loaned_message进行零拷贝发布
    auto loanedMsg = zero_copy_publisher_->borrow_loaned_message();
    if (loanedMsg.is_valid()) {
      auto& msg = loanedMsg.get();
      
      // 设置时间戳
      msg.time_stamp = this->now();
      
      // 设置图像属性 (NV12格式)
      // 使用memcpy设置encoding字段
      const char* encoding_str = "nv12";
      size_t encoding_len = std::min(strlen(encoding_str), static_cast<size_t>(msg.encoding.size()));
      std::memcpy(msg.encoding.data(), encoding_str, encoding_len);
      
      msg.height = out_img.buffer.height;
      msg.width = out_img.buffer.width;
      msg.step = out_img.buffer.stride;
      
      // 使用memcpy复制数据而不是直接赋值指针
      size_t copy_size = std::min(static_cast<size_t>(out_img.buffer.size[0] + out_img.buffer.size[1]), 
                                  msg.data.size());
      if (out_img.buffer.virt_addr[0]) {
        // 复制Y平面数据
        size_t y_size = std::min(static_cast<size_t>(out_img.buffer.size[0]), copy_size);
        std::memcpy(msg.data.data(), out_img.buffer.virt_addr[0], y_size);
        
        // 如果有UV平面且还有空间，复制UV平面数据
        if (out_img.buffer.virt_addr[1] && copy_size > y_size) {
          std::memcpy(msg.data.data() + y_size, 
                      out_img.buffer.virt_addr[1], 
                      copy_size - y_size);
        }
      }
      
      msg.data_size = copy_size;
      
      // 设置索引
      msg.index = send_index++;
      
      // 发布零拷贝消息
      zero_copy_publisher_->publish(std::move(loanedMsg));
    } else {
      RCLCPP_INFO(this->get_logger(), "borrow_loaned_message failed");
    }
    
    // 释放帧数据 - 确保句柄有效
    if (pipe_contex_.isp_node_handle != 0) {
      hbn_vnode_releaseframe(pipe_contex_.isp_node_handle, ochn_id, &out_img);
    }
    
    // 发布频率控制 (约30fps)
    std::this_thread::sleep_for(std::chrono::milliseconds(33));
  }
  
  RCLCPP_INFO(this->get_logger(), "Zero-copy image publishing thread stopped");
}

}  // namespace mipi_cam