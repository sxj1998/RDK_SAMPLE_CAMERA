#pragma once

// 正确包含C函数的头文件
extern "C" {
    #include "common_utils.h"
    #include "hbn_isp_api.h"
    #include "vp_sensors.h"
}
// 视频管道相关函数声明
int create_camera_node(pipe_contex_t *pipe_contex);
int create_vin_node(pipe_contex_t *pipe_contex);
int create_isp_node(pipe_contex_t *pipe_contex);
int create_and_run_vflow(pipe_contex_t *pipe_contex);
void vin_dump_func(hbn_vnode_handle_t vin_node_handle);
void isp_dump_func(hbn_vnode_handle_t isp_node_handle);

// 全局变量声明（如果需要）
extern int settle;
extern uint32_t sensor_mode; // 1: NORMAL_M; 2: DOL2_M; 6: SLAVE_M
