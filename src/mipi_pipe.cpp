#include "mipi_pipe.hpp"
#include <stdio.h>

// 全局变量定义
int settle = -1;
uint32_t sensor_mode = 0; // 1: NORMAL_M; 2: DOL2_M; 6: SLAVE_M

int create_camera_node(pipe_contex_t *pipe_contex) {
    camera_config_t *camera_config = NULL;
    vp_sensor_config_t *sensor_config = NULL;
    int32_t ret = 0;

    printf("[create_camera_node] 创建相机节点...\n");
    
    sensor_config = pipe_contex->sensor_config;
    camera_config = sensor_config->camera_config;
    
    /* Debug settle */
    if (settle >= 0 && settle <= 127) {
        camera_config->mipi_cfg->rx_attr.settle = settle;
        printf("[create_camera_node] 设置 settle 时间: %d\n", settle);
    }
    if (sensor_mode >= NORMAL_M && sensor_mode < INVALID_MOD) {
        camera_config->sensor_mode = sensor_mode;
        sensor_config->vin_node_attr->lpwm_attr.enable = 1;
        printf("[create_camera_node] 设置传感器模式: %d\n", sensor_mode);
    }
    
    printf("[create_camera_node] 创建相机...\n");
    ret = hbn_camera_create(camera_config, &pipe_contex->cam_fd);
    if (ret != 0) {
        printf("[create_camera_node] 错误: 创建相机失败, ret = %d\n", ret);
        return ret;
    }
    printf("[create_camera_node] 相机创建成功\n");

    return 0;
}

int create_vin_node(pipe_contex_t *pipe_contex) {
    vp_sensor_config_t *sensor_config = NULL;
    vin_node_attr_t *vin_node_attr = NULL;
    vin_ichn_attr_t *vin_ichn_attr = NULL;
    vin_ochn_attr_t *vin_ochn_attr = NULL;
    hbn_vnode_handle_t *vin_node_handle = NULL;
    vin_attr_ex_t vin_attr_ex;
    hbn_buf_alloc_attr_t alloc_attr = {0};
    uint32_t hw_id = 0;
    int32_t ret = 0;
    uint32_t ichn_id = 0;
    uint32_t ochn_id = 0;
    uint64_t vin_attr_ex_mask = 0;

    printf("[create_vin_node] 创建VIN节点...\n");
    
    sensor_config = pipe_contex->sensor_config;
    vin_node_attr = sensor_config->vin_node_attr;
    vin_ichn_attr = sensor_config->vin_ichn_attr;
    vin_ochn_attr = sensor_config->vin_ochn_attr;
    hw_id = vin_node_attr->cim_attr.mipi_rx;
    vin_node_handle = &pipe_contex->vin_node_handle;

    if(pipe_contex->csi_config.mclk_is_not_configed){
        //设备树中没有配置mclk：使用外部晶振
        printf("[create_vin_node] CSI%d 忽略MCLK外部属性，因为未配置MCLK\n", pipe_contex->csi_config.index);
    }else{
        vin_attr_ex.vin_attr_ex_mask = sensor_config->vin_attr_ex->vin_attr_ex_mask;
        vin_attr_ex.mclk_ex_attr.mclk_freq = sensor_config->vin_attr_ex->mclk_ex_attr.mclk_freq;
        vin_attr_ex_mask = vin_attr_ex.vin_attr_ex_mask;
        printf("[create_vin_node] 设置MCLK频率: %d Hz\n", vin_attr_ex.mclk_ex_attr.mclk_freq);
    }
    
    printf("[create_vin_node] 打开VIN节点...\n");
    ret = hbn_vnode_open(HB_VIN, hw_id, AUTO_ALLOC_ID, vin_node_handle);
    if (ret != 0) {
        printf("[create_vin_node] 错误: 打开VIN节点失败, ret = %d\n", ret);
        return ret;
    }
    
    // 设置基本属性
    printf("[create_vin_node] 设置VIN节点属性...\n");
    ret = hbn_vnode_set_attr(*vin_node_handle, vin_node_attr);
    if (ret != 0) {
        printf("[create_vin_node] 错误: 设置VIN节点属性失败, ret = %d\n", ret);
        return ret;
    }
    
    // 设置输入通道的属性
    printf("[create_vin_node] 设置输入通道属性...\n");
    ret = hbn_vnode_set_ichn_attr(*vin_node_handle, ichn_id, vin_ichn_attr);
    if (ret != 0) {
        printf("[create_vin_node] 错误: 设置输入通道属性失败, ret = %d\n", ret);
        return ret;
    }
    
    // 设置输出通道的属性
    // 使能DDR输出
    printf("[create_vin_node] 设置输出通道属性...\n");
    vin_ochn_attr->ddr_en = 1;
    ret = hbn_vnode_set_ochn_attr(*vin_node_handle, ochn_id, vin_ochn_attr);
    if (ret != 0) {
        printf("[create_vin_node] 错误: 设置输出通道属性失败, ret = %d\n", ret);
        return ret;
    }
    
    if (vin_attr_ex_mask) {
        printf("[create_vin_node] 设置VIN扩展属性，掩码: 0x%lx\n", vin_attr_ex_mask);
        for (uint8_t i = 0; i < VIN_ATTR_EX_INVALID; i ++) {
            if ((vin_attr_ex_mask & (1 << i)) == 0)
                continue;

            vin_attr_ex.ex_attr_type = static_cast<vin_attr_ex_type_e>(i);
            /*we need to set hbn_vnode_set_attr_ex in a loop*/
            ret = hbn_vnode_set_attr_ex(*vin_node_handle, &vin_attr_ex);
            if (ret != 0) {
                printf("[create_vin_node] 错误: 设置VIN扩展属性失败, 类型: %d, ret = %d\n", i, ret);
                return ret;
            }
        }
    }
    
    printf("[create_vin_node] 设置输出通道缓冲区属性...\n");
    alloc_attr.buffers_num = 3;
    alloc_attr.is_contig = 1;
    alloc_attr.flags = HB_MEM_USAGE_CPU_READ_OFTEN
                        | HB_MEM_USAGE_CPU_WRITE_OFTEN
                        | HB_MEM_USAGE_CACHED;
    ret = hbn_vnode_set_ochn_buf_attr(*vin_node_handle, ochn_id, &alloc_attr);
    if (ret != 0) {
        printf("[create_vin_node] 警告: 设置输出通道缓冲区属性失败, ret = %d\n", ret);
    }

    printf("[create_vin_node] VIN节点创建成功\n");
    return 0;
}

int create_isp_node(pipe_contex_t *pipe_contex) {
    vp_sensor_config_t *sensor_config = NULL;
    isp_attr_t      *isp_attr = NULL;
    isp_ichn_attr_t *isp_ichn_attr = NULL;
    isp_ochn_attr_t *isp_ochn_attr = NULL;
    hbn_vnode_handle_t *isp_node_handle = NULL;
    hbn_buf_alloc_attr_t alloc_attr = {0};
    uint32_t ichn_id = 0;
    uint32_t ochn_id = 0;
    int ret = 0;

    printf("[create_isp_node] 创建ISP节点...\n");

    sensor_config = pipe_contex->sensor_config;
    isp_attr = sensor_config->isp_attr;
    isp_ichn_attr = sensor_config->isp_ichn_attr;
    isp_ochn_attr = sensor_config->isp_ochn_attr;
    isp_node_handle = &pipe_contex->isp_node_handle;

    // 打开ISP节点
    printf("[create_isp_node] 打开ISP节点...\n");
    ret = hbn_vnode_open(HB_ISP, 0, AUTO_ALLOC_ID, isp_node_handle);
    if (ret != 0) {
        printf("[create_isp_node] 错误: 打开ISP节点失败, ret = %d\n", ret);
        return ret;
    }

    // 设置ISP节点属性
    printf("[create_isp_node] 设置ISP节点属性...\n");
    ret = hbn_vnode_set_attr(*isp_node_handle, isp_attr);
    if (ret != 0) {
        printf("[create_isp_node] 错误: 设置ISP节点属性失败, ret = %d\n", ret);
        return ret;
    }

    // 设置ISP输出通道属性
    printf("[create_isp_node] 设置ISP输出通道属性...\n");
    ret = hbn_vnode_set_ochn_attr(*isp_node_handle, ochn_id, isp_ochn_attr);
    if (ret != 0) {
        printf("[create_isp_node] 错误: 设置ISP输出通道属性失败, ret = %d\n", ret);
        return ret;
    }

    // 设置ISP输入通道属性
    printf("[create_isp_node] 设置ISP输入通道属性...\n");
    ret = hbn_vnode_set_ichn_attr(*isp_node_handle, ichn_id, isp_ichn_attr);
    if (ret != 0) {
        printf("[create_isp_node] 错误: 设置ISP输入通道属性失败, ret = %d\n", ret);
        return ret;
    }

    // 设置输出缓冲区属性
    printf("[create_isp_node] 设置ISP输出缓冲区属性...\n");
    alloc_attr.buffers_num = 3;
    alloc_attr.is_contig = 1;
    alloc_attr.flags = HB_MEM_USAGE_CPU_READ_OFTEN
                        | HB_MEM_USAGE_CPU_WRITE_OFTEN
                        | HB_MEM_USAGE_CACHED;
    ret = hbn_vnode_set_ochn_buf_attr(*isp_node_handle, ochn_id, &alloc_attr);
    if (ret != 0) {
        printf("[create_isp_node] 错误: 设置ISP输出缓冲区属性失败, ret = %d\n", ret);
        return ret;
    }

    printf("[create_isp_node] ISP节点创建成功, 句柄: %d\n", (int)*isp_node_handle);
    return 0;
}

int create_and_run_vflow(pipe_contex_t *pipe_contex) {
    int32_t ret = 0;

    printf("[create_and_run_vflow] 创建和运行视频流管道...\n");

    // 创建pipeline中的每个node
    printf("[create_and_run_vflow] 创建相机节点...\n");
    ret = create_camera_node(pipe_contex);
    if (ret != 0) {
        printf("[create_and_run_vflow] 错误: 创建相机节点失败\n");
        return ret;
    }
    
    printf("[create_and_run_vflow] 创建VIN节点...\n");
    ret = create_vin_node(pipe_contex);
    if (ret != 0) {
        printf("[create_and_run_vflow] 错误: 创建VIN节点失败\n");
        return ret;
    }

    printf("[create_and_run_vflow] 创建ISP节点...\n");
    ret = create_isp_node(pipe_contex);
    if (ret != 0) {
        printf("[create_and_run_vflow] 错误: 创建ISP节点失败\n");
        return ret;
    }

    // 创建HBN flow
    printf("[create_and_run_vflow] 创建视频流...\n");
    ret = hbn_vflow_create(&pipe_contex->vflow_fd);
    if (ret != 0) {
        printf("[create_and_run_vflow] 错误: 创建视频流失败, ret = %d\n", ret);
        return ret;
    }
    
    printf("[create_and_run_vflow] 添加VIN节点到视频流...\n");
    ret = hbn_vflow_add_vnode(pipe_contex->vflow_fd, pipe_contex->vin_node_handle);
    if (ret != 0) {
        printf("[create_and_run_vflow] 错误: 添加VIN节点到视频流失败, ret = %d\n", ret);
        return ret;
    }

    printf("[create_and_run_vflow] 添加ISP节点到视频流...\n");
    ret = hbn_vflow_add_vnode(pipe_contex->vflow_fd, pipe_contex->isp_node_handle);
    if (ret != 0) {
        printf("[create_and_run_vflow] 错误: 添加ISP节点到视频流失败, ret = %d\n", ret);
        return ret;
    }

    printf("[create_and_run_vflow] 绑定VIN和ISP节点...\n");
    ret = hbn_vflow_bind_vnode(pipe_contex->vflow_fd,
                            pipe_contex->vin_node_handle,
                            1, // online
                            pipe_contex->isp_node_handle,
                            0);
    if (ret != 0) {
        printf("[create_and_run_vflow] 错误: 绑定VIN和ISP节点失败, ret = %d\n", ret);
        return ret;
    }
    
    printf("[create_and_run_vflow] 将相机附加到VIN节点...\n");
    ret = hbn_camera_attach_to_vin(pipe_contex->cam_fd, pipe_contex->vin_node_handle);
    if (ret != 0) {
        printf("[create_and_run_vflow] 错误: 将相机附加到VIN节点失败, ret = %d\n", ret);
        return ret;
    }
    
    printf("[create_and_run_vflow] 启动视频流...\n");
    ret = hbn_vflow_start(pipe_contex->vflow_fd);
    if (ret != 0) {
        printf("[create_and_run_vflow] 错误: 启动视频流失败, ret = %d\n", ret);
        return ret;
    }

    printf("[create_and_run_vflow] 视频流启动成功\n");
    return 0;
}

void vin_dump_func(hbn_vnode_handle_t vin_node_handle) {
    int ret;
    char dst_file[128];
    uint32_t ochn_id = 0;
    uint32_t timeout = 10000;
    hbn_vnode_image_t out_img;

    printf("[vin_dump_func] 获取帧数据...\n");
    
    // 调用hbn_vnode_getframe获取帧数据
    ret = hbn_vnode_getframe(vin_node_handle, ochn_id, timeout, &out_img);
    if (ret != 0) {
        printf("[vin_dump_func] 错误: 从VIN通道%d获取帧失败(%d)\n", ochn_id, ret);
        return;
    }

    // 将帧数据写入文件
    snprintf(dst_file, sizeof(dst_file),
        "handle_%d_vin_chn%d_%dx%d_stride_%d_frameid_%d_ts_%ld.raw",
        (int)vin_node_handle, ochn_id,
        out_img.buffer.width, out_img.buffer.height, out_img.buffer.stride,
        out_img.info.frame_id, out_img.info.timestamps);
    
    printf("[vin_dump_func] 保存帧数据到文件: %s\n", dst_file);
    printf("[vin_dump_func] 节点句柄 %d VIN 输出 %dx%d(步长:%d), 缓冲区大小: %ld 帧ID: %d, 时间戳: %ld\n",
            (int)vin_node_handle,
            out_img.buffer.width, out_img.buffer.height,
            out_img.buffer.stride,
            out_img.buffer.size[0],
            out_img.info.frame_id,
            out_img.info.timestamps);
            
    dump_image_to_file(dst_file, out_img.buffer.virt_addr[0], out_img.buffer.size[0]);
    
    // 释放帧数据
    printf("[vin_dump_func] 释放帧数据...\n");
    hbn_vnode_releaseframe(vin_node_handle, ochn_id, &out_img);
    printf("[vin_dump_func] 帧数据处理完成\n");
}

void isp_dump_func(hbn_vnode_handle_t isp_node_handle) {
    int ret;
    char dst_file[128];
    uint32_t ochn_id = 0;
    uint32_t timeout = 10000;
    hbn_vnode_image_t out_img;

    printf("[isp_dump_func] 获取ISP帧数据...\n");

    // 调用hbn_vnode_getframe获取帧数据
    ret = hbn_vnode_getframe(isp_node_handle, ochn_id, timeout, &out_img);
    if (ret != 0) {
        printf("[isp_dump_func] 错误: 从ISP通道%d获取帧失败(%d)\n", ochn_id, ret);
        return;
    }

    // 将帧数据写入文件
    snprintf(dst_file, sizeof(dst_file),
        "isp_handle_%d_chn%d_%dx%d_stride_%d_frameid_%d_ts_%ld.yuv",
        (int)isp_node_handle, ochn_id,
        out_img.buffer.width, out_img.buffer.height, out_img.buffer.stride,
        out_img.info.frame_id, out_img.info.timestamps);

    printf("[isp_dump_func] 保存帧数据到文件: %s\n", dst_file);
    printf("[isp_dump_func] ISP节点句柄 %d 输出 %dx%d(步长:%d), 缓冲区大小: %ld + %ld 帧ID: %d, 时间戳: %ld\n",
            (int)isp_node_handle,
            out_img.buffer.width, out_img.buffer.height,
            out_img.buffer.stride,
            out_img.buffer.size[0], out_img.buffer.size[1],
            out_img.info.frame_id,
            out_img.info.timestamps);

    // 写入YUV数据（NV12格式，双平面）
    dump_2plane_yuv_to_file(dst_file,
            out_img.buffer.virt_addr[0],
            out_img.buffer.virt_addr[1],
            out_img.buffer.size[0],
            out_img.buffer.size[1]);

    // 释放帧数据
    printf("[isp_dump_func] 释放帧数据...\n");
    hbn_vnode_releaseframe(isp_node_handle, ochn_id, &out_img);
    printf("[isp_dump_func] 帧数据处理完成\n");
}
