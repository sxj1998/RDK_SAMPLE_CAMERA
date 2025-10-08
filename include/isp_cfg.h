#ifndef __ISP_CFG_H__
#define __ISP_CFG_H__

#include <stdint.h>

// ISP配置相关的枚举和结构体定义

typedef enum {
    FRM_FMT_RAW = 0,
    FRM_FMT_NV12 = 1,
    FRM_FMT_YUV422 = 2,
} frm_fmt_t;

typedef enum {
    ISP_NORMAL_M = 0,
    ISP_HDR_M = 1,
} isp_sensor_mode_t;

typedef struct {
    uint32_t x;
    uint32_t y;
    uint32_t w;
    uint32_t h;
} crop_t;

typedef struct {
    uint32_t input_mode;
    isp_sensor_mode_t sensor_mode;
    crop_t crop;
} isp_attr_t;

typedef struct {
    uint32_t width;
    uint32_t height;
    frm_fmt_t fmt;
    uint32_t bit_width;
} isp_ichn_attr_t;

typedef struct {
    uint32_t ddr_en;
    frm_fmt_t fmt;
    uint32_t bit_width;
} isp_ochn_attr_t;

#endif // __ISP_CFG_H__