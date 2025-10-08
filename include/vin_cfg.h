#ifndef __VIN_CFG_H__
#define __VIN_CFG_H__

#include <stdint.h>

// VIN配置相关的枚举和结构体定义

typedef enum {
    NOT_HDR = 0,
    HDR_2DOL = 1,
    HDR_3DOL = 2,
} hdr_mode_t;

typedef enum {
    VIN_BASIC_ATTR = 0,
    VIN_ADVANCED_ATTR = 1,
} vin_ochn_attr_type_t;

typedef enum {
    VIN_ATTR_EX_MCLK = 0,
    VIN_ATTR_EX_LPWM = 1,
    VIN_ATTR_EX_INVALID = 2,
} vin_attr_ex_type_t;

typedef struct {
    uint32_t enable_frame_id;
    uint32_t set_init_frame_id;
    hdr_mode_t hdr_mode;
    uint32_t time_stamp_en;
} vin_func_t;

typedef struct {
    uint32_t mipi_rx;
    uint32_t vc_index;
    uint32_t ipi_channel;
    uint32_t cim_isp_flyby;
    vin_func_t func;
} vin_cim_attr_t;

typedef struct {
    vin_cim_attr_t cim_attr;
    lpwm_attr_t lpwm_attr;
} vin_node_attr_t;

typedef struct {
    uint32_t width;
    uint32_t height;
    uint32_t format;
} vin_ichn_attr_t;

typedef struct {
    uint32_t format;
    uint32_t wstride;
} vin_basic_attr_t;

typedef struct {
    uint32_t ddr_en;
    vin_ochn_attr_type_t ochn_attr_type;
    vin_basic_attr_t vin_basic_attr;
} vin_ochn_attr_t;

typedef struct {
    uint32_t mclk_freq;
} mclk_ex_attr_t;

typedef struct {
    uint32_t enable;
} lpwm_attr_t;

typedef struct {
    uint32_t vin_attr_ex_mask;
    vin_attr_ex_type_t ex_attr_type;
    union {
        mclk_ex_attr_t mclk_ex_attr;
        lpwm_attr_t lpwm_attr;
    };
} vin_attr_ex_t;

#endif // __VIN_CFG_H__