#ifndef __HB_CAMERA_DATA_CONFIG_H__
#define __HB_CAMERA_DATA_CONFIG_H__

#include <stdint.h>

// 相机数据配置相关的结构体定义

typedef struct {
    uint32_t phy;
    uint32_t lane;
    uint32_t datatype;
    uint32_t fps;
    uint32_t mclk;
    uint32_t mipiclk;
    uint32_t width;
    uint32_t height;
    uint32_t linelenth;
    uint32_t framelenth;
    uint32_t settle;
    uint32_t channel_num;
    uint32_t channel_sel[4];
    uint32_t hsdTime;
    uint32_t hsaTime;
    uint32_t hbpTime;
} mipi_rx_attr_t;

typedef struct {
    uint32_t stop_check_instart;
} mipi_rx_attr_ex_t;

typedef struct {
    uint32_t rx_enable;
    mipi_rx_attr_t rx_attr;
    uint32_t rx_ex_mask;
    mipi_rx_attr_ex_t rx_attr_ex;
} mipi_config_t;

typedef enum {
    NORMAL_M = 0,
    HDR_M = 1,
    INVALID_MOD = 2,
} sensor_mode_t;

typedef struct {
    char name[64];
    uint32_t addr;
    sensor_mode_t sensor_mode;
    uint32_t fps;
    uint32_t format;
    uint32_t width;
    uint32_t height;
    uint32_t gpio_enable_bit;
    uint32_t gpio_level_bit;
    mipi_config_t *mipi_cfg;
    char calib_lname[64];
    uint32_t config_index;
} camera_config_t;

#endif // __HB_CAMERA_DATA_CONFIG_H__