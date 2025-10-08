# 独立VIN示例工程

这是一个独立的VIN（Video Input）示例程序工程，包含了所有必要的依赖文件，可以独立编译运行。

## 工程结构
```
standalone_vin_project/
├── src/                    # 源代码目录
│   ├── main.c             # 主程序
│   ├── common_utils.c     # 公共工具函数
│   └── common_utils.h     # 公共工具头文件
├── vp_sensors/            # 传感器配置目录
│   ├── vp_sensors.c       # 传感器配置实现
│   ├── vp_sensors.h       # 传感器配置头文件
│   └── sensor_configs/    # 传感器配置文件
├── Makefile               # 编译配置文件
└── README.md             # 说明文档
```

## 功能特性

- ✅ 支持多传感器（最多4个）
- ✅ 视频流管道管理
- ✅ 帧数据捕获和保存
- ✅ LPWM低功耗模式控制
- ✅ 交互式命令界面

## 编译说明

### 环境要求
- 交叉编译工具链：aarch64-linux-gnu-
- Horizon Robotics平台SDK

### 编译命令
```bash
make all        # 编译程序
make clean      # 清理编译文件
make install    # 安装到目标目录
```

## 使用方法

### 基本用法
```bash
./get_vin_data -s <sensor_index>
```

### 命令行参数
- `-s <sensor_index>`: 指定传感器索引（必需）
- `-t <settle_value>`: 设置settle时间用于调试
- `-m <sensor_mode>`: 设置传感器模式（1: NORMAL_M, 2: DOL2_M, 6: SLAVE_M）
- `-h`: 显示帮助信息

### 交互命令
程序启动后支持以下交互命令：
- `g`: 获取单帧数据
- `l`: 获取12帧数据
- `x`: 启用LPWM
- `y`: 禁用LPWM
- `q`: 退出程序
- `h`: 显示帮助

## 输出文件

程序会将捕获的帧数据保存为RAW格式文件，命名格式：
```
handle_<node_handle>_vin_chn<channel>_<width>x<height>_stride_<stride>_frameid_<frame_id>_ts_<timestamp>.raw
```