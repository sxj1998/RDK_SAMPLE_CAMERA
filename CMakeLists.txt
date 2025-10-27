# CMakeLists.txt for standalone VIN project
# 独立VIN数据采集工程CMake配置

# 设置CMake最低版本要求
cmake_minimum_required(VERSION 3.10)

# 项目名称
project(standalone_vin_project VERSION 1.0.0 LANGUAGES C CXX)

# 设置C和C++标准
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 编译标志
set(CMAKE_C_FLAGS "-Wall -Werror")
set(CMAKE_CXX_FLAGS "-Wall -Werror")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(OpenCV REQUIRED)

# 交叉编译工具链配置
if(EXISTS "/opt/gcc-arm-11.2-2022.02-x86_64-aarch64-none-linux-gnu/bin/aarch64-none-linux-gnu-gcc")
    set(CROSS_COMPILE "/opt/gcc-arm-11.2-2022.02-x86_64-aarch64-none-linux-gnu/bin/aarch64-none-linux-gnu-")
else()
    set(CROSS_COMPILE "aarch64-linux-gnu-")
endif()

# 设置交叉编译工具链
set(CMAKE_C_COMPILER ${CROSS_COMPILE}gcc)
set(CMAKE_CXX_COMPILER ${CROSS_COMPILE}g++)
set(CMAKE_AR ${CROSS_COMPILE}ar)
set(CMAKE_ASM_COMPILER ${CROSS_COMPILE}as)
set(CMAKE_LINKER ${CROSS_COMPILE}ld)
set(CMAKE_STRIP ${CROSS_COMPILE}strip)
set(CMAKE_RANLIB ${CROSS_COMPILE}ranlib)
set(CMAKE_OBJCOPY ${CROSS_COMPILE}objcopy)
set(CMAKE_OBJDUMP ${CROSS_COMPILE}objdump)
set(CMAKE_READELF ${CROSS_COMPILE}readelf)
set(CMAKE_NM ${CROSS_COMPILE}nm)

# RDK环境配置
set(HR_TOP_DIR "/usr/hobot")
if(NOT DEFINED HR_BUILD_OUTPUT_DIR)
    set(HR_BUILD_OUTPUT_DIR ${HR_TOP_DIR})
endif()

if(NOT DEFINED APP_DEPLOY_DIR)
    set(APP_DEPLOY_DIR "/app/")
endif()

set(PLATFORM_SAMPLES_DEPLOY_DIR "${APP_DEPLOY_DIR}/platform_samples")

# 获取当前目录的绝对路径
get_filename_component(PLATFORM_SAMPLES_DIR "${CMAKE_SOURCE_DIR}/.." ABSOLUTE)

# 输出平台目录信息
message(STATUS "PLATFORM_SAMPLES_DIR directory (absolute): ${PLATFORM_SAMPLES_DIR}")

# 包含目录配置
include_directories(
    .
    ${HR_BUILD_OUTPUT_DIR}/include/
    ${PLATFORM_SAMPLES_DIR}/include/
    ${PLATFORM_SAMPLES_DIR}/utils/
    ${PLATFORM_SAMPLES_DIR}/vp_sensors
    ./src
    ./src/vp_sensors
)

# 库目录配置
link_directories(
    ${HR_BUILD_OUTPUT_DIR}/lib
    ${PLATFORM_SAMPLES_DIR}/lib
)

# vp_sensors源文件
set(VPSENSORS_SRCS
    src/vp_sensors/vp_sensors.c
    src/vp_sensors/dummy_sensor/dummy_sensor.c
    src/vp_sensors/sc230ai/linear_1920x1080_raw10_10fps_1lane.c
    src/vp_sensors/sc230ai/linear_1920x1080_raw10_30fps_1lane.c
    src/vp_sensors/f37/linear_1920x1080_raw10_30fps_1lane.c
)

# 创建vp_sensors静态库
add_library(vp_sensors STATIC ${VPSENSORS_SRCS})

# 主源文件列表
set(SRCS
#    src/main.c
    src/main.cpp
    src/mipi_cam.cpp
    src/mipi_pipe.cpp
    src/common_utils.c
)

# 创建可执行文件
add_executable(${PROJECT_NAME} ${SRCS})

# 设置输出目录
set_target_properties(${PROJECT_NAME} PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}
)

# 链接ROS2依赖
ament_target_dependencies(
  ${PROJECT_NAME}
  "rclcpp"
  "std_msgs"
  "sensor_msgs"
  "yaml_cpp_vendor"
  "OpenCV"
)

# 链接库文件 - 添加必要的库以确保正确编译
target_link_libraries(${PROJECT_NAME}
    vp_sensors  # 首先链接vp_sensors库
    cam
    vpf
    hbmem
    gdcbin
    cjson
    pthread
    alog
    dl
)

install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})
# 安装配置
install(TARGETS ${PROJECT_NAME}
    RUNTIME DESTINATION ${PLATFORM_SAMPLES_DEPLOY_DIR}/${PROJECT_NAME}
    PERMISSIONS OWNER_EXECUTE OWNER_WRITE OWNER_READ GROUP_EXECUTE GROUP_READ WORLD_EXECUTE WORLD_READ
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

# 显示构建信息
message(STATUS "Project: ${PROJECT_NAME}")
message(STATUS "Source files: ${SRCS}")
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")
message(STATUS "Install directory: ${PLATFORM_SAMPLES_DEPLOY_DIR}/${PROJECT_NAME}")