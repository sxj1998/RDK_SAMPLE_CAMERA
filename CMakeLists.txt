# CMakeLists.txt for standalone VIN project
# 独立VIN数据采集工程CMake配置

# 设置CMake最低版本要求
cmake_minimum_required(VERSION 3.10)

# 项目名称
project(standalone_vin_project VERSION 1.0.0 LANGUAGES C)

# 设置C标准
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

# 编译标志
set(CMAKE_C_FLAGS "-Wall -Werror")
set(CMAKE_CXX_FLAGS "-Wall -Werror")

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

# 源文件列表
set(SRCS
    src/main.c
    src/common_utils.c
    src/vp_sensors/vp_sensors.c
    src/vp_sensors/dummy_sensor/dummy_sensor.c
    src/vp_sensors/sc230ai/linear_1920x1080_raw10_10fps_1lane.c
    src/vp_sensors/sc230ai/linear_1920x1080_raw10_30fps_1lane.c
    src/vp_sensors/f37/linear_1920x1080_raw10_30fps_1lane.c
)

# 创建可执行文件
add_executable(${PROJECT_NAME} ${SRCS})

# 链接库文件
target_link_libraries(${PROJECT_NAME}
    cam
    vpf
    hbmem
    gdcbin
    cjson
    pthread
    alog
    dl
)

# 设置输出目录
set_target_properties(${PROJECT_NAME} PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}
)

# 安装配置
install(TARGETS ${PROJECT_NAME}
    RUNTIME DESTINATION ${PLATFORM_SAMPLES_DEPLOY_DIR}/${PROJECT_NAME}
    PERMISSIONS OWNER_EXECUTE OWNER_WRITE OWNER_READ GROUP_EXECUTE GROUP_READ WORLD_EXECUTE WORLD_READ
)

# 自定义目标：clean
add_custom_target(clean_all
    COMMAND ${CMAKE_COMMAND} -E remove_directory ${CMAKE_BINARY_DIR}
    COMMENT "Cleaning all build files"
)

# 显示构建信息
message(STATUS "Project: ${PROJECT_NAME}")
message(STATUS "Source files: ${SRCS}")
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")
message(STATUS "Install directory: ${PLATFORM_SAMPLES_DEPLOY_DIR}/${PROJECT_NAME}")