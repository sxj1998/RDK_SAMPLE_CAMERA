#include <iostream>

// 正确包含C函数的头文件
extern "C" {
    #include "vp_sensors.h"
}

int main() {
    std::cout << "Testing vp_sensors functions..." << std::endl;
    
    // 测试vp_get_sensors_list_number函数
    uint32_t sensor_count = vp_get_sensors_list_number();
    std::cout << "Number of sensors available: " << sensor_count << std::endl;
    
    // 如果有传感器，测试vp_sensor_fixed_mipi_host函数
    if (sensor_count > 0) {
        vp_sensor_config_s sensor_config = {0};
        vp_csi_config_s csi_config = {0};
        
        int result = vp_sensor_fixed_mipi_host(&sensor_config, &csi_config);
        std::cout << "vp_sensor_fixed_mipi_host result: " << result << std::endl;
    }
    
    std::cout << "Test completed successfully!" << std::endl;
    return 0;
}