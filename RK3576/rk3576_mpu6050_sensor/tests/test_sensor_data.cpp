#include <cmath>
#include <iostream>

#include "sensor/sensor_data.h"

int main()
{
    // 1. 创建 SensorData 对象
    rk3576::sensor::SensorData data;

    // 2. 验证默认初始化
    if (data.sample_id != 0 ||
        data.timestamp_ns != 0 ||
        data.ax != 0.0) {

        std::cerr << "Default initialization failed\n";
        return 1;
    }

    // 3. 模拟一组 MPU6050 六轴数据
    data.ax = 0.456;
    data.ay = 1.032;
    data.az = 11.484;

    data.gx = -0.0373;
    data.gy = -0.0138;
    data.gz = 0.0170;

    data.timestamp_ns = 1000000000ULL;
    data.sample_id = 1;

    // 4. 验证数据是否保存正确
    if (std::abs(data.az - 11.484) > 1e-9 ||
        data.sample_id != 1) {

        std::cerr << "SensorData test failed\n";
        return 1;
    }

    // 5. 输出测试结果
    std::cout << "Sample ID: "
              << data.sample_id << '\n';

    std::cout << "Acceleration: "
              << data.ax << ", "
              << data.ay << ", "
              << data.az << " m/s^2\n";

    std::cout << "Gyroscope: "
              << data.gx << ", "
              << data.gy << ", "
              << data.gz << " rad/s\n";

    std::cout << "SensorData test passed\n";

    return 0;
}
