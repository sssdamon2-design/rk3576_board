
#include <cmath>
#include <iostream>

#include "sensor/mpu6050_reader.h"

int main()
{
    rk3576::sensor::MPU6050Reader reader;

    // 1. 初始化设备
    if (!reader.initialize()) {
        std::cerr << "Initialize failed\n";
        return 1;
    }

    rk3576::sensor::SensorData data1;
    rk3576::sensor::SensorData data2;

    // 2. 读取第一组数据
    if (!reader.read(data1)) {
        std::cerr << "First read failed\n";
        return 1;
    }

    // 3. 读取第二组数据
    if (!reader.read(data2)) {
        std::cerr << "Second read failed\n";
        return 1;
    }

    // 4. 验证采样序号与时间戳
    if (data1.sample_id != 1 ||
        data2.sample_id != 2 ||
        data1.timestamp_ns == 0 ||
        data2.timestamp_ns < data1.timestamp_ns) {

        std::cerr << "Timestamp or sample ID error\n";
        return 1;
    }

    // 5. 检查物理量是否为有限数
    if (!std::isfinite(data2.ax) ||
        !std::isfinite(data2.ay) ||
        !std::isfinite(data2.az) ||
        !std::isfinite(data2.gx) ||
        !std::isfinite(data2.gy) ||
        !std::isfinite(data2.gz)) {

        std::cerr << "Invalid sensor value\n";
        return 1;
    }

    // 6. 输出第二组六轴数据
    std::cout << "Sample ID: "
              << data2.sample_id << '\n';

    std::cout << "Timestamp(ns): "
              << data2.timestamp_ns << '\n';

    std::cout << "Acceleration: "
              << data2.ax << ", "
              << data2.ay << ", "
              << data2.az << " m/s^2\n";

    std::cout << "Gyroscope: "
              << data2.gx << ", "
              << data2.gy << ", "
              << data2.gz << " rad/s\n";

    std::cout << "MPU6050 read test passed\n";

    return 0;
}
