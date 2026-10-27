
#include <iostream>

#include "sensor/mpu6050_reader.h"

int main()
{
    rk3576::sensor::MPU6050Reader reader;

    // 自动发现 MPU6050
    if (!reader.initialize()) {
        std::cerr << "MPU6050 discovery failed\n";
        return 1;
    }

    // 输出发现的设备路径
    std::cout << "IIO Path: "
              << reader.devicePath() << '\n';

    std::cout << "MPU6050 discovery test passed\n";

    return 0;
}
