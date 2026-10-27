
#include <chrono>
#include <iostream>
#include <thread>

#include "sensor/sensor_service.h"

int main()
{
    using namespace std::chrono_literals;

    // 创建 SensorService，目标周期 100ms
    rk3576::sensor::SensorService service(100ms);

    // 启动采集
    if (!service.start()) {
        std::cerr << "SensorService start failed\n";
        return 1;
    }

    std::cout << "SensorService started\n";

    // 主线程继续执行其他工作
    std::this_thread::sleep_for(3s);

    // 停止采集并等待线程退出
    service.stop();

    const auto count = service.sampleCount();

    std::cout << "Successful samples: "
              << count << '\n';

    // 至少应成功采集一次
    if (count == 0) {
        std::cerr << "No sensor data collected\n";
        return 1;
    }

    std::cout << "SensorService test passed\n";

    return 0;
}
