
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <thread>

#include "sensor/sensor_service.h"

int main()
{
    using namespace std::chrono_literals;
    using rk3576::sensor::SensorData;
    using rk3576::sensor::SensorService;

    SensorService service(100ms);

    SensorData data;

    // 1. 启动前不应该有有效数据
    if (service.getLatest(data)) {
        std::cerr << "Unexpected data before start\n";
        return 1;
    }

    // 2. 启动后台采集
    if (!service.start()) {
        std::cerr << "SensorService start failed\n";
        return 1;
    }

    std::uint64_t previous_id = 0;
    bool received_update = false;

    // 3. 主线程每 400ms 读取最新数据
    for (int i = 0; i < 5; ++i) {

        std::this_thread::sleep_for(400ms);

        if (!service.getLatest(data)) {
            std::cerr << "No latest sensor data\n";
            service.stop();
            return 1;
        }

        // 检查样本序号不能倒退
        if (data.sample_id < previous_id) {
            std::cerr << "Sample ID went backwards\n";
            service.stop();
            return 1;
        }

        // 检查是否收到更新
        if (data.sample_id > previous_id) {
            received_update = true;
        }

        previous_id = data.sample_id;

        // 检查六轴数据是否有效
        if (!std::isfinite(data.ax) ||
            !std::isfinite(data.ay) ||
            !std::isfinite(data.az) ||
            !std::isfinite(data.gx) ||
            !std::isfinite(data.gy) ||
            !std::isfinite(data.gz)) {

            std::cerr << "Invalid sensor data\n";
            service.stop();
            return 1;
        }

        std::cout
            << "[Main] sample_id=" << data.sample_id
            << " ax=" << data.ax
            << " ay=" << data.ay
            << " az=" << data.az
            << '\n';
    }

    // 4. 停止后台线程
    service.stop();

    if (!received_update || service.sampleCount() == 0) {
        std::cerr << "No successful samples\n";
        return 1;
    }

    // 5. 停止后，最新一组数据仍然可以读取
    SensorData last_data;

    if (!service.getLatest(last_data)) {
        std::cerr << "Latest data lost after stop\n";
        return 1;
    }

    std::cout << "Total samples: "
              << service.sampleCount() << '\n';

    std::cout << "Last sample ID: "
              << last_data.sample_id << '\n';

    std::cout << "Latest sensor test passed\n";

    return 0;
}
