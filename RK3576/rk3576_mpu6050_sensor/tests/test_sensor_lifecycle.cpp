
#include <chrono>
#include <cstdint>
#include <iostream>
#include <thread>

#include "sensor/sensor_service.h"

int main()
{
    using namespace std::chrono_literals;
    using rk3576::sensor::SensorData;
    using rk3576::sensor::SensorService;

    // 1. 创建传感器服务
    SensorService service(100ms);

    SensorData data;

    // 2. 未启动时调用 stop()
    service.stop();

    if (service.getLatest(data)) {
        std::cerr << "Unexpected data before start\n";
        return 1;
    }

    // 3. 首次启动
    if (!service.start()) {
        std::cerr << "First start failed\n";
        return 1;
    }

    std::cout << "First start succeeded\n";

    // 4. 重复启动应该失败
    if (service.start()) {
        std::cerr << "Duplicate start was accepted\n";
        service.stop();
        return 1;
    }

    std::cout << "Duplicate start rejected\n";

    // 5. 等待后台采集
    std::this_thread::sleep_for(600ms);

    if (!service.getLatest(data)) {
        std::cerr << "No data after first start\n";
        service.stop();
        return 1;
    }

    const std::uint64_t first_timestamp =
        data.timestamp_ns;

    // 6. 第一次停止
    service.stop();

    const auto first_count = service.sampleCount();

    std::cout << "First run samples: "
              << first_count << '\n';

    // 7. 停止后不应该继续采集
    std::this_thread::sleep_for(200ms);

    if (service.sampleCount() != first_count) {
        std::cerr << "Samples changed after stop\n";
        return 1;
    }

    // 8. 再次停止应该安全
    service.stop();

    std::cout << "Repeated stop succeeded\n";

    // 9. 第二次启动
    if (!service.start()) {
        std::cerr << "Restart failed\n";
        return 1;
    }

    std::cout << "Restart succeeded\n";

    std::this_thread::sleep_for(600ms);

    SensorData restarted_data;

    if (!service.getLatest(restarted_data)) {
        std::cerr << "No data after restart\n";
        service.stop();
        return 1;
    }

    // 检查重新启动后的样本
    if (restarted_data.sample_id == 0 ||
        restarted_data.timestamp_ns <= first_timestamp) {

        std::cerr << "Invalid data after restart\n";
        service.stop();
        return 1;
    }

    service.stop();

    std::cout << "Second run samples: "
              << service.sampleCount() << '\n';

    // 10. 测试无效周期
    SensorService invalid_service(0ms);

    if (invalid_service.start()) {
        std::cerr << "Invalid period was accepted\n";
        invalid_service.stop();
        return 1;
    }

    std::cout << "Invalid period rejected\n";

    // 11. 测试析构自动停止
    {
        SensorService scoped_service(100ms);

        if (!scoped_service.start()) {
            std::cerr << "Scoped service start failed\n";
            return 1;
        }

        std::this_thread::sleep_for(300ms);

        std::cout << "Leaving service scope\n";

        // 不手动调用 stop()
        // 离开作用域后自动调用析构函数
    }

    std::cout << "Destructor cleanup succeeded\n";
    std::cout << "Sensor lifecycle test passed\n";

    return 0;
}
