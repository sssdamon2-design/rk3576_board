
#include <chrono>
#include <cstdint>
#include <iostream>
#include <thread>

#include "sensor/sensor_service.h"

int main()
{
    using rk3576::sensor::SensorService;
    using rk3576::sensor::SensorData;
    using namespace std::chrono_literals;

    SensorService service(100ms);

    // 仅在回调线程中修改；
    // 主线程等 stop() 完成后再读取。
    std::uint64_t callback_count = 0;

    bool data_consistent = true;

    // 1. 注册成功采样回调
    const bool registered =
        service.setSampleCallback(
            [&](const SensorData& sample) {

                ++callback_count;

                // 验证回调调用时 latest_mutex_
                // 已经释放，可以安全调用 getLatest()
                SensorData latest;

                if (!service.getLatest(latest)) {
                    data_consistent = false;
                    return;
                }

                if (latest.sample_id != sample.sample_id ||
                    latest.timestamp_ns !=
                        sample.timestamp_ns) {

                    data_consistent = false;
                }
            }
        );

    if (!registered) {
        std::cerr << "Callback registration failed\n";
        return 1;
    }

    // 2. 启动后台采集
    if (!service.start()) {
        return 1;
    }

    // 3. 验证运行期间不允许替换回调
    if (service.setSampleCallback({})) {
        std::cerr
            << "Callback changed while running\n";
        service.stop();
        return 1;
    }

    // 4. 持续采集约一秒
    std::this_thread::sleep_for(1s);

    service.stop();

    // 5. 线程退出后检查回调数量
    const auto samples = service.sampleCount();

    std::cout << "Successful samples: "
              << samples << '\n';

    std::cout << "Callback count: "
              << callback_count << '\n';

    if (samples == 0 ||
        callback_count != samples ||
        !data_consistent) {

        std::cerr
            << "Sensor callback test FAILED\n";
        return 1;
    }

    std::cout
        << "Sensor callback test PASSED\n";

    return 0;
}
