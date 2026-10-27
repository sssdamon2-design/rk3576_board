
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

    if (!service.start()) {
        std::cerr << "SensorService start failed\n";
        return 1;
    }

    // 连续采集约 3 秒
    std::this_thread::sleep_for(3s);

    service.stop();

    const auto s = service.stats();

    SensorData latest;

    if (!service.getLatest(latest)) {
        std::cerr << "No latest data\n";
        return 1;
    }

    std::cout << "\n=== Sensor Statistics ===\n";

    std::cout << "Attempts: "
              << s.read_attempts << '\n';

    std::cout << "Successful: "
              << s.successful_samples << '\n';

    std::cout << "Failed: "
              << s.failed_reads << '\n';

    std::cout << "Actual Hz: "
              << s.actual_hz << '\n';

    std::cout << "Average read(ms): "
              << s.avg_read_ms << '\n';

    std::cout << "Maximum read(ms): "
              << s.max_read_ms << '\n';

    std::cout << "Maximum period error(ms): "
              << s.max_period_error_ms << '\n';

    std::cout << "Data age(ms): "
              << s.data_age_ms << '\n';

    // 验证统计关系
    if (s.read_attempts == 0 ||
        s.successful_samples < 2 ||
        s.read_attempts !=
            s.successful_samples + s.failed_reads ||
        s.successful_samples != service.sampleCount() ||
        latest.sample_id != s.successful_samples ||
        !s.has_data ||
        !std::isfinite(s.actual_hz) ||
        s.actual_hz <= 0.0 ||
        !std::isfinite(s.avg_read_ms) ||
        s.avg_read_ms < 0.0 ||
        s.max_read_ms < s.avg_read_ms) {

        std::cerr << "Sensor statistics test failed\n";
        return 1;
    }

    // 停止采集后，缓存数据会逐渐变旧
    std::this_thread::sleep_for(600ms);

    const auto later = service.stats();

    std::cout << "Data age after stop(ms): "
              << later.data_age_ms << '\n';

    if (later.data_age_ms <
        s.data_age_ms + 300.0) {

        std::cerr << "Data age not increasing\n";
        return 1;
    }

    std::cout << "Sensor statistics test passed\n";

    return 0;
}
