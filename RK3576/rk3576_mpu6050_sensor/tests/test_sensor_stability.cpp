
#include <chrono>
#include <cmath>
#include <exception>
#include <iostream>
#include <string>
#include <thread>

#include "sensor/sensor_service.h"

int main(int argc, char* argv[])
{
    using namespace std::chrono;
    using rk3576::sensor::SensorData;
    using rk3576::sensor::SensorService;

    // 默认测试 60 秒；也可以传入 1800 秒
    int duration_sec = 60;

    if (argc > 2) {
        std::cerr << "Usage: test_sensor_stability [seconds]\n";
        return 1;
    }

    if (argc == 2) {
        try {
            std::size_t pos = 0;
            duration_sec = std::stoi(argv[1], &pos);

            if (argv[1][pos] != '\0' || duration_sec <= 0) {
                throw std::invalid_argument("invalid duration");
            }
        }
        catch (const std::exception& e) {
            std::cerr << "Invalid duration: "
                      << e.what() << '\n';
            return 1;
        }
    }

    // 目标：100ms 一次，即 10Hz
    SensorService service(milliseconds{100});

    if (!service.start()) {
        std::cerr << "SensorService start failed\n";
        return 1;
    }

    const auto begin = steady_clock::now();
    bool healthy = true;

    // 每秒检查一次运行状态
    for (int sec = 1; sec <= duration_sec; ++sec) {

        std::this_thread::sleep_until(
            begin + seconds{sec});

        const auto s = service.stats();
        SensorData data;

        // 检查计数一致性和数据是否过期
        if (!service.getLatest(data) ||
            !s.has_data ||
            s.read_attempts !=
                s.successful_samples + s.failed_reads ||
            s.data_age_ms > 1000.0 ||
            data.sample_id == 0) {

            std::cerr << "Health check failed at "
                      << sec << " seconds\n";

            healthy = false;
            break;
        }

        // 每 10 秒输出一次简要统计
        if (sec == 1 ||
            sec % 10 == 0 ||
            sec == duration_sec) {

            std::cout
                << "[Health] elapsed=" << sec << "s"
                << " success=" << s.successful_samples
                << " failed=" << s.failed_reads
                << " hz=" << s.actual_hz
                << " age_ms=" << s.data_age_ms
                << std::endl;
        }
    }

    // 测量停止服务的耗时
    const auto stop_begin = steady_clock::now();
    service.stop();
    const auto stop_end = steady_clock::now();

    const double stop_ms =
        duration<double, std::milli>(
            stop_end - stop_begin).count();

    const auto s = service.stats();

    std::cout << "\n=== Stability Summary ===\n"
              << "Attempts: " << s.read_attempts << '\n'
              << "Successful: " << s.successful_samples << '\n'
              << "Failed: " << s.failed_reads << '\n'
              << "Actual Hz: " << s.actual_hz << '\n'
              << "Average read(ms): " << s.avg_read_ms << '\n'
              << "Maximum read(ms): " << s.max_read_ms << '\n'
              << "Max period error(ms): "
              << s.max_period_error_ms << '\n'
              << "Stop latency(ms): " << stop_ms << '\n';

    // 初步验收：无失败，平均采样率在 9~11Hz
    const bool passed =
        healthy &&
        s.successful_samples >= 2 &&
        s.failed_reads == 0 &&
        s.read_attempts == s.successful_samples &&
        s.actual_hz >= 9.0 &&
        s.actual_hz <= 11.0 &&
        std::isfinite(s.avg_read_ms) &&
        s.max_read_ms >= s.avg_read_ms;

    if (!passed) {
        std::cerr << "Stability test FAILED\n";
        return 1;
    }

    std::cout << "Stability test PASSED\n";
    return 0;
}
