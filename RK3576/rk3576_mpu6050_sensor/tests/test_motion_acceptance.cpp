
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <iostream>
#include <thread>

#include "sensor/sensor_service.h"
#include "motion/motion_detector.h"

int main()
{
    using namespace std::chrono_literals;

    using rk3576::sensor::SensorData;
    using rk3576::sensor::SensorService;
    using rk3576::motion::MotionDetector;
    using rk3576::motion::MotionDetectorConfig;

    // 1. 使用 4.1 实测校准参数
    MotionDetectorConfig config;

    config.accel = {
         0.212559,
        -0.144483,
         1.515511,
         1.001213,
         1.006555,
         0.978363
    };

    config.gyro = {
        -0.0382023,
        -0.0129790,
         0.0148742
    };

    config.baseline_samples = 50;
    config.filter_tau_seconds = 0.1;

    // 静态验收沿用传感器坐标系
    // 不将当前轴向假定为真实车辆坐标系
    MotionDetector detector(config);

    SensorService service(100ms);

    // 这些变量只由采集回调线程写入，
    // 主线程在 stop() 完成后才读取
    std::uint64_t callback_count = 0;
    std::uint64_t rejected_count = 0;
    std::uint64_t event_count = 0;
    std::uint64_t sequence_errors = 0;

    std::uint64_t last_sample_id = 0;
    std::uint64_t last_timestamp_ns = 0;

    double max_sample_gap_ms = 0.0;
    double max_motion_norm = 0.0;

    // 2. 注册连续采样回调
    if (!service.setSampleCallback(
        [&](const SensorData& sample) {

            ++callback_count;

            // 检查成功样本序号是否连续
            if (last_sample_id != 0 &&
                sample.sample_id != last_sample_id + 1) {
                ++sequence_errors;
            }

            last_sample_id = sample.sample_id;

            // 统计相邻成功样本间隔
            if (last_timestamp_ns != 0) {

                if (sample.timestamp_ns <= last_timestamp_ns) {
                    ++sequence_errors;
                } else {

                    const double gap_ms =
                        static_cast<double>(
                            sample.timestamp_ns -
                            last_timestamp_ns
                        ) / 1000000.0;

                    if (gap_ms > max_sample_gap_ms) {
                        max_sample_gap_ms = gap_ms;
                    }
                }
            }

            last_timestamp_ns = sample.timestamp_ns;

            // 3. 完整运动检测流水线
            const auto result = detector.process(sample);

            if (!result) {
                ++rejected_count;
                return;
            }

            if (result->ready) {

                const auto& motion = result->smooth_motion;

                const double motion_norm = std::sqrt(
                    motion.ax * motion.ax +
                    motion.ay * motion.ay +
                    motion.az * motion.az
                );

                if (motion_norm > max_motion_norm) {
                    max_motion_norm = motion_norm;
                }
            }

            // 4. 记录运动事件
            for (const auto& event : result->events) {

                ++event_count;

                std::cout
                    << "[MotionEvent] type="
                    << static_cast<int>(event.type)
                    << " sample=" << event.sample_id
                    << " value=" << event.primary_value
                    << '\n';
            }

            // 每 100 组输出一次运行状态
            if (callback_count % 100 == 0) {
                std::cout
                    << "[Health] callbacks="
                    << callback_count
                    << " events=" << event_count
                    << " ready=" << detector.ready()
                    << '\n';
            }
        })) {

        std::cerr << "Callback registration failed\n";
        return 1;
    }

    // 5. 开始 60 秒静态测试
    std::cout
        << "=== Motion Acceptance Test ===\n"
        << "Keep MPU6050 stationary for 60 seconds\n";

    if (!service.start()) {
        return 1;
    }

    std::this_thread::sleep_for(60s);

    // 等待采集线程及回调彻底结束
    const auto stop_begin =
        std::chrono::steady_clock::now();

    service.stop();

    const auto stop_end =
        std::chrono::steady_clock::now();

    const double stop_latency_ms =
        std::chrono::duration<double, std::milli>(
            stop_end - stop_begin
        ).count();

    // 6. 获取完整统计结果
    const auto stats = service.stats();

    std::array<double, 3> baseline{};
    const bool baseline_ready =
        detector.getBaseline(baseline);

    std::cout << "\n=== Acceptance Summary ===\n";

    std::cout
        << "Attempts: " << stats.read_attempts << '\n'
        << "Successful: " << stats.successful_samples << '\n'
        << "Failed reads: " << stats.failed_reads << '\n'
        << "Callback samples: " << callback_count << '\n'
        << "Rejected samples: " << rejected_count << '\n'
        << "Sequence errors: " << sequence_errors << '\n'
        << "Motion events: " << event_count << '\n'
        << "Actual Hz: " << stats.actual_hz << '\n'
        << "Average read(ms): " << stats.avg_read_ms << '\n'
        << "Maximum read(ms): " << stats.max_read_ms << '\n'
        << "Max period error(ms): "
        << stats.max_period_error_ms << '\n'
        << "Max callback sample gap(ms): "
        << max_sample_gap_ms << '\n'
        << "Max motion norm(m/s^2): "
        << max_motion_norm << '\n'
        << "Stop latency(ms): "
        << stop_latency_ms << '\n'
        << "Baseline ready: " << baseline_ready << '\n';

    if (baseline_ready) {
        std::cout
            << "Baseline XYZ: "
            << baseline[0] << ", "
            << baseline[1] << ", "
            << baseline[2] << '\n';
    }

    // 7. 最终验收条件
    const bool passed =
        stats.successful_samples >= 500 &&
        stats.failed_reads == 0 &&
        stats.successful_samples == callback_count &&
        stats.read_attempts ==
            stats.successful_samples + stats.failed_reads &&
        rejected_count == 0 &&
        sequence_errors == 0 &&
        event_count == 0 &&
        baseline_ready &&
        stats.actual_hz >= 9.0 &&
        stats.actual_hz <= 11.0;

    // 长尾读取仅提示，不直接决定验收失败
    if (stats.max_read_ms > 100.0) {
        std::cerr
            << "WARNING: Read latency exceeded 100ms\n";
    }

    if (!passed) {
        std::cerr
            << "Motion acceptance FAILED\n";
        return 1;
    }

    std::cout
        << "Motion acceptance PASSED\n";

    return 0;
}
