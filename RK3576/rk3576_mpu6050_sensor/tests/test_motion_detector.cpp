
#include <array>
#include <cstdint>
#include <iostream>

#include "motion/motion_detector.h"

int main()
{
    using rk3576::motion::MotionDetector;
    using rk3576::motion::UnifiedEventType;
    using rk3576::sensor::SensorData;

    constexpr double G = 9.80665;

    // 测试采用车辆坐标与传感器坐标一致的情况
    MotionDetector detector;

    // 五种事件的计数
    std::array<int, 5> event_count{};

    std::uint64_t sample_id = 0;
    std::uint64_t timestamp_ns = 1000000000ULL;

    // 向统一检测器输入一组模拟六轴数据
    const auto feed = [&](double ax,
                          double ay,
                          double dynamic_az,
                          double gz) {

        SensorData data;

        data.ax = ax;
        data.ay = ay;

        // 模拟静态重力 + 动态垂向分量
        data.az = G + dynamic_az;
        data.gz = gz;

        data.sample_id = ++sample_id;
        data.timestamp_ns = timestamp_ns;

        timestamp_ns += 100000000ULL;

        const auto result = detector.process(data);

        if (!result) {
            std::cerr << "Pipeline rejected sample\n";
            return false;
        }

        for (const auto& event : result->events) {

            const auto index =
                static_cast<std::size_t>(event.type);

            if (index >= event_count.size()) {
                return false;
            }

            ++event_count[index];

            std::cout
                << "[Event] type=" << index
                << " sample=" << event.sample_id
                << " primary=" << event.primary_value
                << '\n';
        }

        return true;
    };

    const auto feedRepeated = [&](int count,
                                  double ax,
                                  double ay,
                                  double az,
                                  double gz) {

        for (int i = 0; i < count; ++i) {
            if (!feed(ax, ay, az, gz)) {
                return false;
            }
        }

        return true;
    };

    // 1. 静态初始化：50 个样本
    if (!feedRepeated(50, 0.0, 0.0, 0.0, 0.0)) {
        return 1;
    }

    if (!detector.ready()) {
        std::cerr << "Baseline not ready\n";
        return 1;
    }

    std::cout << "Baseline initialized\n";

    // 2. 空闲阶段
    if (!feedRepeated(5, 0.0, 0.0, 0.0, 0.0)) {
        return 1;
    }

    // 3. 急加速
    if (!feedRepeated(6, 5.0, 0.0, 0.0, 0.0)) {
        return 1;
    }

    // 恢复静止
    if (!feedRepeated(6, 0.0, 0.0, 0.0, 0.0)) {
        return 1;
    }

    // 4. 急刹车
    if (!feedRepeated(6, -5.0, 0.0, 0.0, 0.0)) {
        return 1;
    }

    if (!feedRepeated(6, 0.0, 0.0, 0.0, 0.0)) {
        return 1;
    }

    // 5. 左急转
    if (!feedRepeated(6, 0.0, 4.0, 0.0, 0.9)) {
        return 1;
    }

    if (!feedRepeated(6, 0.0, 0.0, 0.0, 0.0)) {
        return 1;
    }

    // 6. 右急转
    if (!feedRepeated(6, 0.0, -4.0, 0.0, -0.9)) {
        return 1;
    }

    if (!feedRepeated(6, 0.0, 0.0, 0.0, 0.0)) {
        return 1;
    }

    // 7. 短时垂向颠簸
    if (!feed(0.0, 0.0, 0.0, 0.0) ||
        !feed(0.0, 0.0, 3.5, 0.0) ||
        !feed(0.0, 0.0, 0.0, 0.0)) {

        return 1;
    }

    std::cout << "\n=== Unified Event Summary ===\n";

    const std::array<const char*, 5> names{
        "HarshAcceleration",
        "HarshBraking",
        "SharpLeftTurn",
        "SharpRightTurn",
        "Bump"
    };

    bool passed = true;

    for (std::size_t i = 0; i < names.size(); ++i) {

        std::cout
            << names[i]
            << ": " << event_count[i] << '\n';

        if (event_count[i] != 1) {
            passed = false;
        }
    }

    if (!passed) {
        std::cerr << "Unified motion detector FAILED\n";
        return 1;
    }

    // 检查重置功能
    detector.reset();

    if (detector.ready()) {
        std::cerr << "Reset FAILED\n";
        return 1;
    }

    std::cout
        << "Unified motion detector PASSED\n";

    return 0;
}
