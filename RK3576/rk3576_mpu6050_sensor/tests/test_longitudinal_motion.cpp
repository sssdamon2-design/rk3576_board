
#include <cstdint>
#include <iostream>

#include "motion/longitudinal_motion_detector.h"

int main()
{
    using rk3576::motion::LongitudinalMotionDetector;
    using rk3576::motion::MotionEventType;
    using rk3576::sensor::SensorData;

    LongitudinalMotionDetector detector;

    std::uint64_t sample_id = 0;

    // 输入模拟样本，时间单位 ms
    const auto feed = [&](std::uint64_t time_ms,
                          double ax) {

        SensorData data;

        data.timestamp_ns =
            1000000000ULL + time_ms * 1000000ULL;

        data.sample_id = ++sample_id;
        data.ax = ax;

        return detector.process(data);
    };

    // 1. 急加速检测：连续超过阈值 200ms
    if (feed(0, 0.0) ||
        feed(100, 2.8) ||
        feed(200, 2.9)) {

        std::cerr << "Early acceleration event\n";
        return 1;
    }

    const auto acceleration = feed(300, 2.7);

    if (!acceleration ||
        acceleration->type !=
            MotionEventType::HarshAcceleration) {

        std::cerr << "Acceleration detection FAILED\n";
        return 1;
    }

    std::cout
        << "Harsh acceleration detected: ax="
        << acceleration->longitudinal_accel
        << '\n';

    // 2. 已触发状态下不应重复报警
    if (feed(400, 3.0)) {
        std::cerr << "Duplicate acceleration event\n";
        return 1;
    }

    // 3. 回到空闲，并测试急刹车
    if (feed(500, 0.0) ||
        feed(600, -3.0) ||
        feed(700, -2.7)) {

        std::cerr << "Early braking event\n";
        return 1;
    }

    const auto braking = feed(800, -3.1);

    if (!braking ||
        braking->type != MotionEventType::HarshBraking) {

        std::cerr << "Braking detection FAILED\n";
        return 1;
    }

    std::cout
        << "Harsh braking detected: ax="
        << braking->longitudinal_accel
        << '\n';

    // 4. 单个短时尖峰不能触发事件
    if (feed(900, -3.2) ||
        feed(1000, 0.0) ||
        feed(1100, 3.5) ||
        feed(1200, 0.0)) {

        std::cerr << "False event from short spike\n";
        return 1;
    }

    std::cout << "Short spike correctly rejected\n";

    // 5. 恢复之后能够再次识别急加速
    if (feed(1300, 3.0) ||
        feed(1400, 3.1)) {

        std::cerr << "Unexpected early event\n";
        return 1;
    }

    const auto acceleration2 = feed(1500, 3.2);

    if (!acceleration2 ||
        acceleration2->type !=
            MotionEventType::HarshAcceleration) {

        std::cerr << "Re-trigger test FAILED\n";
        return 1;
    }

    // 6. 数据中断超过 500ms，必须重新开始判断
    if (feed(2500, -3.0) ||
        feed(2600, -3.0)) {

        std::cerr << "Invalid gap handling\n";
        return 1;
    }

    const auto braking2 = feed(2700, -3.0);

    if (!braking2 ||
        braking2->type != MotionEventType::HarshBraking) {

        std::cerr << "Gap recovery FAILED\n";
        return 1;
    }

    // 7. reset 后可以重新处理数据流
    detector.reset();

    if (feed(0, 0.0) ||
        feed(100, 2.8)) {

        std::cerr << "Reset test FAILED\n";
        return 1;
    }

    std::cout
        << "Longitudinal motion detector PASSED\n";

    return 0;
}
