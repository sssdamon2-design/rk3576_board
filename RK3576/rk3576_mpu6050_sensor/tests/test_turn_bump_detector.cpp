
#include <cstdint>
#include <iostream>

#include "motion/turn_bump_detector.h"

int main()
{
    using rk3576::motion::TurnDetector;
    using rk3576::motion::BumpDetector;
    using rk3576::motion::TurnBumpEventType;
    using rk3576::sensor::SensorData;

    TurnDetector turn_detector;
    BumpDetector bump_detector;

    std::uint64_t turn_sample_id = 0;
    std::uint64_t bump_sample_id = 0;

    // 构造急转弯模拟数据
    const auto feedTurn =
        [&](std::uint64_t time_ms,
            double ay,
            double gz) {

        SensorData data;

        data.timestamp_ns =
            1000000000ULL + time_ms * 1000000ULL;

        data.sample_id = ++turn_sample_id;
        data.ay = ay;
        data.gz = gz;

        return turn_detector.process(data);
    };

    // ------------------------------
    // 1. 左急转弯测试
    // ------------------------------

    if (feedTurn(0, 0.0, 0.0) ||
        feedTurn(100, 2.0, 0.45) ||
        feedTurn(200, 2.1, 0.46)) {

        std::cerr << "Early left turn event\n";
        return 1;
    }

    const auto left = feedTurn(300, 2.0, 0.43);

    if (!left ||
        left->type != TurnBumpEventType::SharpLeftTurn) {

        std::cerr << "Left turn FAILED\n";
        return 1;
    }

    std::cout << "Sharp left turn detected\n";

    // 持续转弯不应重复报警
    if (feedTurn(400, 2.2, 0.46)) {
        std::cerr << "Duplicate left turn\n";
        return 1;
    }

    // 回归空闲
    if (feedTurn(500, 0.0, 0.0)) {
        return 1;
    }

    // ------------------------------
    // 2. 右急转弯测试
    // ------------------------------

    if (feedTurn(600, -2.0, -0.45) ||
        feedTurn(700, -2.1, -0.44)) {

        std::cerr << "Early right turn event\n";
        return 1;
    }

    const auto right = feedTurn(800, -2.2, -0.46);

    if (!right ||
        right->type != TurnBumpEventType::SharpRightTurn) {

        std::cerr << "Right turn FAILED\n";
        return 1;
    }

    std::cout << "Sharp right turn detected\n";

    // ------------------------------
    // 3. 符号矛盾：不应识别为正常急转弯
    // ------------------------------

    if (feedTurn(900, -2.0, 0.45)) {
        std::cerr << "Invalid direction accepted\n";
        return 1;
    }

    std::cout << "Direction mismatch rejected\n";

    // ------------------------------
    // 4. 数据缺口测试
    // ------------------------------

    if (feedTurn(1000, 2.0, 0.45) ||
        feedTurn(1100, 2.0, 0.45) ||
        feedTurn(2000, 2.0, 0.45) ||
        feedTurn(2100, 2.0, 0.45)) {

        std::cerr << "Gap handling FAILED\n";
        return 1;
    }

    const auto left2 = feedTurn(2200, 2.0, 0.45);

    if (!left2 ||
        left2->type != TurnBumpEventType::SharpLeftTurn) {

        std::cerr << "Turn recovery FAILED\n";
        return 1;
    }

    std::cout << "Turn gap recovery PASSED\n";

    // ------------------------------
    // 5. 颠簸检测
    // ------------------------------

    const auto feedBump =
        [&](std::uint64_t time_ms,
            double az) {

        SensorData data;

        data.timestamp_ns =
            1000000000ULL + time_ms * 1000000ULL;

        data.sample_id = ++bump_sample_id;
        data.az = az;

        return bump_detector.process(data);
    };

    // 建立第一组样本
    if (feedBump(0, 0.0)) {
        return 1;
    }

    // 明显垂向冲击
    const auto bump1 = feedBump(100, 3.2);

    if (!bump1 ||
        bump1->type != TurnBumpEventType::Bump) {

        std::cerr << "Bump detection FAILED\n";
        return 1;
    }

    std::cout << "Bump detected: az="
              << bump1->primary_value << '\n';

    // 500ms 内抑制重复报警
    if (feedBump(200, 0.0) ||
        feedBump(300, 3.0)) {

        std::cerr << "Bump cooldown FAILED\n";
        return 1;
    }

    std::cout << "Bump cooldown PASSED\n";

    // 冷却结束后应能再次触发
    if (feedBump(400, 0.0)) {
        return 1;
    }

    const auto bump2 = feedBump(700, 3.2);

    if (!bump2 ||
        bump2->type != TurnBumpEventType::Bump) {

        std::cerr << "Bump re-trigger FAILED\n";
        return 1;
    }

    // 未达到幅度阈值，不触发
    if (feedBump(800, 0.0) ||
        feedBump(900, 1.8)) {

        std::cerr << "Small bump incorrectly detected\n";
        return 1;
    }

    std::cout << "Small impulse rejected\n";

    // 跨过较大的数据缺口，只更新基线
    if (feedBump(1500, 0.0)) {
        return 1;
    }

    // 负方向冲击同样可以检测
    const auto negative_bump = feedBump(1600, -3.4);

    if (!negative_bump ||
        negative_bump->type != TurnBumpEventType::Bump) {

        std::cerr << "Negative bump FAILED\n";
        return 1;
    }

    std::cout << "Negative bump detected\n";

    // 再验证长时间数据中断
    if (feedBump(3000, 4.0)) {
        std::cerr << "Gap incorrectly caused bump\n";
        return 1;
    }

    std::cout << "Bump gap handling PASSED\n";

    std::cout << "Turn and bump detector PASSED\n";

    return 0;
}
