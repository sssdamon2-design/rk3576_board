
#include <array>
#include <cmath>
#include <iostream>

#include "motion/gravity_compensator.h"

int main()
{
    using rk3576::motion::GravityCompensator;
    using rk3576::sensor::SensorData;

    constexpr double eps = 1e-9;

    // 使用两组静态数据测试计算过程
    GravityCompensator compensator(2);

    SensorData data1;
    data1.ax = 0.2;
    data1.ay = -0.1;
    data1.az = 9.8;

    SensorData data2;
    data2.ax = 0.4;
    data2.ay = -0.3;
    data2.az = 9.6;

    SensorData output;

    // 基线完成前不能进行补偿
    if (compensator.compensate(data1, output)) {
        return 1;
    }

    if (!compensator.addStationarySample(data1) ||
        !compensator.addStationarySample(data2) ||
        !compensator.ready()) {
        return 1;
    }

    std::array<double, 3> baseline{};

    if (!compensator.getBaseline(baseline)) {
        return 1;
    }

    // 预期基线：(0.3, -0.2, 9.7)
    std::cout << "Baseline: "
              << baseline[0] << ", "
              << baseline[1] << ", "
              << baseline[2] << '\n';

    if (std::abs(baseline[0] - 0.3) > eps ||
        std::abs(baseline[1] + 0.2) > eps ||
        std::abs(baseline[2] - 9.7) > eps) {
        return 1;
    }

    // 构造一组相对静态基线发生变化的数据
    SensorData moving;
    moving.ax = 2.3;
    moving.ay = -0.2;
    moving.az = 9.7;
    moving.gx = 0.42;
    moving.sample_id = 3;
    moving.timestamp_ns = 123456789;

    if (!compensator.compensate(moving, output)) {
        return 1;
    }

    std::cout << "Motion acceleration: "
              << output.ax << ", "
              << output.ay << ", "
              << output.az << '\n';

    // 预期 (2.0, 0.0, 0.0)
    if (std::abs(output.ax - 2.0) > eps ||
        std::abs(output.ay) > eps ||
        std::abs(output.az) > eps ||
        std::abs(output.gx - 0.42) > eps ||
        output.sample_id != moving.sample_id ||
        output.timestamp_ns != moving.timestamp_ns) {

        std::cerr << "Gravity compensation FAILED\n";
        return 1;
    }

    compensator.reset();

    if (compensator.ready()) {
        std::cerr << "Reset FAILED\n";
        return 1;
    }

    std::cout << "Gravity compensator test PASSED\n";

    return 0;
}
