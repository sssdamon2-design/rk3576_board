
#include <cmath>
#include <iostream>

#include "motion/motion_filter.h"

int main()
{
    using rk3576::motion::MotionFilter;
    using rk3576::sensor::SensorData;

    constexpr double eps = 1e-6;

    MotionFilter filter(0.2);

    SensorData input;
    SensorData output;

    // 1. 第一组：加速度为 0
    input.sample_id = 1;
    input.timestamp_ns = 1000000000ULL;
    input.ax = 0.0;
    input.gx = 0.0;

    if (!filter.process(input, output)) {
        return 1;
    }

    if (std::abs(output.ax) > eps) {
        return 1;
    }

    // 2. 第二组：100ms 后加速度阶跃到 10
    input.sample_id = 2;
    input.timestamp_ns = 1100000000ULL;
    input.ax = 10.0;
    input.gx = 2.0;

    if (!filter.process(input, output)) {
        return 1;
    }

    // tau=0.2s，dt=0.1s
    const double alpha =
        1.0 - std::exp(-0.1 / 0.2);

    const double expected_ax = 10.0 * alpha;
    const double expected_gx = 2.0 * alpha;

    std::cout << "Alpha: " << alpha << '\n';
    std::cout << "Filtered ax: " << output.ax << '\n';
    std::cout << "Expected ax: " << expected_ax << '\n';

    if (std::abs(output.ax - expected_ax) > eps ||
        std::abs(output.gx - expected_gx) > eps ||
        output.sample_id != 2 ||
        output.timestamp_ns != 1100000000ULL) {

        std::cerr << "Step response FAILED\n";
        return 1;
    }

    // 3. 第三组：输入继续保持 10
    input.sample_id = 3;
    input.timestamp_ns = 1200000000ULL;

    if (!filter.process(input, output)) {
        return 1;
    }

    std::cout << "Next filtered ax: "
              << output.ax << '\n';

    // 应逐渐接近 10，但不能超过 10
    if (output.ax <= expected_ax ||
        output.ax >= 10.0) {

        std::cerr << "Filter convergence FAILED\n";
        return 1;
    }

    // 4. 重复时间戳应该被拒绝
    if (filter.process(input, output)) {
        std::cerr << "Duplicate timestamp accepted\n";
        return 1;
    }

    // 5. reset 后，第一组样本应直接通过
    filter.reset();

    input.sample_id = 1;
    input.timestamp_ns = 2000000000ULL;
    input.ax = 7.0;

    if (!filter.process(input, output) ||
        std::abs(output.ax - 7.0) > eps) {

        std::cerr << "Filter reset FAILED\n";
        return 1;
    }

    // 6. 非法时间常数应该被拒绝
    MotionFilter invalid_filter(0.0);

    if (invalid_filter.process(input, output)) {
        std::cerr << "Invalid tau accepted\n";
        return 1;
    }

    std::cout << "Motion filter test PASSED\n";

    return 0;
}
