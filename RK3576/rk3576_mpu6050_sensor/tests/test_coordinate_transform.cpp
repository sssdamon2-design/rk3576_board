
#include <cmath>
#include <iostream>

#include "motion/coordinate_transform.h"

int main()
{
    using rk3576::motion::CoordinateTransform;
    using rk3576::sensor::SensorData;

    // 根据六面实验推断的传感器 → 模块坐标映射
    const CoordinateTransform::Matrix3 sensor_to_module{{
        {{ 0.0,  1.0, 0.0 }},
        {{-1.0,  0.0, 0.0 }},
        {{ 0.0,  0.0, 1.0 }}
    }};

    CoordinateTransform transform(sensor_to_module);

    // 创建一组已知输入
    SensorData input;

    input.ax = 1.0;
    input.ay = 2.0;
    input.az = 3.0;

    input.gx = 0.1;
    input.gy = 0.2;
    input.gz = 0.3;

    input.sample_id = 7;
    input.timestamp_ns = 123456789;

    // 执行坐标转换
    const auto output = transform.transform(input);

    std::cout << "Input accel: "
              << input.ax << ", "
              << input.ay << ", "
              << input.az << '\n';

    std::cout << "Output accel: "
              << output.ax << ", "
              << output.ay << ", "
              << output.az << '\n';

    std::cout << "Output gyro: "
              << output.gx << ", "
              << output.gy << ", "
              << output.gz << '\n';

    constexpr double eps = 1e-9;

    const auto close = [](double a, double b) {
        return std::abs(a - b) < eps;
    };

    // 预期输出：
    // X' = Y, Y' = -X, Z' = Z
    if (!close(output.ax, 2.0) ||
        !close(output.ay, -1.0) ||
        !close(output.az, 3.0) ||
        !close(output.gx, 0.2) ||
        !close(output.gy, -0.1) ||
        !close(output.gz, 0.3) ||
        output.sample_id != input.sample_id ||
        output.timestamp_ns != input.timestamp_ns) {

        std::cerr << "Coordinate transform FAILED\n";
        return 1;
    }

    // 真正的旋转不应改变向量模长
    const double input_norm = std::sqrt(
        input.ax * input.ax +
        input.ay * input.ay +
        input.az * input.az
    );

    const double output_norm = std::sqrt(
        output.ax * output.ax +
        output.ay * output.ay +
        output.az * output.az
    );

    if (!close(input_norm, output_norm)) {
        std::cerr << "Vector norm changed\n";
        return 1;
    }

    std::cout << "Coordinate transform PASSED\n";

    return 0;
}
