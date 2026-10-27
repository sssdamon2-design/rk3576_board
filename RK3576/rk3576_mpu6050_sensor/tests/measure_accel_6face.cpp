
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <iomanip>
#include <iostream>
#include <string>
#include <thread>

#include "sensor/mpu6050_reader.h"

int main()
{
    using namespace std::chrono_literals;
    using rk3576::sensor::MPU6050Reader;
    using rk3576::sensor::SensorData;

    constexpr int N = 100;
    constexpr double G = 9.80665;

    struct Pose {
        const char* name;
        std::size_t axis;
        int sign;
    };

    const std::array<Pose, 6> poses{{
        {"+X upward", 0, +1},
        {"-X upward", 0, -1},
        {"+Y upward", 1, +1},
        {"-Y upward", 1, -1},
        {"+Z upward", 2, +1},
        {"-Z upward", 2, -1}
    }};

    MPU6050Reader reader;

    if (!reader.initialize()) {
        std::cerr << "MPU6050 init failed\n";
        return 1;
    }

    // 保存六个姿态的三轴平均值
    std::array<std::array<double, 3>, 6> means{};

    std::cout << std::fixed
              << std::setprecision(6);

    // 依次测量六个姿态
    for (std::size_t p = 0; p < poses.size(); ++p) {

        const auto& pose = poses[p];

        std::cout << "\nPose " << p + 1
                  << "/6: " << pose.name << '\n';

        std::cout
            << "Place sensor steadily, then press ENTER..."
            << std::flush;

        std::string line;

        if (!std::getline(std::cin, line)) {
            std::cerr << "Input ended\n";
            return 1;
        }

        std::array<double, 3> mean{};
        std::array<double, 3> m2{};

        auto next_time =
            std::chrono::steady_clock::now();

        for (int i = 0; i < N; ++i) {

            SensorData data;

            if (!reader.read(data)) {
                std::cerr << "Read failed\n";
                return 1;
            }

            const std::array<double, 3> value{
                data.ax, data.ay, data.az
            };

            // Welford 在线均值与方差统计
            for (std::size_t j = 0; j < 3; ++j) {

                const double delta =
                    value[j] - mean[j];

                mean[j] +=
                    delta / static_cast<double>(i + 1);

                const double delta2 =
                    value[j] - mean[j];

                m2[j] += delta * delta2;
            }

            next_time += 100ms;
            std::this_thread::sleep_until(next_time);
        }

        means[p] = mean;

        const double axis_stddev = std::sqrt(
            m2[pose.axis] / static_cast<double>(N - 1)
        );

        std::cout
            << "\nMean X=" << mean[0]
            << " Y=" << mean[1]
            << " Z=" << mean[2]
            << " m/s^2\n";

        std::cout
            << "Target axis stddev="
            << axis_stddev << " m/s^2\n";

        if (pose.sign * mean[pose.axis] < 5.0) {
            std::cerr
                << "WARNING: Check pose direction!\n";
        }

        if (axis_stddev > 0.15) {
            std::cerr
                << "WARNING: Sensor may have moved!\n";
        }
    }

    // 使用正负姿态估计每个轴的偏置和比例系数
    std::cout << "\n=== Six-Face Calibration ===\n";

    const std::array<const char*, 3> axes{
        "X", "Y", "Z"
    };

    bool valid = true;

    for (std::size_t axis = 0; axis < 3; ++axis) {

        const double positive =
            means[2 * axis][axis];

        const double negative =
            means[2 * axis + 1][axis];

        const double bias =
            (positive + negative) / 2.0;

        const double half_span =
            (positive - negative) / 2.0;

        if (positive <= 0.0 ||
            negative >= 0.0 ||
            half_span < 5.0) {

            std::cerr
                << axes[axis]
                << ": Invalid pose measurements\n";

            valid = false;
            continue;
        }

        const double gain =
            G / half_span;

        std::cout
            << axes[axis]
            << ": positive=" << positive
            << " negative=" << negative
            << " bias=" << bias
            << " gain=" << gain
            << '\n';
    }

    if (!valid) {
        std::cerr << "Calibration data invalid\n";
        return 1;
    }

    std::cout << "Six-face measurement completed\n";

    return 0;
}
