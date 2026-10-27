
#include <array>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <thread>

#include "sensor/mpu6050_reader.h"

int main()
{
    using namespace std::chrono_literals;
    using rk3576::sensor::MPU6050Reader;
    using rk3576::sensor::SensorData;

    // 1. 初始化 MPU6050
    MPU6050Reader reader;

    if (!reader.initialize()) {
        std::cerr << "MPU6050 initialization failed\n";
        return 1;
    }

    constexpr std::size_t N = 100;

    // 六轴均值
    std::array<double, 6> mean{};

    // 用于计算标准差的累计量
    std::array<double, 6> m2{};

    const std::array<const char*, 6> names = {
        "Accel X",
        "Accel Y",
        "Accel Z",
        "Gyro X",
        "Gyro Y",
        "Gyro Z"
    };

    std::cout << "Keep MPU6050 stationary...\n";

    auto next_time = std::chrono::steady_clock::now();

    // 2. 连续采集 100 组数据
    for (std::size_t i = 0; i < N; ++i) {

        SensorData data;

        if (!reader.read(data)) {
            std::cerr << "Read failed at sample "
                      << i + 1 << '\n';
            return 1;
        }

        const std::array<double, 6> values = {
            data.ax, data.ay, data.az,
            data.gx, data.gy, data.gz
        };

        // 3. 递推计算均值和标准差
        for (std::size_t j = 0; j < 6; ++j) {

            const double delta = values[j] - mean[j];

            mean[j] += delta / static_cast<double>(i + 1);

            const double delta2 = values[j] - mean[j];

            m2[j] += delta * delta2;
        }

        next_time += 100ms;
        std::this_thread::sleep_until(next_time);
    }

    // 4. 计算平均加速度向量模长
    const double accel_norm = std::sqrt(
        mean[0] * mean[0] +
        mean[1] * mean[1] +
        mean[2] * mean[2]
    );

    // 5. 计算平均角速度向量模长
    const double gyro_norm = std::sqrt(
        mean[3] * mean[3] +
        mean[4] * mean[4] +
        mean[5] * mean[5]
    );

    constexpr double gravity = 9.80665;

    std::cout << std::fixed
              << std::setprecision(6);

    std::cout << "\n=== Static Baseline ===\n";

    std::cout << "Samples: " << N << '\n';

    // 6. 输出六轴均值和标准差
    for (std::size_t j = 0; j < 6; ++j) {

        const double stddev = std::sqrt(
            m2[j] / static_cast<double>(N - 1)
        );

        std::cout
            << names[j]
            << " mean=" << mean[j]
            << " stddev=" << stddev
            << '\n';
    }

    std::cout << "Accel norm(m/s^2): "
              << accel_norm << '\n';

    std::cout << "Gravity reference(m/s^2): "
              << gravity << '\n';

    std::cout << "Accel norm error(m/s^2): "
              << accel_norm - gravity << '\n';

    std::cout << "Gyro norm(rad/s): "
              << gyro_norm << '\n';

    std::cout << "Static baseline completed\n";

    return 0;
}
