
#include <chrono>
#include <cmath>
#include <iostream>
#include <thread>

#include "sensor/mpu6050_reader.h"
#include "motion/sensor_calibration.h"

int main()
{
    using namespace std::chrono_literals;
    using rk3576::sensor::MPU6050Reader;
    using rk3576::sensor::SensorData;
    using rk3576::motion::GyroBias;
    using rk3576::motion::SensorCalibration;

    MPU6050Reader reader;

    if (!reader.initialize()) {
        return 1;
    }

    constexpr int N = 100;

    GyroBias bias;

    std::cout << "Keep sensor stationary: calibration\n";

    // 第一阶段：采集 100 组，估计零偏
    for (int i = 0; i < N; ++i) {
        SensorData data;

        if (!reader.read(data)) {
            return 1;
        }

        bias.gx += data.gx;
        bias.gy += data.gy;
        bias.gz += data.gz;

        std::this_thread::sleep_for(100ms);
    }

    bias.gx /= N;
    bias.gy /= N;
    bias.gz /= N;

    SensorCalibration calibration;
    calibration.setGyroBias(bias);

    std::cout << "Estimated bias(rad/s): "
              << bias.gx << ", "
              << bias.gy << ", "
              << bias.gz << '\n';

    // 第二阶段：使用新的 100 组样本独立验证
    double corrected_gx = 0.0;
    double corrected_gy = 0.0;
    double corrected_gz = 0.0;

    std::cout << "Keep stationary: verification\n";

    for (int i = 0; i < N; ++i) {
        SensorData raw;

        if (!reader.read(raw)) {
            return 1;
        }

        const auto corrected =
            calibration.correctGyro(raw);

        corrected_gx += corrected.gx;
        corrected_gy += corrected.gy;
        corrected_gz += corrected.gz;

        std::this_thread::sleep_for(100ms);
    }

    corrected_gx /= N;
    corrected_gy /= N;
    corrected_gz /= N;

    const double residual_norm = std::sqrt(
        corrected_gx * corrected_gx +
        corrected_gy * corrected_gy +
        corrected_gz * corrected_gz
    );

    std::cout << "\n=== Gyro Calibration Result ===\n";

    std::cout << "Corrected mean(rad/s): "
              << corrected_gx << ", "
              << corrected_gy << ", "
              << corrected_gz << '\n';

    std::cout << "Residual norm(rad/s): "
              << residual_norm << '\n';

    std::cout << "Gyro calibration completed\n";

    return 0;
}
