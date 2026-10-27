
#include <array>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <string>
#include <thread>

#include "sensor/mpu6050_reader.h"
#include "motion/sensor_calibration.h"

int main()
{
    using namespace std::chrono_literals;
    using rk3576::sensor::MPU6050Reader;
    using rk3576::sensor::SensorData;
    using rk3576::motion::SensorCalibration;
    using rk3576::motion::AccelCalibration;

    constexpr int N = 100;
    constexpr double G = 9.80665;

    MPU6050Reader reader;

    if (!reader.initialize()) {
        return 1;
    }

    SensorCalibration calibration;

    // 使用上一轮六面标定的实测参数
    AccelCalibration params;

    params.bx = 0.212559;
    params.by = -0.144483;
    params.bz = 1.515511;

    params.kx = 1.001213;
    params.ky = 1.006555;
    params.kz = 0.978363;

    calibration.setAccelCalibration(params);

    const std::array<std::string, 2> poses = {
        "+Z upward",
        "-Z upward"
    };

    bool all_passed = true;

    std::cout << std::fixed
              << std::setprecision(6);

    for (int p = 0; p < 2; ++p) {

        std::cout << "\nPose: " << poses[p] << '\n';

        std::cout
            << "Place sensor steadily, press ENTER..."
            << std::flush;

        std::string line;

        if (!std::getline(std::cin, line)) {
            return 1;
        }

        double raw_x = 0.0;
        double raw_y = 0.0;
        double raw_z = 0.0;

        double corrected_x = 0.0;
        double corrected_y = 0.0;
        double corrected_z = 0.0;

        auto next_time =
            std::chrono::steady_clock::now();

        for (int i = 0; i < N; ++i) {

            SensorData data;

            if (!reader.read(data)) {
                std::cerr << "Read failed\n";
                return 1;
            }

            const auto corrected =
                calibration.correctAccel(data);

            raw_x += data.ax;
            raw_y += data.ay;
            raw_z += data.az;

            corrected_x += corrected.ax;
            corrected_y += corrected.ay;
            corrected_z += corrected.az;

            next_time += 100ms;

            std::this_thread::sleep_until(next_time);
        }

        raw_x /= N;
        raw_y /= N;
        raw_z /= N;

        corrected_x /= N;
        corrected_y /= N;
        corrected_z /= N;

        const double raw_norm = std::sqrt(
            raw_x * raw_x +
            raw_y * raw_y +
            raw_z * raw_z
        );

        const double corrected_norm = std::sqrt(
            corrected_x * corrected_x +
            corrected_y * corrected_y +
            corrected_z * corrected_z
        );

        const double error =
            std::abs(corrected_norm - G);

        std::cout << "\n=== Verification Result ===\n";

        std::cout << "Raw XYZ: "
                  << raw_x << ", "
                  << raw_y << ", "
                  << raw_z << '\n';

        std::cout << "Corrected XYZ: "
                  << corrected_x << ", "
                  << corrected_y << ", "
                  << corrected_z << '\n';

        std::cout << "Raw norm: "
                  << raw_norm << '\n';

        std::cout << "Corrected norm: "
                  << corrected_norm << '\n';

        std::cout << "Gravity reference: "
                  << G << '\n';

        std::cout << "Corrected norm error: "
                  << error << '\n';

        // 本次采用宽松的初步验收阈值
        const double expected_sign =
            (p == 0) ? 1.0 : -1.0;

        if (error > 0.5 ||
            expected_sign * corrected_z < 8.0) {

            std::cerr
                << "Verification FAILED for this pose\n";

            all_passed = false;
        }
    }

    if (!all_passed) {
        std::cerr << "Accel calibration verification FAILED\n";
        return 1;
    }

    std::cout
        << "\nAccel calibration verification PASSED\n";

    return 0;
}
