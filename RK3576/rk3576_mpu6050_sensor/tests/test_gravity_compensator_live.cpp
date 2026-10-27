
#include <array>
#include <chrono>
#include <cmath>
#include <iostream>
#include <thread>

#include "sensor/mpu6050_reader.h"
#include "motion/sensor_calibration.h"
#include "motion/motion_filter.h"
#include "motion/gravity_compensator.h"

int main()
{
    using namespace std::chrono_literals;

    using rk3576::sensor::MPU6050Reader;
    using rk3576::sensor::SensorData;

    using rk3576::motion::SensorCalibration;
    using rk3576::motion::AccelCalibration;
    using rk3576::motion::GyroBias;
    using rk3576::motion::MotionFilter;
    using rk3576::motion::GravityCompensator;

    MPU6050Reader reader;

    if (!reader.initialize()) {
        return 1;
    }

    // 4.1 实机获得的校准参数
    SensorCalibration calibration;

    calibration.setAccelCalibration(
        AccelCalibration{
            0.212559,
            -0.144483,
            1.515511,
            1.001213,
            1.006555,
            0.978363
        }
    );

    calibration.setGyroBias(
        GyroBias{
            -0.0382023,
            -0.0129790,
            0.0148742
        }
    );

    MotionFilter filter(0.2);

    // 前 50 组样本估计静态基线
    GravityCompensator compensator(50);

    auto next_time =
        std::chrono::steady_clock::now();

    std::array<double, 3> motion_sum{};

    std::cout
        << "Keep MPU6050 stationary for entire test\n";

    // 共采集 100 组，约 10 秒
    for (int i = 0; i < 100; ++i) {

        SensorData raw;
        SensorData filtered;

        if (!reader.read(raw)) {
            std::cerr << "Sensor read failed\n";
            return 1;
        }

        // 先完成加速度和角速度软件校准
        const auto accel_corrected =
            calibration.correctAccel(raw);

        const auto calibrated =
            calibration.correctGyro(accel_corrected);

        // 再执行低通滤波
        if (!filter.process(calibrated, filtered)) {
            std::cerr << "Filter failed\n";
            return 1;
        }

        if (i < 50) {

            // 前 50 组：估计静态基线
            if (!compensator.addStationarySample(filtered)) {
                return 1;
            }

        } else {

            // 后 50 组：独立验证补偿
            SensorData motion;

            if (!compensator.compensate(filtered, motion)) {
                return 1;
            }

            motion_sum[0] += motion.ax;
            motion_sum[1] += motion.ay;
            motion_sum[2] += motion.az;

            if ((i - 50) % 10 == 0) {
                std::cout
                    << "[Motion] sample=" << raw.sample_id
                    << " ax=" << motion.ax
                    << " ay=" << motion.ay
                    << " az=" << motion.az
                    << '\n';
            }
        }

        next_time += 100ms;
        std::this_thread::sleep_until(next_time);
    }

    std::array<double, 3> baseline{};

    if (!compensator.getBaseline(baseline)) {
        return 1;
    }

    const double mean_x = motion_sum[0] / 50.0;
    const double mean_y = motion_sum[1] / 50.0;
    const double mean_z = motion_sum[2] / 50.0;

    const double residual_norm = std::sqrt(
        mean_x * mean_x +
        mean_y * mean_y +
        mean_z * mean_z
    );

    std::cout << "\n=== Gravity Compensation ===\n";

    std::cout << "Baseline XYZ: "
              << baseline[0] << ", "
              << baseline[1] << ", "
              << baseline[2] << '\n';

    std::cout << "Mean motion XYZ: "
              << mean_x << ", "
              << mean_y << ", "
              << mean_z << '\n';

    std::cout << "Residual norm(m/s^2): "
              << residual_norm << '\n';

    // 静止情况下，动态加速度均值应接近零
    if (residual_norm > 0.3) {
        std::cerr
            << "Gravity compensation verification FAILED\n";
        return 1;
    }

    std::cout
        << "Gravity compensation verification PASSED\n";

    return 0;
}
