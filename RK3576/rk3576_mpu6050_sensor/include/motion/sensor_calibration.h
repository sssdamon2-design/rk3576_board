
#pragma once

#include "sensor/sensor_data.h"

namespace rk3576::motion {

// 陀螺仪零偏
struct GyroBias {
    double gx = 0.0;
    double gy = 0.0;
    double gz = 0.0;
};

// 加速度计校准参数
struct AccelCalibration {
    // 三轴零偏，单位 m/s²
    double bx = 0.0;
    double by = 0.0;
    double bz = 0.0;

    // 三轴比例修正系数
    double kx = 1.0;
    double ky = 1.0;
    double kz = 1.0;
};

class SensorCalibration {
public:
    // 设置陀螺仪零偏
    void setGyroBias(const GyroBias& bias)
    {
        gyro_bias_ = bias;
    }

    // 设置加速度计标定参数
    void setAccelCalibration(
        const AccelCalibration& calibration)
    {
        accel_calibration_ = calibration;
    }

    // 陀螺仪零偏补偿
    rk3576::sensor::SensorData correctGyro(
        const rk3576::sensor::SensorData& input) const
    {
        auto result = input;

        result.gx -= gyro_bias_.gx;
        result.gy -= gyro_bias_.gy;
        result.gz -= gyro_bias_.gz;

        return result;
    }

    // 加速度计零偏及比例误差补偿
    rk3576::sensor::SensorData correctAccel(
        const rk3576::sensor::SensorData& input) const
    {
        auto result = input;

        result.ax =
            (input.ax - accel_calibration_.bx) *
            accel_calibration_.kx;

        result.ay =
            (input.ay - accel_calibration_.by) *
            accel_calibration_.ky;

        result.az =
            (input.az - accel_calibration_.bz) *
            accel_calibration_.kz;

        return result;
    }

private:
    GyroBias gyro_bias_{};
    AccelCalibration accel_calibration_{};
};

} // namespace rk3576::motion
