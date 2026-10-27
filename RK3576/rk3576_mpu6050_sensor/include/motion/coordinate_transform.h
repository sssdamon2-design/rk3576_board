
#pragma once

#include <array>

#include "sensor/sensor_data.h"

namespace rk3576::motion {

class CoordinateTransform {
public:
    // 3×3 坐标旋转矩阵
    using Matrix3 =
        std::array<std::array<double, 3>, 3>;

    // 传入坐标变换矩阵
    explicit CoordinateTransform(const Matrix3& rotation)
        : rotation_(rotation)
    {
    }

    // 转换一组完整六轴数据
    rk3576::sensor::SensorData transform(
        const rk3576::sensor::SensorData& input) const
    {
        auto output = input;

        const auto accel = rotate({
            input.ax, input.ay, input.az
        });

        const auto gyro = rotate({
            input.gx, input.gy, input.gz
        });

        output.ax = accel[0];
        output.ay = accel[1];
        output.az = accel[2];

        output.gx = gyro[0];
        output.gy = gyro[1];
        output.gz = gyro[2];

        return output;
    }

private:
    std::array<double, 3> rotate(
        const std::array<double, 3>& v) const
    {
        std::array<double, 3> result{};

        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                result[i] += rotation_[i][j] * v[j];
            }
        }

        return result;
    }

    Matrix3 rotation_;
};

} // namespace rk3576::motion
