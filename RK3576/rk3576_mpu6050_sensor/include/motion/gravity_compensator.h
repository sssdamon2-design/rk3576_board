
#pragma once

#include <array>
#include <cmath>
#include <cstddef>

#include "sensor/sensor_data.h"

namespace rk3576::motion {

class GravityCompensator {
public:
    // 使用多少组静态样本估计重力基线
    explicit GravityCompensator(
        std::size_t required_samples = 50)
        : required_samples_(required_samples)
    {
    }

    // 添加一组静态样本
    bool addStationarySample(
        const rk3576::sensor::SensorData& data)
    {
        if (required_samples_ == 0 ||
            ready() ||
            !valid(data)) {
            return false;
        }

        sums_[0] += data.ax;
        sums_[1] += data.ay;
        sums_[2] += data.az;

        ++sample_count_;

        return true;
    }

    // 是否已经完成静态基线估计
    bool ready() const
    {
        return required_samples_ > 0 &&
               sample_count_ == required_samples_;
    }

    // 获取已计算的静态基线
    bool getBaseline(std::array<double, 3>& baseline) const
    {
        if (!ready()) {
            return false;
        }

        for (std::size_t i = 0; i < 3; ++i) {
            baseline[i] =
                sums_[i] /
                static_cast<double>(sample_count_);
        }

        return true;
    }

    // 从当前加速度中减去静态基线
    bool compensate(
        const rk3576::sensor::SensorData& input,
        rk3576::sensor::SensorData& output) const
    {
        if (!ready() || !valid(input)) {
            return false;
        }

        std::array<double, 3> baseline{};

        if (!getBaseline(baseline)) {
            return false;
        }

        auto result = input;

        result.ax -= baseline[0];
        result.ay -= baseline[1];
        result.az -= baseline[2];

        // 角速度、时间戳和样本序号不变
        output = result;

        return true;
    }

    // 重置基线，用于重新标定
    void reset()
    {
        sums_ = {};
        sample_count_ = 0;
    }

private:
    static bool valid(
        const rk3576::sensor::SensorData& data)
    {
        return std::isfinite(data.ax) &&
               std::isfinite(data.ay) &&
               std::isfinite(data.az);
    }

    std::size_t required_samples_;

    std::size_t sample_count_ = 0;

    std::array<double, 3> sums_{};
};

} // namespace rk3576::motion
