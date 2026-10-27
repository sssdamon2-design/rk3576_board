
#pragma once

#include <cmath>
#include <cstdint>

#include "sensor/sensor_data.h"

namespace rk3576::motion {

class MotionFilter {
public:
    // tau_seconds：滤波时间常数，单位秒
    explicit MotionFilter(double tau_seconds = 0.2)
        : tau_seconds_(tau_seconds)
    {
    }

    // 输入一组六轴数据，输出滤波后的数据
    bool process(
        const rk3576::sensor::SensorData& input,
        rk3576::sensor::SensorData& output)
    {
        // 1. 检查时间常数和时间戳
        if (!std::isfinite(tau_seconds_) ||
            tau_seconds_ <= 0.0 ||
            input.timestamp_ns == 0) {
            return false;
        }

        // 2. 检查六轴输入是否有效
        if (!std::isfinite(input.ax) ||
            !std::isfinite(input.ay) ||
            !std::isfinite(input.az) ||
            !std::isfinite(input.gx) ||
            !std::isfinite(input.gy) ||
            !std::isfinite(input.gz)) {
            return false;
        }

        // 3. 首次采样：直接初始化滤波器状态
        if (!initialized_) {
            state_ = input;
            last_timestamp_ns_ = input.timestamp_ns;
            initialized_ = true;

            output = state_;
            return true;
        }

        // 4. 时间戳必须严格递增
        if (input.timestamp_ns <= last_timestamp_ns_) {
            return false;
        }

        // 5. 计算实际采样时间间隔，单位秒
        const double dt =
            static_cast<double>(
                input.timestamp_ns - last_timestamp_ns_
            ) * 1e-9;

        // 6. 根据时间间隔计算平滑系数
        const double alpha =
            1.0 - std::exp(-dt / tau_seconds_);

        // 7. 对三个加速度轴进行低通滤波
        state_.ax += alpha * (input.ax - state_.ax);
        state_.ay += alpha * (input.ay - state_.ay);
        state_.az += alpha * (input.az - state_.az);

        // 8. 对三个角速度轴进行低通滤波
        state_.gx += alpha * (input.gx - state_.gx);
        state_.gy += alpha * (input.gy - state_.gy);
        state_.gz += alpha * (input.gz - state_.gz);

        // 9. 保留当前样本的元数据
        state_.timestamp_ns = input.timestamp_ns;
        state_.sample_id = input.sample_id;

        last_timestamp_ns_ = input.timestamp_ns;

        // 10. 返回滤波后的完整样本
        output = state_;

        return true;
    }

    // 清空内部历史状态
    void reset()
    {
        state_ = rk3576::sensor::SensorData{};
        last_timestamp_ns_ = 0;
        initialized_ = false;
    }

private:
    double tau_seconds_;

    rk3576::sensor::SensorData state_{};

    std::uint64_t last_timestamp_ns_ = 0;

    bool initialized_ = false;
};

} // namespace rk3576::motion
