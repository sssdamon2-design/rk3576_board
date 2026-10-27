
#pragma once

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <stdexcept>
#include <vector>

#include "sensor/sensor_data.h"

#include "motion/sensor_calibration.h"
#include "motion/coordinate_transform.h"
#include "motion/motion_filter.h"
#include "motion/gravity_compensator.h"
#include "motion/longitudinal_motion_detector.h"
#include "motion/turn_bump_detector.h"

namespace rk3576::motion {

// 统一事件类型
enum class UnifiedEventType {
    HarshAcceleration,
    HarshBraking,
    SharpLeftTurn,
    SharpRightTurn,
    Bump
};

// 统一输出的事件
struct UnifiedMotionEvent {
    UnifiedEventType type;

    std::uint64_t timestamp_ns = 0;
    std::uint64_t sample_id = 0;

    double primary_value = 0.0;
    double secondary_value = 0.0;
};

// 一次样本处理结果
struct MotionProcessResult {
    // 静态基线是否准备完成
    bool ready = false;

    // 平滑后的动态加速度
    rk3576::sensor::SensorData smooth_motion{};

    // 未经低通的动态加速度
    rk3576::sensor::SensorData fast_motion{};

    // 一组样本可能产生多个不同类型事件
    std::vector<UnifiedMotionEvent> events;
};

// 统一检测器配置
struct MotionDetectorConfig {
    AccelCalibration accel{};
    GyroBias gyro{};

    // 默认单位矩阵：不改变坐标轴
    CoordinateTransform::Matrix3 sensor_to_vehicle{{
        {{1.0, 0.0, 0.0}},
        {{0.0, 1.0, 0.0}},
        {{0.0, 0.0, 1.0}}
    }};

    std::size_t baseline_samples = 50;

    // 平滑通路的时间常数
    double filter_tau_seconds = 0.1;
};

class MotionDetector {
public:
    explicit MotionDetector(
        const MotionDetectorConfig& config = {})
        : config_(config),
          transform_(config.sensor_to_vehicle),
          filter_(config.filter_tau_seconds),
          gravity_(config.baseline_samples)
    {
        if (config.baseline_samples == 0 ||
            !std::isfinite(config.filter_tau_seconds) ||
            config.filter_tau_seconds <= 0.0) {
            throw std::invalid_argument(
                "Invalid MotionDetector config");
        }

        calibration_.setAccelCalibration(config.accel);
        calibration_.setGyroBias(config.gyro);
    }

    // 每输入一个新样本，运行完整检测流水线
    // nullopt 表示输入无效或时间戳异常
    std::optional<MotionProcessResult> process(
        const rk3576::sensor::SensorData& raw)
    {
        if (!valid(raw) || raw.timestamp_ns == 0) {
            return std::nullopt;
        }

        // 1. 加速度计校准
        const auto accel_corrected =
            calibration_.correctAccel(raw);

        // 2. 陀螺仪校准
        const auto calibrated =
            calibration_.correctGyro(accel_corrected);

        // 3. 坐标系转换
        const auto aligned =
            transform_.transform(calibrated);

        // 4. 生成平滑通路
        rk3576::sensor::SensorData filtered;

        if (!filter_.process(aligned, filtered)) {
            return std::nullopt;
        }

        MotionProcessResult result;

        // 5. 前 N 组样本建立静态基线
        if (!gravity_.ready()) {
            if (!gravity_.addStationarySample(filtered)) {
                return std::nullopt;
            }

            result.ready = gravity_.ready();
            return result;
        }

        // 6. 对平滑数据进行静态基线补偿
        if (!gravity_.compensate(
                filtered, result.smooth_motion)) {
            return std::nullopt;
        }

        // 7. 对未低通的数据执行相同基线补偿
        if (!gravity_.compensate(
                aligned, result.fast_motion)) {
            return std::nullopt;
        }

        result.ready = true;

        // 8. 急加速 / 急刹车检测
        const auto longitudinal_event =
            longitudinal_.process(result.smooth_motion);

        if (longitudinal_event) {
            const auto& e = *longitudinal_event;

            const auto type =
                e.type == MotionEventType::HarshAcceleration
                ? UnifiedEventType::HarshAcceleration
                : UnifiedEventType::HarshBraking;

            result.events.push_back({
                type,
                e.timestamp_ns,
                e.sample_id,
                e.longitudinal_accel,
                0.0
            });
        }

        // 9. 左急转 / 右急转检测
        const auto turn_event =
            turn_.process(result.smooth_motion);

        if (turn_event) {
            const auto& e = *turn_event;

            const auto type =
                e.type == TurnBumpEventType::SharpLeftTurn
                ? UnifiedEventType::SharpLeftTurn
                : UnifiedEventType::SharpRightTurn;

            result.events.push_back({
                type,
                e.timestamp_ns,
                e.sample_id,
                e.primary_value,
                e.secondary_value
            });
        }

        // 10. 颠簸检测：使用未经低通的数据
        const auto bump_event =
            bump_.process(result.fast_motion);

        if (bump_event) {
            const auto& e = *bump_event;

            result.events.push_back({
                UnifiedEventType::Bump,
                e.timestamp_ns,
                e.sample_id,
                e.primary_value,
                e.secondary_value
            });
        }

        return result;
    }

    bool ready() const
    {
        return gravity_.ready();
    }

    // 查询当前基线
    bool getBaseline(std::array<double, 3>& output) const
    {
        return gravity_.getBaseline(output);
    }

    // 重新开始一段新的采样会话
    void reset()
    {
        filter_.reset();
        gravity_.reset();
        longitudinal_.reset();
        turn_.reset();
        bump_.reset();
    }

private:
    static bool valid(
        const rk3576::sensor::SensorData& d)
    {
        return std::isfinite(d.ax) &&
               std::isfinite(d.ay) &&
               std::isfinite(d.az) &&
               std::isfinite(d.gx) &&
               std::isfinite(d.gy) &&
               std::isfinite(d.gz);
    }

    MotionDetectorConfig config_;

    SensorCalibration calibration_;
    CoordinateTransform transform_;
    MotionFilter filter_;
    GravityCompensator gravity_;

    LongitudinalMotionDetector longitudinal_;
    TurnDetector turn_;
    BumpDetector bump_;
};

} // namespace rk3576::motion
