
#pragma once

#include <cmath>
#include <cstdint>
#include <optional>
#include <stdexcept>

#include "sensor/sensor_data.h"

namespace rk3576::motion {

// 纵向运动事件类型
enum class MotionEventType {
    HarshAcceleration,
    HarshBraking
};

// 一次检测事件的数据
struct MotionEvent {
    MotionEventType type;

    std::uint64_t timestamp_ns = 0;
    std::uint64_t sample_id = 0;

    double longitudinal_accel = 0.0;
};

// 检测器配置
struct LongitudinalDetectorConfig {
    double trigger_mps2 = 2.5;
    double release_mps2 = 1.0;

    // 持续达到阈值至少 200ms
    std::uint64_t min_duration_ns = 200000000ULL;

    // 超过 500ms 没有新样本则重置状态
    std::uint64_t max_gap_ns = 500000000ULL;
};

class LongitudinalMotionDetector {
public:
    explicit LongitudinalMotionDetector(
        LongitudinalDetectorConfig config = {})
        : config_(config)
    {
        if (!std::isfinite(config_.trigger_mps2) ||
            !std::isfinite(config_.release_mps2) ||
            config_.trigger_mps2 <= 0.0 ||
            config_.release_mps2 < 0.0 ||
            config_.release_mps2 >= config_.trigger_mps2 ||
            config_.min_duration_ns == 0 ||
            config_.max_gap_ns == 0) {

            throw std::invalid_argument(
                "Invalid motion detector configuration");
        }
    }

    // 每输入一组车辆坐标系下的动态加速度
    // 可能返回一个新事件，也可能没有事件
    std::optional<MotionEvent> process(
        const rk3576::sensor::SensorData& data)
    {
        const auto ts = data.timestamp_ns;
        const double ax = data.ax;

        if (ts == 0 || !std::isfinite(ax)) {
            return std::nullopt;
        }

        // 防止重复或倒退时间戳
        if (last_timestamp_ns_ != 0) {
            if (ts <= last_timestamp_ns_) {
                return std::nullopt;
            }

            // 数据间隔太长，不能认为运动连续
            if (ts - last_timestamp_ns_ >
                config_.max_gap_ns) {
                state_ = State::Idle;
            }
        }

        last_timestamp_ns_ = ts;

        // 已经触发急加速，避免连续重复报警
        if (state_ == State::AccelerationActive) {
            if (ax > config_.release_mps2) {
                return std::nullopt;
            }

            state_ = State::Idle;
        }

        // 已经触发急刹车，避免连续重复报警
        if (state_ == State::BrakingActive) {
            if (ax < -config_.release_mps2) {
                return std::nullopt;
            }

            state_ = State::Idle;
        }

        // 急加速候选
        if (ax >= config_.trigger_mps2) {

            if (state_ != State::AccelerationCandidate) {
                state_ = State::AccelerationCandidate;
                candidate_since_ns_ = ts;
                return std::nullopt;
            }

            if (ts - candidate_since_ns_ >=
                config_.min_duration_ns) {

                state_ = State::AccelerationActive;

                return MotionEvent{
                    MotionEventType::HarshAcceleration,
                    ts,
                    data.sample_id,
                    ax
                };
            }

            return std::nullopt;
        }

        // 急刹车候选
        if (ax <= -config_.trigger_mps2) {

            if (state_ != State::BrakingCandidate) {
                state_ = State::BrakingCandidate;
                candidate_since_ns_ = ts;
                return std::nullopt;
            }

            if (ts - candidate_since_ns_ >=
                config_.min_duration_ns) {

                state_ = State::BrakingActive;

                return MotionEvent{
                    MotionEventType::HarshBraking,
                    ts,
                    data.sample_id,
                    ax
                };
            }

            return std::nullopt;
        }

        // 未达到检测阈值，清除候选状态
        state_ = State::Idle;

        return std::nullopt;
    }

    // 清空检测历史
    void reset()
    {
        state_ = State::Idle;
        candidate_since_ns_ = 0;
        last_timestamp_ns_ = 0;
    }

private:
    enum class State {
        Idle,
        AccelerationCandidate,
        BrakingCandidate,
        AccelerationActive,
        BrakingActive
    };

    LongitudinalDetectorConfig config_;

    State state_ = State::Idle;

    std::uint64_t candidate_since_ns_ = 0;
    std::uint64_t last_timestamp_ns_ = 0;
};

} // namespace rk3576::motion
