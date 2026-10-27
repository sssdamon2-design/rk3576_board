
#pragma once

#include <cmath>
#include <cstdint>
#include <optional>
#include <stdexcept>

#include "sensor/sensor_data.h"

namespace rk3576::motion {

// 事件类型
enum class TurnBumpEventType {
    SharpLeftTurn,
    SharpRightTurn,
    Bump
};

// 事件数据
struct TurnBumpEvent {
    TurnBumpEventType type;

    std::uint64_t timestamp_ns = 0;
    std::uint64_t sample_id = 0;

    // 转弯：角速度；颠簸：垂向加速度
    double primary_value = 0.0;

    // 转弯：横向加速度；颠簸：垂向加速度变化量
    double secondary_value = 0.0;
};

// ------------------------------
// 一、急转弯检测
// ------------------------------

struct TurnDetectorConfig {
    double trigger_yaw = 0.35;
    double trigger_lateral = 1.5;

    double release_yaw = 0.15;
    double release_lateral = 0.6;

    std::uint64_t min_duration_ns = 200000000ULL;
    std::uint64_t max_gap_ns = 500000000ULL;
};

class TurnDetector {
public:
    explicit TurnDetector(TurnDetectorConfig config = {})
        : config_(config)
    {
        if (!std::isfinite(config_.trigger_yaw) ||
            !std::isfinite(config_.trigger_lateral) ||
            !std::isfinite(config_.release_yaw) ||
            !std::isfinite(config_.release_lateral) ||
            config_.trigger_yaw <= config_.release_yaw ||
            config_.trigger_lateral <= config_.release_lateral ||
            config_.release_yaw < 0.0 ||
            config_.release_lateral < 0.0 ||
            config_.min_duration_ns == 0 ||
            config_.max_gap_ns == 0) {
            throw std::invalid_argument(
                "Invalid turn detector config");
        }
    }

    std::optional<TurnBumpEvent> process(
        const rk3576::sensor::SensorData& data)
    {
        const auto ts = data.timestamp_ns;

        if (ts == 0 ||
            !std::isfinite(data.ay) ||
            !std::isfinite(data.gz)) {
            return std::nullopt;
        }

        if (last_timestamp_ns_ != 0) {
            if (ts <= last_timestamp_ns_) {
                return std::nullopt;
            }

            if (ts - last_timestamp_ns_ >
                config_.max_gap_ns) {
                state_ = State::Idle;
            }
        }

        last_timestamp_ns_ = ts;

        // 判断新的转弯候选方向
        int direction = 0;

        if (std::abs(data.gz) >= config_.trigger_yaw &&
            std::abs(data.ay) >= config_.trigger_lateral) {

            if (data.gz > 0.0 && data.ay > 0.0) {
                direction = 1;
            } else if (data.gz < 0.0 && data.ay < 0.0) {
                direction = -1;
            }
        }

        // 左转已激活：需要低于释放条件才退出
        if (state_ == State::LeftActive) {
            if (data.gz > config_.release_yaw &&
                data.ay > config_.release_lateral) {
                return std::nullopt;
            }

            state_ = State::Idle;
        }

        // 右转已激活
        if (state_ == State::RightActive) {
            if (data.gz < -config_.release_yaw &&
                data.ay < -config_.release_lateral) {
                return std::nullopt;
            }

            state_ = State::Idle;
        }

        if (direction == 0) {
            state_ = State::Idle;
            return std::nullopt;
        }

        const State candidate =
            (direction > 0) ?
            State::LeftCandidate :
            State::RightCandidate;

        // 第一次达到条件，进入候选状态
        if (state_ != candidate) {
            state_ = candidate;
            candidate_since_ns_ = ts;
            return std::nullopt;
        }

        // 还未满足持续时间
        if (ts - candidate_since_ns_ <
            config_.min_duration_ns) {
            return std::nullopt;
        }

        // 正式触发一次急转弯事件
        state_ = (direction > 0) ?
            State::LeftActive :
            State::RightActive;

        return TurnBumpEvent{
            (direction > 0) ?
                TurnBumpEventType::SharpLeftTurn :
                TurnBumpEventType::SharpRightTurn,
            ts,
            data.sample_id,
            data.gz,
            data.ay
        };
    }

    void reset()
    {
        state_ = State::Idle;
        candidate_since_ns_ = 0;
        last_timestamp_ns_ = 0;
    }

private:
    enum class State {
        Idle,
        LeftCandidate,
        RightCandidate,
        LeftActive,
        RightActive
    };

    TurnDetectorConfig config_;

    State state_ = State::Idle;

    std::uint64_t candidate_since_ns_ = 0;
    std::uint64_t last_timestamp_ns_ = 0;
};

// ------------------------------
// 二、路面颠簸检测
// ------------------------------

struct BumpDetectorConfig {
    double trigger_vertical = 2.5;
    double trigger_delta = 1.5;

    std::uint64_t cooldown_ns = 500000000ULL;
    std::uint64_t max_gap_ns = 500000000ULL;
};

class BumpDetector {
public:
    explicit BumpDetector(BumpDetectorConfig config = {})
        : config_(config)
    {
        if (!std::isfinite(config_.trigger_vertical) ||
            !std::isfinite(config_.trigger_delta) ||
            config_.trigger_vertical <= 0.0 ||
            config_.trigger_delta <= 0.0 ||
            config_.cooldown_ns == 0 ||
            config_.max_gap_ns == 0) {
            throw std::invalid_argument(
                "Invalid bump detector config");
        }
    }

    std::optional<TurnBumpEvent> process(
        const rk3576::sensor::SensorData& data)
    {
        const auto ts = data.timestamp_ns;
        const double az = data.az;

        if (ts == 0 || !std::isfinite(az)) {
            return std::nullopt;
        }

        // 第一次输入只建立上一样本基线
        if (!initialized_) {
            initialized_ = true;
            last_timestamp_ns_ = ts;
            last_az_ = az;
            return std::nullopt;
        }

        // 拒绝重复或倒退时间戳
        if (ts <= last_timestamp_ns_) {
            return std::nullopt;
        }

        const auto gap = ts - last_timestamp_ns_;
        const double delta_az = az - last_az_;

        last_timestamp_ns_ = ts;
        last_az_ = az;

        // 数据中断太久，不跨越数据缺口检测冲击
        if (gap > config_.max_gap_ns) {
            return std::nullopt;
        }

        // 垂向加速度幅度检查
        if (std::abs(az) < config_.trigger_vertical) {
            return std::nullopt;
        }

        // 相邻样本变化量检查
        if (std::abs(delta_az) < config_.trigger_delta) {
            return std::nullopt;
        }

        // 抑制同一段时间内重复报警
        if (last_event_ns_ != 0 &&
            ts - last_event_ns_ < config_.cooldown_ns) {
            return std::nullopt;
        }

        last_event_ns_ = ts;

        return TurnBumpEvent{
            TurnBumpEventType::Bump,
            ts,
            data.sample_id,
            az,
            delta_az
        };
    }

    void reset()
    {
        initialized_ = false;
        last_timestamp_ns_ = 0;
        last_event_ns_ = 0;
        last_az_ = 0.0;
    }

private:
    BumpDetectorConfig config_;

    bool initialized_ = false;

    std::uint64_t last_timestamp_ns_ = 0;
    std::uint64_t last_event_ns_ = 0;

    double last_az_ = 0.0;
};

} // namespace rk3576::motion
