
#pragma once

#include <cstdint>

namespace rk3576::sensor {

struct SensorStats {
    // 读取次数统计
    std::uint64_t read_attempts = 0;
    std::uint64_t successful_samples = 0;
    std::uint64_t failed_reads = 0;

    // 实测采样率，单位 Hz
    double actual_hz = 0.0;

    // 单次读取耗时，单位 ms
    double avg_read_ms = 0.0;
    double max_read_ms = 0.0;

    // 采样间隔相对目标周期的最大偏差
    double max_period_error_ms = 0.0;

    // 最新数据距今的时间，单位 ms
    double data_age_ms = 0.0;

    // 是否已经获得有效数据
    bool has_data = false;
};

} // namespace rk3576::sensor
