#pragma once

#include <cstdint>

namespace rk3576::sensor {

// 保存一次 MPU6050 六轴数据
struct SensorData {

    // 三轴加速度，单位 m/s²
    double ax = 0.0;
    double ay = 0.0;
    double az = 0.0;

    // 三轴角速度，单位 rad/s
    double gx = 0.0;
    double gy = 0.0;
    double gz = 0.0;

    // 用户空间采集时间戳，单位 ns
    std::uint64_t timestamp_ns = 0;

    // 成功采集的数据序号
    std::uint64_t sample_id = 0;
};

} // namespace rk3576::sensor
