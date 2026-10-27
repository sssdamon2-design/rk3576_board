
#pragma once

#include <cstdint>
#include <filesystem>

#include "sensor/sensor_data.h"

namespace rk3576::sensor {

class MPU6050Reader {
public:
    // 初始化：发现设备并读取量程换算系数
    bool initialize();

    // 读取一次六轴数据
    bool read(SensorData& data);

    // 获取 IIO 设备路径
    const std::filesystem::path& devicePath() const;

private:
    // 当前找到的 IIO 设备目录
    std::filesystem::path device_path_;

    // 加速度与角速度换算系数
    double accel_scale_ = 0.0;
    double gyro_scale_ = 0.0;

    // 成功采集次数
    std::uint64_t sample_id_ = 0;
};

} // namespace rk3576::sensor

