
#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <functional>
#include <mutex>
#include <thread>

#include "sensor/mpu6050_reader.h"
#include "sensor/sensor_data.h"
#include "sensor/sensor_stats.h"

namespace rk3576::sensor {

class SensorService {
public:
    // 采样回调函数类型
    // 每次成功采集后，将 SensorData 传递给回调函数
    using SampleCallback =
        std::function<void(const SensorData&)>;

    // 构造函数，默认采样周期为 100ms，即 10Hz
    explicit SensorService(
        std::chrono::milliseconds period =
            std::chrono::milliseconds{100});

    // 析构函数，自动停止并回收后台线程
    ~SensorService();

    // 禁止复制，避免线程及资源被错误复制
    SensorService(const SensorService&) = delete;
    SensorService& operator=(const SensorService&) = delete;

    // 启动后台采集服务
    bool start();

    // 停止后台采集服务
    void stop();

    // 设置成功采样回调
    // 只能在 start() 之前，或 stop() 完成之后调用
    // 生命周期函数由同一个主控线程顺序管理
    bool setSampleCallback(SampleCallback callback);

    // 获取成功采样总次数
    std::uint64_t sampleCount() const;

    // 获取最新一组完整六轴数据
    bool getLatest(SensorData& data) const;

    // 获取运行统计快照
    SensorStats stats() const;

private:
    // 后台线程执行函数
    void workerLoop();

    // MPU6050 底层读取对象
    MPU6050Reader reader_;

    // 采样周期
    std::chrono::milliseconds period_;

    // 原子运行标志
    std::atomic<bool> running_{false};

    // 原子成功采样计数
    std::atomic<std::uint64_t> sample_count_{0};

    // 后台采集线程
    std::thread worker_;

    // 新增：成功采样回调
    // 运行期间不允许修改该对象
    SampleCallback sample_callback_{};

    // 保护最新六轴数据与统计信息
    mutable std::mutex latest_mutex_;

    // 最新成功采集的数据
    SensorData latest_data_{};

    // 是否已经存在有效数据
    bool has_latest_ = false;

    // 采样运行统计
    SensorStats stats_{};

    // 首次成功采样时间戳
    std::uint64_t first_success_ns_ = 0;

    // 最近一次成功采样时间戳
    std::uint64_t last_success_ns_ = 0;
};

} // namespace rk3576::sensor
