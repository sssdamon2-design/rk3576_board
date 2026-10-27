
#include "sensor/sensor_service.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <exception>
#include <iostream>
#include <mutex>
#include <system_error>
#include <utility>

namespace rk3576::sensor {

// ============================================================
// 1. 构造函数
// ============================================================

SensorService::SensorService(
    std::chrono::milliseconds period)
    : period_(period)
{
}

// ============================================================
// 2. 析构函数
// ============================================================

SensorService::~SensorService()
{
    // 析构时确保后台线程停止
    stop();
}

// ============================================================
// 3. 启动采集服务
// ============================================================

bool SensorService::start()
{
    // 生命周期由同一个主控线程顺序调用 start/stop

    // 检查是否已经启动，或者旧线程仍需要回收
    if (running_.load() || worker_.joinable()) {
        std::cerr
            << "SensorService already running\n";
        return false;
    }

    // 检查采样周期是否合法
    if (period_.count() <= 0) {
        std::cerr
            << "Invalid sampling period\n";
        return false;
    }

    // 清空上一轮采集的数据和统计信息
    {
        std::lock_guard<std::mutex> lock(latest_mutex_);

        latest_data_ = SensorData{};
        has_latest_ = false;

        stats_ = SensorStats{};

        first_success_ns_ = 0;
        last_success_ns_ = 0;
    }

    // 重置成功采样计数
    sample_count_.store(0);

    // 初始化 MPU6050Reader
    if (!reader_.initialize()) {
        std::cerr
            << "MPU6050 initialization failed\n";
        return false;
    }

    // 设置运行标志
    running_.store(true);

    try {
        // 创建后台采集线程
        worker_ = std::thread(
            &SensorService::workerLoop,
            this
        );
    }
    catch (const std::system_error& e) {

        // 线程创建失败，恢复运行标志
        running_.store(false);

        std::cerr
            << "Thread creation failed: "
            << e.what()
            << '\n';

        return false;
    }

    return true;
}

// ============================================================
// 4. 停止采集服务
// ============================================================

void SensorService::stop()
{
    // 通知后台线程退出
    running_.store(false);

    // 等待后台线程执行结束
    if (worker_.joinable()) {
        worker_.join();
    }
}

// ============================================================
// 5. 新增：设置采样回调函数
// ============================================================

bool SensorService::setSampleCallback(
    SampleCallback callback)
{
    // 仅允许主控线程在启动前或完全停止后调用
    // 不允许采集期间更换回调函数
    if (running_.load() || worker_.joinable()) {

        std::cerr
            << "Cannot change callback while running\n";

        return false;
    }

    // 保存新的回调对象
    // std::move 避免不必要的额外复制
    sample_callback_ = std::move(callback);

    return true;
}

// ============================================================
// 6. 获取成功采样次数
// ============================================================

std::uint64_t SensorService::sampleCount() const
{
    return sample_count_.load();
}

// ============================================================
// 7. 获取最新六轴数据
// ============================================================

bool SensorService::getLatest(
    SensorData& data) const
{
    // 使用互斥锁保护共享数据
    std::lock_guard<std::mutex> lock(latest_mutex_);

    // 尚未获得有效样本
    if (!has_latest_) {
        return false;
    }

    // 返回完整数据副本
    data = latest_data_;

    return true;
}

// ============================================================
// 8. 获取采样运行统计信息
// ============================================================

SensorStats SensorService::stats() const
{
    // 获取互斥锁，保证统计快照的一致性
    std::lock_guard<std::mutex> lock(latest_mutex_);

    // 复制统计数据
    SensorStats result = stats_;

    // 是否已有有效数据
    result.has_data = has_latest_;

    // 计算最新样本的数据年龄
    if (has_latest_) {

        const auto now =
            std::chrono::steady_clock::now()
                .time_since_epoch();

        const auto now_ns =
            std::chrono::duration_cast<
                std::chrono::nanoseconds
            >(now).count();

        // 确保当前时间不早于样本时间
        if (now_ns >= 0 &&
            static_cast<std::uint64_t>(now_ns) >=
                latest_data_.timestamp_ns) {

            result.data_age_ms =
                static_cast<double>(
                    static_cast<std::uint64_t>(now_ns) -
                    latest_data_.timestamp_ns
                ) / 1000000.0;
        }
    }

    return result;
}

// ============================================================
// 9. 后台采集线程
// ============================================================

void SensorService::workerLoop()
{
    // 使用单调时钟安排采样周期
    auto next_time =
        std::chrono::steady_clock::now();

    // 运行标志为 true 时持续采集
    while (running_.load()) {

        // 保存本次采集的完整六轴数据
        SensorData sample;

        // ----------------------------------------------------
        // 第一步：执行 MPU6050 六轴读取
        // ----------------------------------------------------

        // 记录读取开始时间
        const auto read_begin =
            std::chrono::steady_clock::now();

        // 执行真实 IIO sysfs 读取
        const bool success =
            reader_.read(sample);

        // 记录读取完成时间
        const auto read_end =
            std::chrono::steady_clock::now();

        // 计算读取耗时，单位 ms
        const double read_ms =
            std::chrono::duration<double, std::milli>(
                read_end - read_begin
            ).count();

        std::uint64_t current_count = 0;

        // ----------------------------------------------------
        // 第二步：更新最新数据和运行统计
        // ----------------------------------------------------

        {
            // 进入临界区
            std::lock_guard<std::mutex> lock(
                latest_mutex_
            );

            // 1. 更新读取尝试次数
            ++stats_.read_attempts;

            // 2. 更新平均读取耗时
            stats_.avg_read_ms +=
                (read_ms - stats_.avg_read_ms) /
                static_cast<double>(
                    stats_.read_attempts
                );

            // 3. 更新最大读取耗时
            stats_.max_read_ms =
                std::max(
                    stats_.max_read_ms,
                    read_ms
                );

            if (success) {

                // 4. 更新成功采样次数
                ++stats_.successful_samples;

                // 5. 计算采样率和采样周期误差
                if (stats_.successful_samples == 1) {

                    // 第一组成功样本
                    first_success_ns_ =
                        sample.timestamp_ns;

                } else {

                    // 计算相邻成功样本时间间隔
                    const auto interval_ns =
                        sample.timestamp_ns -
                        last_success_ns_;

                    const double interval_ms =
                        static_cast<double>(
                            interval_ns
                        ) / 1000000.0;

                    // 与目标采样周期比较
                    const double error_ms =
                        std::abs(
                            interval_ms -
                            static_cast<double>(
                                period_.count()
                            )
                        );

                    // 更新最大周期误差
                    stats_.max_period_error_ms =
                        std::max(
                            stats_.max_period_error_ms,
                            error_ms
                        );

                    // 从第一次到当前成功样本的时间跨度
                    const auto elapsed_ns =
                        sample.timestamp_ns -
                        first_success_ns_;

                    if (elapsed_ns > 0) {

                        // 实际平均采样率
                        stats_.actual_hz =
                            static_cast<double>(
                                stats_.successful_samples - 1
                            ) * 1000000000.0 /
                            static_cast<double>(
                                elapsed_ns
                            );
                    }
                }

                // 6. 更新最近一次成功采样时间戳
                last_success_ns_ =
                    sample.timestamp_ns;

                // 7. 发布最新完整六轴数据
                latest_data_ = sample;
                has_latest_ = true;

                // 8. 更新成功采样计数
                current_count =
                    sample_count_.fetch_add(1) + 1;

            } else {

                // 9. 记录读取失败
                ++stats_.failed_reads;
            }

        } // latest_mutex_ 在这里自动释放

        // ----------------------------------------------------
        // 第三步：打印传感器运行日志
        // ----------------------------------------------------

        if (success) {

            // 第一组和每十组成功样本打印一次
            if (current_count == 1 ||
                current_count % 10 == 0) {

                std::cout
                    << "[Sensor] sample="
                    << current_count
                    << " ax=" << sample.ax
                    << " ay=" << sample.ay
                    << " az=" << sample.az
                    << '\n';
            }

        } else {

            std::cerr
                << "MPU6050 read failed\n";
        }

        // ----------------------------------------------------
        // 第四步：新增采样回调
        // ----------------------------------------------------

        // 只有成功采集的数据才传递给业务模块
        // 必须在 latest_mutex_ 解锁之后执行回调
        if (success && sample_callback_) {

            try {

                // 将当前完整六轴样本传给 MotionDetector
                // 或其他注册的业务回调
                sample_callback_(sample);

            }
            catch (const std::exception& e) {

                // 避免回调异常直接终止后台线程
                std::cerr
                    << "Sample callback exception: "
                    << e.what()
                    << '\n';

            }
            catch (...) {

                // 捕获其他类型的 C++ 异常
                std::cerr
                    << "Unknown sample callback exception\n";
            }
        }

        // ----------------------------------------------------
        // 第五步：周期调度
        // ----------------------------------------------------

        // 计算下一次目标采样时刻
        next_time += period_;

        const auto now =
            std::chrono::steady_clock::now();

        // 如果本轮已经超过预定时间
        // 则从当前时间重新安排下一次采集
        if (next_time < now) {

            next_time = now + period_;
        }

        // 等待下一次采样时间
        std::this_thread::sleep_until(next_time);
    }

    std::cout
        << "Sensor worker stopped\n";
}

} // namespace rk3576::sensor
