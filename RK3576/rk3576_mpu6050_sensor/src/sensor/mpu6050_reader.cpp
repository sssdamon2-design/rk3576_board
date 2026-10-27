
#include "sensor/mpu6050_reader.h"

#include <chrono>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <string>

namespace rk3576::sensor {

namespace {

// 读取指定 sysfs 文件中的一个数值
template <typename T>
bool readAttribute(
    const std::filesystem::path& path,
    T& value)
{
    std::ifstream file(path);

    if (!file.is_open()) {
        std::cerr << "Cannot open: "
                  << path << '\n';
        return false;
    }

    if (!(file >> value)) {
        std::cerr << "Cannot read: "
                  << path << '\n';
        return false;
    }

    return true;
}

} // anonymous namespace

bool MPU6050Reader::initialize()
{
    const std::filesystem::path root =
        "/sys/bus/iio/devices";

    device_path_.clear();
    accel_scale_ = 0.0;
    gyro_scale_ = 0.0;
    sample_id_ = 0;

    try {
        for (const auto& entry :
             std::filesystem::directory_iterator(root)) {

            const std::string dir_name =
                entry.path().filename().string();

            if (dir_name.rfind("iio:device", 0) != 0) {
                continue;
            }

            std::ifstream name_file(entry.path() / "name");

            if (!name_file.is_open()) {
                continue;
            }

            std::string device_name;

            if (!(name_file >> device_name)) {
                continue;
            }

            if (device_name != "mpu6050") {
                continue;
            }

            const auto candidate = entry.path();

            double accel_scale = 0.0;
            double gyro_scale = 0.0;

            if (!readAttribute(
                    candidate / "in_accel_scale",
                    accel_scale) ||
                !readAttribute(
                    candidate / "in_anglvel_scale",
                    gyro_scale)) {
                return false;
            }

            if (!std::isfinite(accel_scale) ||
                !std::isfinite(gyro_scale) ||
                accel_scale <= 0.0 ||
                gyro_scale <= 0.0) {

                std::cerr << "Invalid IIO scale\n";
                return false;
            }

            device_path_ = candidate;
            accel_scale_ = accel_scale;
            gyro_scale_ = gyro_scale;

            std::cout << "Found MPU6050: "
                      << device_path_ << '\n';

            std::cout << "Accel scale: "
                      << accel_scale_ << '\n';

            std::cout << "Gyro scale: "
                      << gyro_scale_ << '\n';

            return true;
        }
    }
    catch (const std::filesystem::filesystem_error& e) {
        std::cerr << "IIO discovery error: "
                  << e.what() << '\n';
        return false;
    }

    std::cerr << "MPU6050 IIO device not found\n";
    return false;
}

bool MPU6050Reader::read(SensorData& data)
{
    if (device_path_.empty()) {
        std::cerr << "MPU6050Reader not initialized\n";
        return false;
    }

    std::int64_t ax_raw = 0;
    std::int64_t ay_raw = 0;
    std::int64_t az_raw = 0;

    std::int64_t gx_raw = 0;
    std::int64_t gy_raw = 0;
    std::int64_t gz_raw = 0;

    // 读取三轴加速度
    if (!readAttribute(device_path_ / "in_accel_x_raw", ax_raw) ||
        !readAttribute(device_path_ / "in_accel_y_raw", ay_raw) ||
        !readAttribute(device_path_ / "in_accel_z_raw", az_raw)) {
        return false;
    }

    // 读取三轴角速度
    if (!readAttribute(device_path_ / "in_anglvel_x_raw", gx_raw) ||
        !readAttribute(device_path_ / "in_anglvel_y_raw", gy_raw) ||
        !readAttribute(device_path_ / "in_anglvel_z_raw", gz_raw)) {
        return false;
    }

    // 先保存到临时对象，避免部分读取失败时
    // 修改调用者原有的数据
    SensorData sample;

    // Raw -> 物理单位
    sample.ax = static_cast<double>(ax_raw) * accel_scale_;
    sample.ay = static_cast<double>(ay_raw) * accel_scale_;
    sample.az = static_cast<double>(az_raw) * accel_scale_;

    sample.gx = static_cast<double>(gx_raw) * gyro_scale_;
    sample.gy = static_cast<double>(gy_raw) * gyro_scale_;
    sample.gz = static_cast<double>(gz_raw) * gyro_scale_;

    // 记录本次软件采集完成的时间
    const auto now =
        std::chrono::steady_clock::now().time_since_epoch();

    sample.timestamp_ns =
        static_cast<std::uint64_t>(
            std::chrono::duration_cast<
                std::chrono::nanoseconds>(now).count()
        );

    // 成功读取后更新序号
    sample.sample_id = ++sample_id_;

    // 所有数据有效后，再赋值给调用者
    data = sample;

    return true;
}

const std::filesystem::path&
MPU6050Reader::devicePath() const
{
    return device_path_;
}

} // namespace rk3576::sensor
