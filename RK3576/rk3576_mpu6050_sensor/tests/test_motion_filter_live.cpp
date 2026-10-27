
#include <chrono>
#include <iostream>
#include <thread>

#include "sensor/mpu6050_reader.h"
#include "motion/motion_filter.h"

int main()
{
    using namespace std::chrono_literals;
    using rk3576::sensor::MPU6050Reader;
    using rk3576::sensor::SensorData;
    using rk3576::motion::MotionFilter;

    MPU6050Reader reader;

    if (!reader.initialize()) {
        std::cerr << "MPU6050 init failed\n";
        return 1;
    }

    // 滤波时间常数 0.2 秒
    MotionFilter filter(0.2);

    auto next_time = std::chrono::steady_clock::now();

    std::cout << "Raw vs Filtered:\n";

    // 采集 50 组，约 5 秒
    for (int i = 0; i < 50; ++i) {

        SensorData raw;
        SensorData filtered;

        if (!reader.read(raw)) {
            std::cerr << "Sensor read failed\n";
            return 1;
        }

        if (!filter.process(raw, filtered)) {
            std::cerr << "Filter failed\n";
            return 1;
        }

        // 每五组输出一次
        if (i % 5 == 0) {
            std::cout
                << "sample=" << raw.sample_id
                << " raw_ax=" << raw.ax
                << " filtered_ax=" << filtered.ax
                << " raw_gz=" << raw.gz
                << " filtered_gz=" << filtered.gz
                << '\n';
        }

        next_time += 100ms;

        std::this_thread::sleep_until(next_time);
    }

    std::cout << "Live motion filter test PASSED\n";

    return 0;
}
