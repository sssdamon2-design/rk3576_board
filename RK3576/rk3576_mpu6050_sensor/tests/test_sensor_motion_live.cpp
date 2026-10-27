
#include <array>
#include <chrono>
#include <cstdint>
#include <iostream>
#include <thread>

#include "sensor/sensor_service.h"
#include "motion/motion_detector.h"

using rk3576::sensor::SensorData;
using rk3576::sensor::SensorService;

using rk3576::motion::MotionDetector;
using rk3576::motion::MotionDetectorConfig;
using rk3576::motion::UnifiedEventType;

const char* eventName(UnifiedEventType type)
{
    switch (type) {
    case UnifiedEventType::HarshAcceleration:
        return "HarshAcceleration";

    case UnifiedEventType::HarshBraking:
        return "HarshBraking";

    case UnifiedEventType::SharpLeftTurn:
        return "SharpLeftTurn";

    case UnifiedEventType::SharpRightTurn:
        return "SharpRightTurn";

    case UnifiedEventType::Bump:
        return "Bump";
    }

    return "Unknown";
}

int main()
{
    using namespace std::chrono_literals;

    // 1. 配置真实 MPU6050 的校准参数
    MotionDetectorConfig config;

    config.accel = {
         0.212559,
        -0.144483,
         1.515511,
         1.001213,
         1.006555,
         0.978363
    };

    config.gyro = {
        -0.0382023,
        -0.0129790,
         0.0148742
    };

    config.baseline_samples = 50;
    config.filter_tau_seconds = 0.1;

    // 本次仅做传感器坐标系下的静止验证。
    // 使用默认单位矩阵，不宣称已完成实车坐标对齐。
    MotionDetector detector(config);

    // 2. 创建真实传感器采集服务
    SensorService service(100ms);

    std::uint64_t callback_count = 0;
    std::uint64_t rejected_count = 0;
    std::uint64_t event_count = 0;

    bool baseline_reported = false;

    // 3. 注册运动检测回调
    const bool registered = service.setSampleCallback(
        [&](const SensorData& sample) {

            ++callback_count;

            // 每一组成功样本进入 MotionDetector
            const auto result = detector.process(sample);

            if (!result) {
                ++rejected_count;
                return;
            }

            // 第 50 组左右：静态基线准备完成
            if (result->ready && !baseline_reported) {

                baseline_reported = true;

                std::array<double, 3> baseline{};

                if (detector.getBaseline(baseline)) {

                    std::cout
                        << "[Motion] Baseline ready: "
                        << baseline[0] << ", "
                        << baseline[1] << ", "
                        << baseline[2] << '\n';
                }
            }

            // 输出统一事件
            for (const auto& event : result->events) {

                ++event_count;

                std::cout
                    << "[MotionEvent] "
                    << eventName(event.type)
                    << " sample=" << event.sample_id
                    << " value=" << event.primary_value
                    << '\n';
            }
        }
    );

    if (!registered) {
        std::cerr << "Cannot register callback\n";
        return 1;
    }

    std::cout
        << "Keep MPU6050 stationary for 8 seconds\n";

    // 4. 真正启动 SensorService 后台采集
    if (!service.start()) {
        return 1;
    }

    std::this_thread::sleep_for(8s);

    // 5. 停止线程并等待所有回调结束
    service.stop();

    const auto stats = service.stats();

    std::cout
        << "\n=== Sensor Motion Integration ===\n";

    std::cout
        << "Successful samples: "
        << stats.successful_samples << '\n';

    std::cout
        << "Callback samples: "
        << callback_count << '\n';

    std::cout
        << "Rejected samples: "
        << rejected_count << '\n';

    std::cout
        << "Motion events: "
        << event_count << '\n';

    std::cout
        << "Motion baseline ready: "
        << detector.ready() << '\n';

    // 6. 验证采样链路和基线初始化
    if (!detector.ready() ||
        callback_count != stats.successful_samples ||
        rejected_count != 0) {

        std::cerr
            << "Sensor motion integration FAILED\n";
        return 1;
    }

    std::cout
        << "Sensor motion integration PASSED\n";

    return 0;
}
