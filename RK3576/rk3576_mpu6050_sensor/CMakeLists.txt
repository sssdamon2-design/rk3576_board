cmake_minimum_required(VERSION 3.16)

project(rk3576_mpu6050_sensor
    VERSION 1.0
    LANGUAGES CXX
)

# 使用 C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 启用常见编译警告
add_compile_options(
    -Wall
    -Wextra
    -Wpedantic
    -Werror
)

# 创建 SensorData 测试程序
add_executable(test_sensor_data
    tests/test_sensor_data.cpp
)

# 配置头文件搜索目录
target_include_directories(test_sensor_data PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# 启用 CTest
enable_testing()

add_test(
    NAME sensor_data_test
    COMMAND test_sensor_data
)


# MPU6050 IIO Reader
add_library(mpu6050_reader
    src/sensor/mpu6050_reader.cpp
)

target_include_directories(mpu6050_reader PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# MPU6050 设备发现测试
add_executable(test_mpu6050_discovery
    tests/test_mpu6050_discovery.cpp
)

target_link_libraries(test_mpu6050_discovery PRIVATE
    mpu6050_reader
)

add_test(
    NAME mpu6050_discovery_test
    COMMAND test_mpu6050_discovery
)


# 六轴实机采集测试
add_executable(test_mpu6050_read
    tests/test_mpu6050_read.cpp
)

target_link_libraries(test_mpu6050_read PRIVATE
    mpu6050_reader
)

add_test(
    NAME mpu6050_read_test
    COMMAND test_mpu6050_read
)
# C++ 线程支持
find_package(Threads REQUIRED)

# SensorService 静态库
add_library(sensor_service
    src/sensor/sensor_service.cpp
)

target_include_directories(sensor_service PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(sensor_service PUBLIC
    mpu6050_reader
    Threads::Threads
)

# 后台采集测试
add_executable(test_sensor_service
    tests/test_sensor_service.cpp
)

target_link_libraries(test_sensor_service PRIVATE
    sensor_service
)

add_test(
    NAME sensor_service_test
    COMMAND test_sensor_service
)



# 最新六轴数据线程安全测试
add_executable(test_sensor_latest
    tests/test_sensor_latest.cpp
)

target_link_libraries(test_sensor_latest PRIVATE
    sensor_service
)

add_test(
    NAME sensor_latest_test
    COMMAND test_sensor_latest
)
# SensorService 生命周期测试
add_executable(test_sensor_lifecycle
    tests/test_sensor_lifecycle.cpp
)

target_link_libraries(test_sensor_lifecycle PRIVATE
    sensor_service
)

add_test(
    NAME sensor_lifecycle_test
    COMMAND test_sensor_lifecycle
)


# SensorService 性能统计测试
add_executable(test_sensor_stats
    tests/test_sensor_stats.cpp
)

target_link_libraries(test_sensor_stats PRIVATE
    sensor_service
)

add_test(
    NAME sensor_stats_test
    COMMAND test_sensor_stats
)


# 长期稳定性测试程序
add_executable(test_sensor_stability
    tests/test_sensor_stability.cpp
)

target_link_libraries(test_sensor_stability PRIVATE
    sensor_service
)




# 第四节：MPU6050 静态基线测量
add_executable(measure_static_baseline
    tests/measure_static_baseline.cpp
)

target_link_libraries(measure_static_baseline PRIVATE
    mpu6050_reader
)


add_executable(test_gyro_calibration
    tests/test_gyro_calibration.cpp
)

target_include_directories(test_gyro_calibration PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(test_gyro_calibration PRIVATE
    mpu6050_reader
)

# MPU6050 加速度计六面测量
add_executable(measure_accel_6face
    tests/measure_accel_6face.cpp
)

target_link_libraries(measure_accel_6face PRIVATE
    mpu6050_reader
)

# 加速度计软件校准独立验证
add_executable(test_accel_calibration
    tests/test_accel_calibration.cpp
)

target_link_libraries(test_accel_calibration PRIVATE
    mpu6050_reader
)

# 第四节：坐标系转换测试
add_executable(test_coordinate_transform
    tests/test_coordinate_transform.cpp
)

target_include_directories(test_coordinate_transform PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

add_test(
    NAME coordinate_transform_test
    COMMAND test_coordinate_transform
)


# 4.3 低通滤波数学单元测试
add_executable(test_motion_filter
    tests/test_motion_filter.cpp
)

target_include_directories(test_motion_filter PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

add_test(
    NAME motion_filter_test
    COMMAND test_motion_filter
)

# 4.3 MPU6050 实机滤波测试
add_executable(test_motion_filter_live
    tests/test_motion_filter_live.cpp
)

target_include_directories(test_motion_filter_live PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(test_motion_filter_live PRIVATE
    mpu6050_reader
)

# 4.4 重力基线补偿单元测试
add_executable(test_gravity_compensator
    tests/test_gravity_compensator.cpp
)

target_include_directories(test_gravity_compensator PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

add_test(
    NAME gravity_compensator_test
    COMMAND test_gravity_compensator
)

# 4.4 MPU6050 重力基线补偿实机测试
add_executable(test_gravity_compensator_live
    tests/test_gravity_compensator_live.cpp
)

target_include_directories(test_gravity_compensator_live PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(test_gravity_compensator_live PRIVATE
    mpu6050_reader
)

# 4.5 急加速与急刹车检测单元测试
add_executable(test_longitudinal_motion
    tests/test_longitudinal_motion.cpp
)

target_include_directories(test_longitudinal_motion PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

add_test(
    NAME longitudinal_motion_test
    COMMAND test_longitudinal_motion
)

# 4.6 急转弯与颠簸检测单元测试
add_executable(test_turn_bump_detector
    tests/test_turn_bump_detector.cpp
)

target_include_directories(test_turn_bump_detector PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

add_test(
    NAME turn_bump_detector_test
    COMMAND test_turn_bump_detector
)

# 4.7 统一运动检测流水线测试
add_executable(test_motion_detector
    tests/test_motion_detector.cpp
)

target_include_directories(test_motion_detector PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

add_test(
    NAME motion_detector_test
    COMMAND test_motion_detector
)

# 4.7 SensorService 采样回调测试
add_executable(test_sensor_callback
    tests/test_sensor_callback.cpp
)

target_link_libraries(test_sensor_callback PRIVATE
    sensor_service
)

add_test(
    NAME sensor_callback_test
    COMMAND test_sensor_callback
)

# 4.7 SensorService + MotionDetector 实机测试
add_executable(test_sensor_motion_live
    tests/test_sensor_motion_live.cpp
)

target_link_libraries(test_sensor_motion_live PRIVATE
    sensor_service
)