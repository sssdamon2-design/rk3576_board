cmake_minimum_required(VERSION 3.16)

project(rk3576_interview_lab LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(rk3576_interview_lab 
src/main.cpp
src/router/intent_router.cpp
)


#配置头文件   目标文件  自动用作编译当前目标  cmake路径/其他
target_include_directories(rk3576_interview_lab
PRIVATE
${CMAKE_CURRENT_SOURCE_DIR}/include)