cmake_minimum_required(VERSION 3.10)
project(h1_mocap_arm_player CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Release)

# H1 SDK 路径：改成 H1_Robot.hpp / MotorStateMonitor.hpp 所在 SDK 根目录
# 也可用 cmake -DH1_SDK_DIR=/path/to/sdk 覆盖
set(H1_SDK_DIR "$ENV{H1_SDK_DIR}" CACHE PATH "H1 SDK 根目录")

include_directories(${H1_SDK_DIR}/include)

# 若 SDK 提供预编译库，取消下面两行注释并改成实际库名
# link_directories(${H1_SDK_DIR}/lib)
# set(H1_LIBS H1_Robot)

add_executable(h1_mocap_arm_player h1_mocap_arm_player.cpp)

if(DEFINED H1_LIBS)
    target_link_libraries(h1_mocap_arm_player ${H1_LIBS} pthread)
else()
    target_link_libraries(h1_mocap_arm_player pthread)
endif()
