#!/bin/bash
# 编译整合后的动捕播放器（底盘原样 + 关节级手臂控制）
# 用法：
#   ./build.sh /path/to/h1_sdk
#   H1_SDK_DIR=/path/to/h1_sdk ./build.sh
set -e

SDK_DIR="${1:-${H1_SDK_DIR}}"
if [ -z "$SDK_DIR" ]; then
    echo "请指定 H1 SDK 根目录：./build.sh /path/to/h1_sdk"
    exit 1
fi

cd "$(dirname "$0")"
rm -rf build
mkdir -p build
cd build
cmake .. -DH1_SDK_DIR="$SDK_DIR"
make -j"$(nproc 2>/dev/null || echo 4)"
cp h1_mocap_arm_player ..
echo "编译完成: $(pwd)/../h1_mocap_arm_player"
