#!/bin/bash

# Armor Judge C++ 编译脚本

echo "======================================"
echo "  Armor Judge C++ - 编译脚本"
echo "======================================"

# 检查依赖
echo "检查依赖..."

# 检查OpenVINO
if [ -z "$INTEL_OPENVINO_DIR" ]; then
    echo "警告: OpenVINO环境未设置，尝试加载..."
    if [ -f "/opt/intel/openvino/setupvars.sh" ]; then
        source /opt/intel/openvino/setupvars.sh
    elif [ -f "/opt/intel/openvino_2023/setupvars.sh" ]; then
        source /opt/intel/openvino_2023/setupvars.sh
    else
        echo "错误: 找不到OpenVINO安装路径"
        exit 1
    fi
fi

# 检查OpenCV
if ! pkg-config --exists opencv4; then
    echo "错误: 未找到OpenCV 4.x"
    exit 1
fi

echo "依赖检查完成！"
echo ""

# 创建构建目录
if [ ! -d "build" ]; then
    echo "创建build目录..."
    mkdir build
fi

cd build

# 清理旧文件
echo "清理旧构建文件..."
rm -rf *

# 配置CMake
echo "配置CMake..."
cmake .. || {
    echo "CMake配置失败！"
    exit 1
}

# 编译
echo ""
echo "开始编译..."
make -j$(nproc) || {
    echo "编译失败！"
    exit 1
}

echo ""
echo "======================================"
echo "  编译成功！"
echo "======================================"
echo "可执行文件: ./armor_judge"
echo ""
echo "使用方法:"
echo "  ./armor_judge --video <视频路径> --model <模型路径> --device <CPU|GPU> --tracker <CSRT|KCF>"
echo ""
echo "示例:"
echo "  ./armor_judge --video ../2.6常州未剪辑.avi --model ../best.onnx --device CPU"
echo ""
