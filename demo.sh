#!/bin/bash

# 装甲板识别系统 - 演示脚本
# 展示完整轨迹处理和视频输出功能

echo "==========================================="
echo "  装甲板识别系统 - 完整轨迹处理演示"
echo "==========================================="
echo ""

cd build

# 检查视频文件
if [ ! -f "../2月7日.mp4" ]; then
    echo "[ERROR] 未找到测试视频文件: 2月7日.mp4"
    exit 1
fi

# 演示1: 实时处理（不保存）
echo "[演示 1] 实时处理视频（按q退出）"
echo "命令: ./armor_judge ../2月7日.mp4"
echo "按回车开始..."
read
./armor_judge ../2月7日.mp4

echo ""
echo "==========================================="
echo ""

# 演示2: 处理并保存
echo "[演示 2] 处理视频并保存结果"
echo "命令: ./armor_judge ../2月7日.mp4 --output result.mp4"
echo "按回车开始..."
read
./armor_judge ../2月7日.mp4 --output result.mp4

# 检查输出
if [ -f "result.mp4" ]; then
    echo ""
    echo "[SUCCESS] 处理完成！"
    ls -lh result.mp4
    echo ""
    echo "你可以使用以下命令播放结果:"
    echo "  vlc result.mp4"
    echo "  mpv result.mp4"
    echo "  ffplay result.mp4"
fi

echo ""
echo "==========================================="
echo "演示完成！"
echo ""
echo "功能总结："
echo "  ✓ 处理每一帧，不跳帧"
echo "  ✓ 完整轨迹记录（最多300点）"
echo "  ✓ 自动验证轨迹合理性"
echo "  ✓ 支持视频输出保存"
echo "  ✓ 详细的统计信息"
echo ""
