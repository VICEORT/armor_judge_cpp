# 完整轨迹处理 - 功能说明

## 📋 更新内容

### ✅ 已实现功能

#### 1. **逐帧处理，无跳帧**
程序使用 `while (cap.read(frame))` 循环处理视频的每一帧：
- 不跳过任何帧
- 保证轨迹的连续性
- 完整记录ball的运动过程

#### 2. **完整轨迹拟合**
```cpp
// 轨迹长度从100点增加到300点
while (detector.getTrajectory().size() > 300) {
    auto& traj = const_cast<std::vector<TrajectoryPoint>&>(detector.getTrajectory());
    traj.erase(traj.begin());
}
```

特点：
- 最多保留**300个轨迹点**（原来100个）
- 自动保存完整轨迹到历史记录
- 支持长时间跟踪

#### 3. **轨迹历史记录**
```cpp
// 所有完整轨迹都会被保存
std::vector<std::vector<TrajectoryPoint>> allTrajectories;
```

自动记录：
- 验证失败时保存当前轨迹（>5个点）
- ball消失时保存完整轨迹
- 视频结束时保存最后一条轨迹

#### 4. **视频输出功能**
新增 `--output` 参数：
```bash
./armor_judge video.mp4 --output result.mp4
```

特性：
- 保持原视频FPS和分辨率
- 包含所有可视化标注（检测框、轨迹、状态）
- MP4格式（H.264编码）
- 自动继承原视频参数

#### 5. **详细统计信息**
处理完成后自动显示：
```
[PERF] Device=GPU, Tracker=CSRT
       Total frames: 197
       Total time: 9.85s
       Average FPS: 20.0
       Average frame time: 50ms
       Ball detections: 45
       Armor detections: 197

[TRAJECTORY] 轨迹统计:
       Total trajectories: 3
       Total trajectory points: 125
       Average trajectory length: 41
       Max trajectory length: 89
```

包含：
- 性能统计（FPS、处理时间）
- 检测计数（armor/ball）
- 轨迹统计（数量、长度、点数）

## 🎯 核心改进

### 轨迹保存逻辑

**改进前**：
```cpp
if (isValid) {
    detector.addToTrajectory(newPt);
} else {
    detector.clearTrajectory();  // 直接清空，轨迹丢失
    detector.addToTrajectory(newPt);
}
```

**改进后**：
```cpp
if (isValid) {
    detector.addToTrajectory(newPt);
} else {
    // 保存当前轨迹到历史记录
    if (detector.getTrajectory().size() > 5) {
        allTrajectories.push_back(detector.getTrajectory());
    }
    detector.clearTrajectory();
    detector.addToTrajectory(newPt);
}
```

### 轨迹长度增加

| 项目 | 原值 | 新值 | 说明 |
|------|------|------|------|
| 最大轨迹点数 | 100 | 300 | 支持更长时间跟踪 |
| 最小保存长度 | - | 5 | 避免保存噪声轨迹 |

## 💡 使用示例

### 基础使用
```bash
# 实时处理
./armor_judge ../2月7日.mp4

# 保存处理结果
./armor_judge ../2月7日.mp4 --output result.mp4
```

### 组合参数
```bash
# CPU + KCF + 保存输出
./armor_judge video.mp4 --device CPU --tracker KCF --output output.mp4

# GPU + CSRT + 保存输出（推荐配置）
./armor_judge video.mp4 --output result.mp4
```

### 演示脚本
```bash
# 运行完整功能演示
./demo.sh
```

## 📊 轨迹验证规则

程序自动验证轨迹的合理性：

1. **垂直方向检查**
   - Ball必须主要向下运动
   - 允许小幅上下波动（卡尔曼滤波处理）

2. **横向移动限制**
   - 横向移动不能过大
   - 相对于armor的位置保持合理范围

3. **停滞检测**
   - 避免长时间静止的误检
   - 确保是运动的ball

4. **面积比例验证**
   - Ball面积 ≥ Armor面积 × 0.2（1/5）
   - Ball面积 ≤ Armor面积 × 0.5（1/2）

## 🔧 技术细节

### 内存管理
- 轨迹采用滑动窗口（300点上限）
- 历史轨迹自动清理短轨迹（<5点）
- 避免内存无限增长

### 性能优化
- GPU加速推理（OpenVINO）
- 混合检测策略（YOLO + 跟踪器）
- 自适应处理流程

### 输出格式
- 编码：H.264
- 容器：MP4
- FPS：自动继承原视频
- 分辨率：保持原始尺寸

## 📈 预期结果

### 典型输出示例
```
[INFO] 开始处理视频...
[提示] 按 'q' 退出, 按 'r' 重置跟踪器

[STAT] frame 0: armor=1, ball=0
[TRACK] Tracker status: initialized=0, confidence=0, miss_count=0
[STAT] frame 200: armor=201, ball=45

[PERF] Device=GPU, Tracker=CSRT
       Total frames: 400
       Total time: 20.5s
       Average FPS: 19.5
       Ball detections: 89
       Armor detections: 400

[TRAJECTORY] 轨迹统计:
       Total trajectories: 5
       Total trajectory points: 245
       Average trajectory length: 49
       Max trajectory length: 112

[INFO] 输出视频已保存: result.mp4
```

### 轨迹可视化
- 黄色线：当前活动轨迹
- 灰色线：历史完整轨迹
- 红色圆点：轨迹消失点

## 🎓 关键概念

### 完整轨迹定义
- 连续检测到的ball位置序列
- 经过验证的合理运动路径
- 包含至少5个有效点

### 轨迹分段原因
1. 验证失败（运动不合理）
2. Ball消失（进入armor或离开视野）
3. 长时间未检测到

### 历史记录价值
- 分析ball的完整运动模式
- 统计系统性能和准确率
- 后期数据分析和可视化

## 🚀 快速开始

```bash
# 1. 编译
cd /home/abc/Projects/armor_judge_cpp
./build.sh

# 2. 运行
cd build
./armor_judge ../2月7日.mp4 --output result.mp4

# 3. 查看结果
ls -lh result.mp4
vlc result.mp4  # 或使用其他播放器
```

---

**核心承诺：处理每一帧，拟合完整轨迹！** ✅
