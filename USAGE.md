# 装甲板识别系统 - 使用指南

## 🚀 快速开始

### 编译程序
```bash
./build.sh
```

### 运行程序
```bash
cd build

# 处理视频文件
./armor_judge ../视频文件.mp4

# 使用相机（相机编号0-9）
./armor_judge 0

# 保存处理结果
./armor_judge ../视频文件.mp4 --output 输出.mp4
```

## 📝 命令格式

```bash
./armor_judge <视频源> [选项]
```

### 参数说明

**必需参数**：
- `<视频源>`: 视频文件路径 或 相机编号(0-9)

**可选参数**（都有默认值）：
- `--device <CPU|GPU>`: 推理设备（默认：GPU）
- `--model <路径>`: 模型文件（默认：best.onnx）
- `--tracker <CSRT|KCF>`: 跟踪器类型（默认：CSRT）
- `--output <路径>`: 输出视频文件（可选，不指定则不保存）

## 💡 使用示例

```bash
# 最简单 - 使用默认GPU和CSRT
./armor_judge ../video.mp4

# 使用相机
./armor_judge 0

# 使用CPU推理
./armor_judge ../video.mp4 --device CPU

# 使用KCF跟踪器（更快）
./armor_judge ../video.mp4 --tracker KCF

# 保存处理后的视频
./armor_judge ../video.mp4 --output result.mp4

# 组合使用
./armor_judge ../video.mp4 --device GPU --tracker CSRT --output result.mp4
```

## ⌨️ 运行时按键

- `q` - 退出程序
- `r` - 重置跟踪器

## 🎯 检测规则

程序会自动过滤不合理的ball检测：
- ✓ Ball面积必须在armor面积的 1/5 到 1/2 之间
- ✓ Ball轨迹必须基本垂直向下
- ✓ Ball横向移动不能太大
- ✓ Ball不能长时间停滞

## 📊 输出说明

**可视化窗口**：
- 绿色框：Armor装甲板
- 红色框：YOLO检测的Ball
- 蓝色框：跟踪器检测的Ball
- 黄色框：融合结果
- 黄色轨迹：当前Ball轨迹
- 灰色轨迹：历史轨迹

**左上角状态**：
- Source：检测来源
- Conf：置信度
- Tracker：跟踪器状态
- Diff：横向偏移
- Horizontal：横向判断（true/Fault）
- Position：位置判断（Front/Back）

**处理完成后统计**：
- 性能统计：总帧数、平均FPS、处理时间
- 轨迹统计：总轨迹数、平均长度、最长轨迹

## 🎬 视频输出

使用 `--output` 参数可以保存处理后的视频：

```bash
# 处理并保存到output.mp4
./armor_judge ../2月7日.mp4 --output output.mp4

# 保存在不同目录
./armor_judge ../video.mp4 --output ~/results/processed.mp4
```

**注意**：
- 输出视频会保持原视频的FPS和分辨率
- 包含所有可视化标注（检测框、轨迹、状态信息）
- MP4格式（H.264编码）

## 🔧 故障排除

**GPU推理失败**：
```bash
./armor_judge video.mp4 --device CPU
```

**找不到模型文件**：
确保 `best.onnx` 在项目根目录或build目录

**视频打不开**：
使用相对于build目录的路径，如 `../video.mp4`

**无法创建输出视频**：
检查输出路径是否有写入权限

## 📈 特性说明

### 完整轨迹处理
- **处理所有帧**：程序不跳帧，逐帧处理视频
- **完整轨迹记录**：最多保留300个点的连续轨迹
- **轨迹统计**：自动统计所有完整轨迹的数量和长度
- **历史轨迹保存**：每条完整轨迹（>5个点）都会被记录

### 轨迹验证
程序使用多重验证确保轨迹质量：
1. 垂直方向检查（主要向下运动）
2. 横向移动限制（避免过度偏移）
3. 停滞检测（避免静止误判）
4. 面积比例验证（ball相对armor的合理大小）

---

**默认配置：GPU + CSRT + 自动查找模型**

