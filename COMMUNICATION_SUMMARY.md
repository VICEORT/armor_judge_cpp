# 上下位机通信修改总结

## 修改内容

已将代码修改为：**发送修正后的绝对值（基准值+修正值）**，而不是只发送修正增量。

## 核心变化

### 1. YawController 类重构

**之前：** 
- 内部维护 `yaw_` 和 `pitch_` 变量
- 直接修改这些变量

**现在：**
- 分离存储：`baseYaw_`（基准值）+ `yawCorrection_`（修正增量）
- 分离存储：`basePitch_`（基准值）+ `pitchCorrection_`（修正增量）

### 2. 新增的接口

| 方法 | 用途 |
|------|------|
| `setBaseYaw(double)` | 设置从单片机接收的基准yaw值 |
| `setBasePitch(double)` | 设置从单片机接收的基准pitch值 |
| `getCorrectedYaw()` | 获取修正后的yaw（基准+修正），发送给单片机 |
| `getCorrectedPitch()` | 获取修正后的pitch（基准+修正），发送给单片机 |
| `getYawCorrection()` | 获取yaw修正增量（仅用于调试） |
| `getPitchCorrection()` | 获取pitch修正增量（仅用于调试） |

## 使用流程

```cpp
// 1. 接收单片机发送的yaw和pitch
yawController.setBaseYaw(receivedYaw);
yawController.setBasePitch(receivedPitch);

// 2. 处理视觉数据，更新修正值
yawController.update(diff, diffValid, position);

// 3. 发送修正后的绝对值回单片机
double correctedYaw = yawController.getCorrectedYaw();     // 这是基准值+修正值
double correctedPitch = yawController.getCorrectedPitch(); // 这是基准值+修正值
// 发送 correctedYaw 和 correctedPitch 给单片机
```

## 关键点

✅ **发送的是绝对值**：`getCorrectedYaw()` 返回 `baseYaw_ + yawCorrection_`

✅ **不是增量**：不是只发送 `yawCorrection_`

✅ **每次通信都更新基准**：每次从单片机接收新值时调用 `setBaseYaw()` 和 `setBasePitch()`

## 示例输出

```
[COMM] 接收基准Yaw: 0.0 deg
[COMM] 接收基准Pitch: 45.0 deg
[YAW] 向右调整 +0.3 deg
      修正后Yaw: 0.3 (基准: 0, 修正: 0.3)
[SEND] 发送给单片机 -> Yaw: 0.3 deg, Pitch: 45.0 deg
```

## 文件修改

- ✅ [include/YawController.h](include/YawController.h) - 接口定义
- ✅ [src/YawController.cpp](src/YawController.cpp) - 实现逻辑  
- ✅ [src/main.cpp](src/main.cpp) - 添加使用示例
- ✅ [COMMUNICATION.md](COMMUNICATION.md) - 详细使用文档

## 编译状态

✅ 编译成功，可以正常使用

## 下一步

需要根据实际硬件添加串口或网络通信代码，将数据真正发送给单片机。
