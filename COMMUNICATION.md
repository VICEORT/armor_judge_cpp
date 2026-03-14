# 上下位机通信说明

## 概述

本文档说明上位机（PC）与下位机（单片机）之间的通信协议和使用方法。

## 通信流程

### 1. 接收阶段（单片机 → 上位机）

单片机发送当前的 **yaw** 和 **pitch** 值给上位机：

```
单片机 ---> [yaw, pitch] ---> 上位机
```

### 2. 处理阶段（上位机内部）

上位机：
- 接收 yaw 和 pitch 作为**基准值**
- 根据视觉检测结果计算**修正增量**
- 将基准值与修正增量相加，得到**修正后的绝对值**

```
修正后的 yaw   = 接收的 yaw   + yaw修正增量
修正后的 pitch = 接收的 pitch + pitch修正增量
```

### 3. 发送阶段（上位机 → 单片机）

上位机将修正后的**绝对值**发送回单片机：

```
上位机 ---> [修正后的yaw, 修正后的pitch] ---> 单片机
```

**重要**：发送的是修正后的绝对值，**不是增量差值**。

## 代码使用示例

### 初始化

```cpp
#include "YawController.h"

// 创建控制器，步长0.3度
YawController yawController(0.3);
```

### 接收数据（每帧/每次通信时）

```cpp
// 从单片机接收基准yaw和pitch
double receivedYaw = 0.0;    // 从串口/网络读取
double receivedPitch = 45.0; // 从串口/网络读取

// 设置基准值
yawController.setBaseYaw(receivedYaw);
yawController.setBasePitch(receivedPitch);
```

### 处理和更新

```cpp
// 根据视觉检测结果更新修正值
// diff: 弹丸中心与装甲板中心的水平距离（像素）
// diffValid: 该距离是否有效
// position: 位置信息（"Front"或"Back"）
yawController.update(diff, diffValid, position);
```

### 发送数据

```cpp
// 获取修正后的绝对值
double correctedYaw = yawController.getCorrectedYaw();
double correctedPitch = yawController.getCorrectedPitch();

// 发送给单片机
// serialPort.send(correctedYaw, correctedPitch);
// 或者通过网络发送
```

### 调试信息（可选）

```cpp
// 查看修正增量（用于调试）
double yawCorrection = yawController.getYawCorrection();
double pitchCorrection = yawController.getPitchCorrection();

std::cout << "Yaw修正增量: " << yawCorrection << " deg" << std::endl;
std::cout << "Pitch修正增量: " << pitchCorrection << " deg" << std::endl;
```

## API 参考

### YawController 类

#### 构造函数

```cpp
YawController(double step = 0.01);
```
- `step`: 每次调整的yaw角度步长（默认0.01度）

#### 设置基准值

```cpp
void setBaseYaw(double yaw);
void setBasePitch(double pitch);
```
- 设置从单片机接收的基准yaw和pitch值

#### 更新修正值

```cpp
void update(double diff, bool diffValid, const std::string& position);
```
- `diff`: 水平偏差（像素）
- `diffValid`: 偏差是否有效
- `position`: 位置信息（"Front"或"Back"）

#### 获取修正后的值（用于发送）

```cpp
double getCorrectedYaw() const;
double getCorrectedPitch() const;
```
- 返回修正后的绝对值（基准值 + 修正增量）
- **这是需要发送回单片机的值**

#### 获取修正增量（用于调试）

```cpp
double getYawCorrection() const;
double getPitchCorrection() const;
```
- 返回修正增量（仅用于调试，不直接发送给单片机）

## 通信协议建议

### 数据格式

建议使用以下格式进行串口通信：

**接收格式（单片机 → 上位机）：**
```
START|YAW:xxx.xx|PITCH:xxx.xx|END\n
```

**发送格式（上位机 → 单片机）：**
```
START|YAW:xxx.xx|PITCH:xxx.xx|END\n
```

### 示例

**接收：**
```
START|YAW:0.00|PITCH:45.00|END
```

**发送：**
```
START|YAW:0.30|PITCH:45.00|END
```

说明：上位机接收到yaw=0.00，计算出需要向右调整0.30度，因此发送回0.30（而不是+0.30）。

## 注意事项

1. **每次通信都要更新基准值**：每次从单片机接收到新的yaw/pitch时，都需要调用`setBaseYaw()`和`setBasePitch()`

2. **发送绝对值而非增量**：一定要发送`getCorrectedYaw()`和`getCorrectedPitch()`的返回值，这是修正后的绝对值

3. **线程安全**：如果使用多线程（例如一个线程接收数据，一个线程处理视频），需要添加互斥锁保护

4. **错误处理**：实际应用中需要添加超时、校验和等错误处理机制

## 完整示例代码

```cpp
#include "YawController.h"
#include <iostream>

int main() {
    YawController controller(0.3);
    
    // 模拟通信循环
    while (true) {
        // 1. 从单片机接收
        double rxYaw = receiveYawFromMCU();      // 自行实现
        double rxPitch = receivePitchFromMCU();  // 自行实现
        
        // 2. 设置基准值
        controller.setBaseYaw(rxYaw);
        controller.setBasePitch(rxPitch);
        
        // 3. 视觉处理（获取diff等）
        double diff = calculateDiff();  // 自行实现
        bool diffValid = true;
        std::string position = "Front";
        
        // 4. 更新修正值
        controller.update(diff, diffValid, position);
        
        // 5. 获取修正后的值并发送
        double txYaw = controller.getCorrectedYaw();
        double txPitch = controller.getCorrectedPitch();
        
        sendToMCU(txYaw, txPitch);  // 自行实现
        
        std::cout << "发送: Yaw=" << txYaw 
                  << ", Pitch=" << txPitch << std::endl;
    }
    
    return 0;
}
```

## 调试输出

程序运行时会输出以下调试信息：

```
[COMM] 接收基准Yaw: 0.0 deg
[COMM] 接收基准Pitch: 45.0 deg
[DEBUG] diff = -15.3 px
[YAW] 向右调整 +0.3 deg
      修正后Yaw: 0.3 (基准: 0, 修正: 0.3)
[SEND] 发送给单片机 -> Yaw: 0.3 deg, Pitch: 45.0 deg
```

这些信息帮助你验证通信流程是否正确。
