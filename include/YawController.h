#ifndef YAW_CONTROLLER_H
#define YAW_CONTROLLER_H

#include <string>

class YawController {
public:
    // 构造函数：不再需要初始的 yaw 和 pitch，依赖外部更新
    YawController(double step = 0.01);

    // 根据水平偏差计算Yaw调整
    void update(double diff, bool diffValid, const std::string& position);

    // 设置单片机发送来的基准 yaw 和 pitch（上位机接收的值）
    void setBaseYaw(double yaw);
    void setBasePitch(double pitch);

    // 获取修正后的Yaw角度（基准值 + 修正值），用于发送回单片机
    double getCorrectedYaw() const;

    // 获取修正后的Pitch角度（基准值 + 修正值），用于发送回单片机
    double getCorrectedPitch() const;

    // 获取当前的修正增量（用于调试）
    double getYawCorrection() const;
    double getPitchCorrection() const;

    bool isHit(double diff) const;  // 判断是否命中

private:
    double baseYaw_;        // 单片机发送的基准Yaw角度
    double basePitch_;      // 单片机发送的基准Pitch角度
    double yawCorrection_;  // Yaw修正增量
    double pitchCorrection_;// Pitch修正增量
    double step_;           // 每次调整的Yaw角度步长
};

#endif // YAW_CONTROLLER_H