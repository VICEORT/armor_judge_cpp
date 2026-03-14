#include "YawController.h"
#include <iostream>
#include <cmath>

YawController::YawController(double step)
    : baseYaw_(0.0),          // 初始基准yaw为0
      basePitch_(0.0),        // 初始基准pitch为0
      yawCorrection_(0.0),    // 初始修正值为0
      pitchCorrection_(0.0),  // 初始修正值为0
      step_(step) {}

bool YawController::isHit(double diff) const {
    // 命中条件：误差小于阈值
    const double hitThreshold = 5.0;  // 像素阈值，可调

    return std::abs(diff) < hitThreshold;
}

void YawController::update(double diff, bool diffValid, const std::string& position) {
    if (!diffValid) return;

    // 输出当前diff值用于调试
    std::cout << "[DEBUG] diff = " << diff << " px" << std::endl;

    // 判断是否命中目标
    if (isHit(diff)) {
        std::cout << "[HIT] 命中目标！" << std::endl;
        std::cout << "[PITCH] 修正后Pitch值: " << getCorrectedPitch() << " deg" << std::endl;
        std::cout << "       (基准: " << basePitch_ << ", 修正: " << pitchCorrection_ << ")" << std::endl;
        return;
    }

    // 弹丸左侧，向右调整（增加yaw修正值）
    if (diff < 0) {
        yawCorrection_ += step_;
        std::cout << "[YAW] 向右调整 +" << step_ << " deg" << std::endl;
        std::cout << "      修正后Yaw: " << getCorrectedYaw() 
                  << " (基准: " << baseYaw_ << ", 修正: " << yawCorrection_ << ")" << std::endl;
    }
    // 弹丸右侧，向左调整（减少yaw修正值）
    else if (diff > 0) {
        yawCorrection_ -= step_;
        std::cout << "[YAW] 向左调整 -" << step_ << " deg" << std::endl;
        std::cout << "      修正后Yaw: " << getCorrectedYaw() 
                  << " (基准: " << baseYaw_ << ", 修正: " << yawCorrection_ << ")" << std::endl;
    }
}

// 设置单片机发送来的基准值
void YawController::setBaseYaw(double yaw) {
    baseYaw_ = yaw;
    std::cout << "[COMM] 接收基准Yaw: " << yaw << " deg" << std::endl;
}

void YawController::setBasePitch(double pitch) {
    basePitch_ = pitch;
    std::cout << "[COMM] 接收基准Pitch: " << pitch << " deg" << std::endl;
}

// 获取修正后的绝对值（用于发送回单片机）
double YawController::getCorrectedYaw() const {
    return baseYaw_ + yawCorrection_;
}

double YawController::getCorrectedPitch() const {
    return basePitch_ + pitchCorrection_;
}

// 获取修正增量（用于调试）
double YawController::getYawCorrection() const {
    return yawCorrection_;
}

double YawController::getPitchCorrection() const {
    return pitchCorrection_;
}