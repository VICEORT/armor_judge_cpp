#include "pitch.h"

#define _USE_MATH_DEFINES

#include <iostream>
#include <cmath>
#include <utility>

using namespace std;

// 定义 M_PI
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

double max_range = 0;

// 计算当前高度处的空气密度
double airDensity(double z) {
    const double rho0 = 1.225; // kg/m^3, 海平面空气密度
    const double h = 0.000128; // 密度衰减系数（经验模型）
    return rho0 * exp(-h * z);
}

// 使用 RK4 进行一步积分更新
void rungeKutta(double& x, double& y, double& z, double& vx, double& vy, double& vz, double m, double r, double C, double CL, double dt) {
    double k1x, k1y, k1z, k1vx, k1vy, k1vz;
    double k2x, k2y, k2z, k2vx, k2vy, k2vz;
    double k3x, k3y, k3z, k3vx, k3vy, k3vz;
    double k4x, k4y, k4z, k4vx, k4vy, k4vz;

    double ax, ay, az;
    double v = sqrt(vx * vx + vy * vy + vz * vz);
    double A = M_PI * r * r;
    double rho = airDensity(z);
    double drag_x = -0.5 * rho * C * A * v * vx;
    double drag_y = -0.5 * rho * C * A * v * vy;
    double drag_z = -0.5 * rho * C * A * v * vz;

    // 假设升力主要作用在 xz 平面，可按实测模型调整
    double alpha = atan2(vz, vx);
    double lift = 0.5 * rho * v * v * A * CL;
    double lift_x = -lift * sin(alpha);
    double lift_z = lift * cos(alpha);
    double lift_y = 0;

    ax = (drag_x + lift_x) / m;
    ay = (drag_y + lift_y) / m;
    az = -9.8 + (drag_z + lift_z) / m;

    k1x = vx;
    k1y = vy;
    k1z = vz;
    k1vx = ax;
    k1vy = ay;
    k1vz = az;

    v = sqrt((vx + 0.5 * dt * k1vx) * (vx + 0.5 * dt * k1vx) + (vy + 0.5 * dt * k1vy) * (vy + 0.5 * dt * k1vy) + (vz + 0.5 * dt * k1vz) * (vz + 0.5 * dt * k1vz));
    rho = airDensity(z + 0.5 * dt * k1z);
    drag_x = -0.5 * rho * C * A * v * (vx + 0.5 * dt * k1vx);
    drag_y = -0.5 * rho * C * A * v * (vy + 0.5 * dt * k1vy);
    drag_z = -0.5 * rho * C * A * v * (vz + 0.5 * dt * k1vz);
    alpha = atan2(vz + 0.5 * dt * k1vz, vx + 0.5 * dt * k1vx);
    lift = 0.5 * rho * v * v * A * CL;
    lift_x = -lift * sin(alpha);
    lift_y = 0;
    lift_z = lift * cos(alpha);
    ax = (drag_x + lift_x) / m;
    ay = (drag_y + lift_y) / m;
    az = -9.8 + (drag_z + lift_z) / m;

    k2x = vx + 0.5 * dt * k1vx;
    k2y = vy + 0.5 * dt * k1vy;
    k2z = vz + 0.5 * dt * k1vz;
    k2vx = ax;
    k2vy = ay;
    k2vz = az;

    v = sqrt((vx + 0.5 * dt * k2vx) * (vx + 0.5 * dt * k2vx) + (vy + 0.5 * dt * k2vy) * (vy + 0.5 * dt * k2vy) + (vz + 0.5 * dt * k2vz) * (vz + 0.5 * dt * k2vz));
    rho = airDensity(z + 0.5 * dt * k2z);
    drag_x = -0.5 * rho * C * A * v * (vx + 0.5 * dt * k2vx);
    drag_y = -0.5 * rho * C * A * v * (vy + 0.5 * dt * k2vy);
    drag_z = -0.5 * rho * C * A * v * (vz + 0.5 * dt * k2vz);
    alpha = atan2(vz + 0.5 * dt * k2vz, vx + 0.5 * dt * k2vx);
    lift = 0.5 * rho * v * v * A * CL;
    lift_x = -lift * sin(alpha);
    lift_y = 0;
    lift_z = lift * cos(alpha);
    ax = (drag_x + lift_x) / m;
    ay = (drag_y + lift_y) / m;
    az = -9.8 + (drag_z + lift_z) / m;

    k3x = vx + 0.5 * dt * k2vx;
    k3y = vy + 0.5 * dt * k2vy;
    k3z = vz + 0.5 * dt * k2vz;
    k3vx = ax;
    k3vy = ay;
    k3vz = az;

    v = sqrt((vx + dt * k3vx) * (vx + dt * k3vx) + (vy + dt * k3vy) * (vy + dt * k3vy) + (vz + dt * k3vz) * (vz + dt * k3vz));
    rho = airDensity(z + dt * k3z);
    drag_x = -0.5 * rho * C * A * v * (vx + dt * k3vx);
    drag_y = -0.5 * rho * C * A * v * (vy + dt * k3vy);
    drag_z = -0.5 * rho * C * A * v * (vz + dt * k3vz);
    alpha = atan2(vz + dt * k3vz, vx + dt * k3vx);
    lift = 0.5 * rho * v * v * A * CL;
    lift_x = -lift * sin(alpha);
    lift_y = 0;
    lift_z = lift * cos(alpha);
    ax = (drag_x + lift_x) / m;
    ay = (drag_y + lift_y) / m;
    az = -9.8 + (drag_z + lift_z) / m;

    k4x = vx + dt * k3vx;
    k4y = vy + dt * k3vy;
    k4z = vz + dt * k3vz;
    k4vx = ax;
    k4vy = ay;
    k4vz = az;

    x += (dt / 6.0) * (k1x + 2 * k2x + 2 * k3x + k4x);
    y += (dt / 6.0) * (k1y + 2 * k2y + 2 * k3y + k4y);
    z += (dt / 6.0) * (k1z + 2 * k2z + 2 * k3z + k4z);
    vx += (dt / 6.0) * (k1vx + 2 * k2vx + 2 * k3vx + k4vx);
    vy += (dt / 6.0) * (k1vy + 2 * k2vy + 2 * k3vy + k4vy);
    vz += (dt / 6.0) * (k1vz + 2 * k2vz + 2 * k3vz + k4vz);
}

std::pair<double, double> calculateHeightAndLateralDisplacementRK4(double x_target, double V0, double theta, double phi, double m, double r, double C, double CL) {
    double dt = 0.001;
    double x = 0, y = 0, z = 0;
    double vx = V0 * cos(theta) * cos(phi);  // x 方向初速度
    double vy = V0 * cos(theta) * sin(phi);  // y 方向初速度
    double vz = V0 * sin(theta);             // z 方向初速度

    while (x < x_target && z >= 0) {
        rungeKutta(x, y, z, vx, vy, vz, m, r, C, CL, dt);
    }

    // 返回 (横向偏移, 高度)
    return std::make_pair(y, z);
}

double calculateHeightAndLateralDisplacementRK41(double x_target, double V0, double theta, double phi, double m, double r, double C, double CL) {
    double dt = 0.001;
    double x = 0, y = 0, z = 0;
    double vx = V0 * cos(theta) * cos(phi);  // x 方向初速度
    double vy = V0 * cos(theta) * sin(phi);  // y 方向初速度
    double vz = V0 * sin(theta);             // z 方向初速度

    while (x < x_target && z >= 0) {
        rungeKutta(x, y, z, vx, vy, vz, m, r, C, CL, dt);
    }

    double V01 = sqrt(vx * vx + vy * vy + vz * vz);
    // 返回末速度大小
    return V01;
}

double calculateHeightAndLateralDisplacementRK42(double x_target, double V0, double theta, double phi, double m, double r, double C, double CL) {
    double dt = 0.001;
    double x = 0, y = 0, z = 0;
    double vx = V0 * cos(theta) * cos(phi);  // x 方向初速度
    double vy = V0 * cos(theta) * sin(phi);  // y 方向初速度
    double vz = V0 * sin(theta);             // z 方向初速度

    while (x < x_target && z >= 0) {
        rungeKutta(x, y, z, vx, vy, vz, m, r, C, CL, dt);
    }

    double V_xy = sqrt(vx * vx + vy * vy);

    // 与水平面的夹角（弧度），使用 atan2 保证象限正确
    double angle_with_x = atan2(vz, V_xy);  // 注意参数顺序 atan2(y, x)

    // 转为角度（0~360）
    double angle_with_x_deg = angle_with_x * 180.0 / 3.14159265358979;
    if (angle_with_x_deg < 0) angle_with_x_deg += 360.0;
    double ANG = angle_with_x_deg;

    // 返回 z 方向速度分量
    return vz;
}

// ===== 俯仰角二分求解 =====

// 在给定发射角下，计算目标 x 位置处的高度
double evaluateHeightAtTarget(double x_target, double V0, double theta, double phi, double m, double r, double C, double CL) {
    double dt = 0.001;
    double x = 0, y = 0, z = 0;
    double vx = V0 * cos(theta) * cos(phi);
    double vy = V0 * cos(theta) * sin(phi);
    double vz = V0 * sin(theta);

    int step_count = 0;
    int max_steps = 300000;

    while (x < x_target && step_count < max_steps) {
        rungeKutta(x, y, z, vx, vy, vz, m, r, C, CL, dt);
        step_count++;
    }

    return z;
}

// 使用二分法搜索发射角，使目标 x 位置处高度逼近 z_target
double calculateLaunchAngleBisection(double x_target, double z_target, double V0, double phi, double m, double r, double C, double CL) {
    double theta_low = -M_PI / 4.0;
    double theta_high = M_PI / 4.0;

    double epsilon = 0.001;
    int max_iter = 100;
    double best_theta = theta_low;

    for (int i = 0; i < max_iter; ++i) {
        double theta_mid = (theta_low + theta_high) / 2.0;
        double z_mid = evaluateHeightAtTarget(x_target, V0, theta_mid, phi, m, r, C, CL);
        double error = z_mid - z_target;

        if (std::abs(error) < epsilon) {
            return theta_mid;
        }

        if (error < 0) {
            theta_low = theta_mid;
        }
        else {
            theta_high = theta_mid;
        }

        best_theta = theta_mid;
    }

    return best_theta;
}