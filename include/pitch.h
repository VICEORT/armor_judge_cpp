#pragma once

#define _USE_MATH_DEFINES
#include <cmath>
#include <utility>

// 供其他模块读取的全局量
extern double max_range;

double airDensity(double z);

void rungeKutta(double& x, double& y, double& z,
    double& vx, double& vy, double& vz,
    double m, double r, double C, double CL, double dt);

std::pair<double, double> calculateHeightAndLateralDisplacementRK4(
    double x_target, double V0, double theta, double phi,
    double m, double r, double C, double CL);

double calculateHeightAndLateralDisplacementRK41(
    double x_target, double V0, double theta, double phi,
    double m, double r, double C, double CL);

double calculateHeightAndLateralDisplacementRK42(
    double x_target, double V0, double theta, double phi,
    double m, double r, double C, double CL);

double evaluateHeightAtTarget(
    double x_target, double V0, double theta, double phi,
    double m, double r, double C, double CL);

double calculateLaunchAngleBisection(
    double x_target, double z_target, double V0, double phi,
    double m, double r, double C, double CL);
