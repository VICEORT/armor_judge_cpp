#pragma once

void calculateDistance(double target_x, double target_y, double target_z,
    double lidar_x, double lidar_y, double lidar_z,
    double& horizontal_distance, double& vertical_distance);

double calculateDeltaYawwithLidar(double target_x, double target_y, double target_z,
    double lidar_x, double lidar_y, double lidar_z);

double calculateDeltaYawwithCamera(
    double camera_x, double camera_y, double camera_z,
    double distance,
    double fx, double fy, double cx, double cy,
    double k1, double k2, double p1, double p2,
    double bbox_mid);
