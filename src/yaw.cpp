#include "yaw.h"
#include <cmath>

// 计算发射器与目标之间的水平距离和垂直距离
void calculateDistance(double target_x, double target_y, double target_z, 
    double lidar_x, double lidar_y, double lidar_z,
    double& horizontal_distance, double& vertical_distance) {
    double dx = target_x - lidar_x;
    double dy = target_y - lidar_y;
    double dz = target_z - lidar_z;
    horizontal_distance = std::sqrt(dx * dx + dy * dy);
    vertical_distance = dz;
}


// 使用 LiDAR 目标点计算需要补偿的 yaw 角，使发射器对准目标。
// 实现思路：
// 1. 以发射器原点为参考，计算目标在水平面的相对位移 (dx, dy)。
// 2. 在 xy 平面使用 atan2(dy, dx) 得到相对 +x 方向的夹角。
// 返回值为弧度（radian），正值表示朝 +y 方向旋转。
double calculateDeltaYawwithLidar(double target_x, double target_y, double target_z,
    double lidar_x, double lidar_y, double lidar_z) {
    // 目标相对发射器的位置
    double dx = target_x - lidar_x;
    double dy = target_y - lidar_y;
	// double dz = target_z - lidar_z; // 垂直分量对 yaw 无影响
	// 若目标在 xy 平面的投影为原点，返回 0 避免无意义旋转
	const double eps = 1e-12;
	if (std::abs(dx) < eps && std::abs(dy) < eps) {
		return 0.0;
	}

    // 返回弧度，atan2 自动处理象限
    return std::atan2(dy, dx);
}

// 基于相机与发射器外参估计 yaw 补偿角。
// 返回值为弧度。
// 说明：当前实现使用几何近似，未直接融合像素反投影结果。
double calculateDeltaYawwithCamera(
	double camera_x, double camera_y, double camera_z, // 相机相对发射器的平移外参
	double distance, // 目标到发射器的距离（米）
	double fx, double fy, double cx, double cy, double k1, double k2, double p1, double p2, // 相机内参与畸变参数
	double bbox_mid // 目标框中心在图像中的水平像素位置
) {
    // 假设：目标距离是沿发射器前向轴测得。
    // camera_x/camera_y/camera_z 表示相机到发射器坐标系的平移外参 T_cl。
    // 发射器坐标系下目标近似为 P_l = (distance, 0, 0)。
    // 相机坐标系下位置近似 P_c = P_l + T_cl，从而 yaw = atan2(P_c.y, P_c.x)。

    // 输入有效性检查
    if (distance <= 0.0) {
        return 0.0;
    }

    // 近似目标在相机坐标系中的位置（平移直接叠加）
    double px_cam = distance + camera_x; // P_c.x
    double py_cam = camera_y;            // P_c.y
    // double pz_cam = camera_z;         // P_c.z 不参与 yaw

    const double eps = 1e-12;
    if (std::abs(px_cam) < eps && std::abs(py_cam) < eps) {
        return 0.0;
    }

    // 直接返回几何 yaw
    double yaw = std::atan2(py_cam, px_cam);

    // 可选：如需结合像素位置做细调，可使用下述占位逻辑继续完善
    // double normalized_x = (bbox_mid - cx) / fx; // 归一化像平面坐标
    // double x_from_pixel = normalized_x * std::sqrt(px_cam*px_cam + py_cam*py_cam); // 估计横向偏移
    // // TODO: 融合像素偏移与几何 yaw（当前未实现）

    return yaw;
}