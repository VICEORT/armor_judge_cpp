#pragma once

#include <cstddef>
#include <vector>

#include <opencv2/core.hpp>

namespace hero_pkg::aim
{

// 点云精细化模块输入类型（与外参反投影解耦）
struct ArmorBBoxInfo
{
  cv::Rect box;
  float confidence{0.0F};
  int detection_index{-1};
};

struct ProjectedPointInfo
{
  float x_cam{0.0F};
  float y_cam{0.0F};
  float z_cam{0.0F};

  float u{0.0F};
  float v{0.0F};

  float armor_score{0.0F};
  int armor_box_index{-1};

  bool finite_point{false};
  bool front_of_camera{false};
  bool valid_projection{false};
};

struct PointCloudRefineInput
{
  bool ok{false};
  std::vector<ProjectedPointInfo> points;
  std::vector<ArmorBBoxInfo> armor_boxes;
};

}  // namespace hero_pkg::aim
