#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <opencv2/opencv.hpp>

#include <array>
#include <vector>
#include <string>
#include <cstdint>
#include <limits>

// 直接复用 FramePacket
#include "time_associator.hpp"

namespace hero_pkg::aim
{

struct ExtrinsicProjectionConfig
{
  // intrinsics
  double fx{0.0};
  double fy{0.0};
  double cx{0.0};
  double cy{0.0};

  // distortion
  double k1{0.0};
  double k2{0.0};
  double p1{0.0};
  double p2{0.0};

  // lidar -> camera
  std::array<double, 9> extrinsic_r{};
  std::array<double, 3> extrinsic_t{};

  std::string interp{"bilinear"};

  int armor_class_id{0};
  double armor_bbox_min_conf{0.0};

  bool enable_rgb_sampling{true};
  bool enable_armor_score{true};

  double min_projected_depth{0.1};

  // bbox扩大比例，例如 0.1 表示四周各扩 10%
  double bbox_expand_ratio{0.0};
};

struct ArmorBBox
{
  cv::Rect box;
  float confidence{0.0f};
  int detection_index{-1};
};

struct ProjectedPoint
{
  float x_lidar{0.0f};
  float y_lidar{0.0f};
  float z_lidar{0.0f};
  float intensity{0.0f};

  float x_cam{0.0f};
  float y_cam{0.0f};
  float z_cam{0.0f};

  float u{0.0f};
  float v{0.0f};

  uint8_t r{100};
  uint8_t g{100};
  uint8_t b{100};

  float armor_score{0.0f};
  int armor_box_index{-1};

  bool finite_point{false};
  bool front_of_camera{false};
  bool valid_projection{false};
};

struct ProjectionResult
{
  bool ok{false};

  rclcpp::Time cloud_stamp{0, 0, RCL_ROS_TIME};
  rclcpp::Time image_stamp{0, 0, RCL_ROS_TIME};

  std::vector<ProjectedPoint> points;
  std::vector<ArmorBBox> armor_boxes;

  size_t num_input_points{0};
  size_t num_front_points{0};
  size_t num_valid_projected_points{0};
  size_t num_armor_points{0};
};

class ExtrinsicProjection
{
public:
  explicit ExtrinsicProjection(const ExtrinsicProjectionConfig& cfg);

  ProjectionResult project(
    const sensor_msgs::msg::PointCloud2& cloud_msg,
    const FramePacket& frame) const;

private:
  bool wrapImage(
    const FramePacket& frame,
    cv::Mat& out,
    bool& is_rgb) const;

  void sampleColor(
    const cv::Mat& img,
    bool is_rgb,
    float u,
    float v,
    uint8_t& r,
    uint8_t& g,
    uint8_t& b) const;

  std::vector<ArmorBBox> extractArmorBoxes(
    const std::vector<DetectionResult>& detections,
    int img_w,
    int img_h) const;

  cv::Rect expandBox(
    const cv::Rect& box,
    int img_w,
    int img_h,
    double ratio) const;

  int hitArmorBox(
    float u,
    float v,
    const std::vector<ArmorBBox>& armor_boxes) const;

private:
  double fx_{0}, fy_{0}, cx_{0}, cy_{0};
  double k1_{0}, k2_{0}, p1_{0}, p2_{0};

  std::array<double, 9> r_{};
  std::array<double, 3> t_{};

  std::string interp_{"bilinear"};

  int armor_class_id_{0};
  double armor_bbox_min_conf_{0.0};

  bool enable_rgb_sampling_{true};
  bool enable_armor_score_{true};

  double min_projected_depth_{0.1};
  double bbox_expand_ratio_{0.0};
};

}  // namespace hero_pkg::aim
