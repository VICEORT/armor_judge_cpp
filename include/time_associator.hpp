#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <opencv2/opencv.hpp>

#include <deque>
#include <vector>
#include <mutex>
#include <optional>
#include <limits>
#include <algorithm>
#include <numeric>
#include <string>

#include "Detection.h"

namespace hero_pkg::aim
{

struct FramePacket
{
  rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
  sensor_msgs::msg::Image::SharedPtr image_msg;
  cv::Mat image_mat;  // 可选；若 core 已提前转好可直接填
  bool has_cv_mat{false};

  std::vector<DetectionResult> detections;
  bool detections_ready{false};

  uint64_t frame_id{0};
};

struct TimeAssociatorConfig
{
  size_t buffer_size{200};

  double integrate_window{0.5};
  double camera_fps{10.0};

  // < 0 时自动推导
  double max_dt{-1.0};
  double warn_gap_image{-1.0};
  double warn_gap_cloud{-1.0};

  int diag_every_n{50};
  int offset_window{80};
  bool store_stamp_sorted{true};
};

struct MatchResult
{
  bool matched{false};

  FramePacket frame;
  double signed_dt{0.0};  // img_stamp - cloud_stamp
  double abs_dt{std::numeric_limits<double>::infinity()};

  size_t buffer_size{0};
};

class TimeAssociator
{
public:
  explicit TimeAssociator(const TimeAssociatorConfig& cfg);

  void pushFrame(const FramePacket& frame);
  std::optional<MatchResult> match(const rclcpp::Time& cloud_stamp);

  size_t bufferSize() const;
  void clear();

  double maxDt() const { return max_dt_; }
  double warnGapImage() const { return warn_gap_img_; }
  double warnGapCloud() const { return warn_gap_cloud_; }

private:
  struct DiagnosticsSnapshot
  {
    int img_cnt{0};
    int cloud_cnt{0};

    int img_backjump_cnt{0};
    int cloud_backjump_cnt{0};

    double img_gap_min{std::numeric_limits<double>::infinity()};
    double img_gap_max{0.0};
    double cloud_gap_min{std::numeric_limits<double>::infinity()};
    double cloud_gap_max{0.0};

    double img_gap_med{std::numeric_limits<double>::quiet_NaN()};
    double cloud_gap_med{std::numeric_limits<double>::quiet_NaN()};

    double signed_dt_med{std::numeric_limits<double>::quiet_NaN()};
    double abs_dt_med{std::numeric_limits<double>::quiet_NaN()};
  };

private:
  static double medianOf(std::vector<double> v);
  static double medianOfDeque(const std::deque<double>& d);
  static void pushWindow(std::deque<double>& win, double x, int max_n);

  void updateImageDiagnostics(const rclcpp::Time& t_img);
  void updateCloudDiagnostics(const rclcpp::Time& t_cloud);
  void maybePrintImageDiagnostics();
  void maybePrintCloudDiagnostics();
  DiagnosticsSnapshot buildDiagnosticsSnapshot() const;

private:
  mutable std::mutex mutex_;
  std::deque<FramePacket> buf_;

  size_t buf_max_{200};
  double max_dt_{0.1};

  int diag_every_n_{50};
  int offset_window_{80};
  double warn_gap_img_{0.08};
  double warn_gap_cloud_{0.20};
  bool store_stamp_sorted_{true};

  // image diagnostics
  rclcpp::Time last_img_stamp_{0, 0, RCL_ROS_TIME};
  int img_cnt_{0};
  int img_backjump_cnt_{0};
  double img_gap_min_{std::numeric_limits<double>::infinity()};
  double img_gap_max_{0.0};
  std::vector<double> img_gaps_;

  // cloud diagnostics
  rclcpp::Time last_cloud_stamp_{0, 0, RCL_ROS_TIME};
  int cloud_cnt_{0};
  int cloud_backjump_cnt_{0};
  double cloud_gap_min_{std::numeric_limits<double>::infinity()};
  double cloud_gap_max_{0.0};
  std::vector<double> cloud_gaps_;

  // cross-sensor diagnostics
  std::deque<double> signed_dt_window_;
  std::deque<double> abs_dt_window_;
};

}  // namespace hero_pkg::aim
