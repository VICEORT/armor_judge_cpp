#include "time_associator.hpp"

#include <cmath>
#include <iostream>

namespace hero_pkg::aim
{

TimeAssociator::TimeAssociator(const TimeAssociatorConfig& cfg)
: buf_max_(cfg.buffer_size),
  diag_every_n_(cfg.diag_every_n),
  offset_window_(cfg.offset_window),
  store_stamp_sorted_(cfg.store_stamp_sorted)
{
  const double img_period = (cfg.camera_fps > 1e-6) ? (1.0 / cfg.camera_fps) : 0.1;

  warn_gap_cloud_ = cfg.warn_gap_cloud;
  warn_gap_img_ = cfg.warn_gap_image;
  max_dt_ = cfg.max_dt;

  if (warn_gap_cloud_ < 0.0) {
    warn_gap_cloud_ = 2.0 * cfg.integrate_window;
  }
  if (warn_gap_img_ < 0.0) {
    warn_gap_img_ = 3.0 * img_period;
  }
  if (max_dt_ < 0.0) {
    max_dt_ = std::min(0.5 * cfg.integrate_window, 1.2 * img_period);
    max_dt_ = std::max(max_dt_, 0.03);
  }
}

void TimeAssociator::pushFrame(const FramePacket& frame)
{
  std::lock_guard<std::mutex> lock(mutex_);

  updateImageDiagnostics(frame.stamp);

  if (store_stamp_sorted_) {
    auto it = std::upper_bound(
      buf_.begin(), buf_.end(), frame.stamp,
      [](const rclcpp::Time& t, const FramePacket& x) { return t < x.stamp; });
    buf_.insert(it, frame);
  } else {
    buf_.push_back(frame);
  }

  while (buf_.size() > buf_max_) {
    buf_.pop_front();
  }

  maybePrintImageDiagnostics();
}

std::optional<MatchResult> TimeAssociator::match(const rclcpp::Time& cloud_stamp)
{
  std::lock_guard<std::mutex> lock(mutex_);

  updateCloudDiagnostics(cloud_stamp);
  maybePrintCloudDiagnostics();

  if (buf_.empty()) {
    return std::nullopt;
  }

  size_t best_idx = 0;
  double best_abs = std::numeric_limits<double>::infinity();
  double best_signed = 0.0;

  for (size_t i = 0; i < buf_.size(); ++i) {
    const double signed_dt = (buf_[i].stamp - cloud_stamp).seconds();
    const double abs_dt = std::abs(signed_dt);
    if (abs_dt < best_abs) {
      best_abs = abs_dt;
      best_signed = signed_dt;
      best_idx = i;
    }
  }

  pushWindow(signed_dt_window_, best_signed, offset_window_);
  pushWindow(abs_dt_window_, best_abs, offset_window_);

  if (best_abs > max_dt_) {
    return std::nullopt;
  }

  MatchResult result;
  result.matched = true;
  result.frame = buf_[best_idx];
  result.signed_dt = best_signed;
  result.abs_dt = best_abs;
  result.buffer_size = buf_.size();
  return result;
}

size_t TimeAssociator::bufferSize() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return buf_.size();
}

void TimeAssociator::clear()
{
  std::lock_guard<std::mutex> lock(mutex_);
  buf_.clear();
  signed_dt_window_.clear();
  abs_dt_window_.clear();
}

double TimeAssociator::medianOf(std::vector<double> v)
{
  if (v.empty()) {
    return std::numeric_limits<double>::quiet_NaN();
  }

  std::nth_element(v.begin(), v.begin() + v.size() / 2, v.end());
  double m = v[v.size() / 2];
  if (v.size() % 2 == 0) {
    auto it = std::max_element(v.begin(), v.begin() + v.size() / 2);
    m = (m + *it) * 0.5;
  }
  return m;
}

double TimeAssociator::medianOfDeque(const std::deque<double>& d)
{
  if (d.empty()) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  std::vector<double> v(d.begin(), d.end());
  return medianOf(std::move(v));
}

void TimeAssociator::pushWindow(std::deque<double>& win, double x, int max_n)
{
  win.push_back(x);
  while (static_cast<int>(win.size()) > max_n) {
    win.pop_front();
  }
}

void TimeAssociator::updateImageDiagnostics(const rclcpp::Time& t_img)
{
  if (last_img_stamp_.nanoseconds() != 0) {
    const double gap = (t_img - last_img_stamp_).seconds();
    if (gap < 0.0) {
      img_backjump_cnt_++;
    } else {
      img_gap_min_ = std::min(img_gap_min_, gap);
      img_gap_max_ = std::max(img_gap_max_, gap);
      img_gaps_.push_back(gap);
    }
  }
  last_img_stamp_ = t_img;
  img_cnt_++;
}

void TimeAssociator::updateCloudDiagnostics(const rclcpp::Time& t_cloud)
{
  if (last_cloud_stamp_.nanoseconds() != 0) {
    const double gap = (t_cloud - last_cloud_stamp_).seconds();
    if (gap < 0.0) {
      cloud_backjump_cnt_++;
    } else {
      cloud_gap_min_ = std::min(cloud_gap_min_, gap);
      cloud_gap_max_ = std::max(cloud_gap_max_, gap);
      cloud_gaps_.push_back(gap);
    }
  }
  last_cloud_stamp_ = t_cloud;
  cloud_cnt_++;
}

TimeAssociator::DiagnosticsSnapshot TimeAssociator::buildDiagnosticsSnapshot() const
{
  DiagnosticsSnapshot s;
  s.img_cnt = img_cnt_;
  s.cloud_cnt = cloud_cnt_;
  s.img_backjump_cnt = img_backjump_cnt_;
  s.cloud_backjump_cnt = cloud_backjump_cnt_;

  s.img_gap_min = img_gap_min_;
  s.img_gap_max = img_gap_max_;
  s.cloud_gap_min = cloud_gap_min_;
  s.cloud_gap_max = cloud_gap_max_;

  s.img_gap_med = medianOf(img_gaps_);
  s.cloud_gap_med = medianOf(cloud_gaps_);
  s.signed_dt_med = medianOfDeque(signed_dt_window_);
  s.abs_dt_med = medianOfDeque(abs_dt_window_);
  return s;
}

void TimeAssociator::maybePrintImageDiagnostics()
{
  if (diag_every_n_ <= 0) {
    return;
  }
  if (img_cnt_ % diag_every_n_ != 0) {
    return;
  }

  const auto s = buildDiagnosticsSnapshot();
  std::cout
    << "[TimeAssociator][IMG] cnt=" << s.img_cnt
    << " gap_min=" << (std::isfinite(s.img_gap_min) ? s.img_gap_min : 0.0)
    << " gap_med=" << (std::isfinite(s.img_gap_med) ? s.img_gap_med : 0.0)
    << " gap_max=" << s.img_gap_max
    << " backjump=" << s.img_backjump_cnt
    << std::endl;

  img_gaps_.clear();
  img_gap_min_ = std::numeric_limits<double>::infinity();
  img_gap_max_ = 0.0;
  img_backjump_cnt_ = 0;
}

void TimeAssociator::maybePrintCloudDiagnostics()
{
  const int cloud_diag_n = std::max(5, diag_every_n_ / 5);
  if (cloud_diag_n <= 0) {
    return;
  }
  if (cloud_cnt_ % cloud_diag_n != 0) {
    return;
  }

  const auto s = buildDiagnosticsSnapshot();
  std::cout
    << "[TimeAssociator][CLOUD] cnt=" << s.cloud_cnt
    << " gap_min=" << (std::isfinite(s.cloud_gap_min) ? s.cloud_gap_min : 0.0)
    << " gap_med=" << (std::isfinite(s.cloud_gap_med) ? s.cloud_gap_med : 0.0)
    << " gap_max=" << s.cloud_gap_max
    << " backjump=" << s.cloud_backjump_cnt
    << " signed_dt_med=" << (std::isfinite(s.signed_dt_med) ? s.signed_dt_med : 0.0)
    << " abs_dt_med=" << (std::isfinite(s.abs_dt_med) ? s.abs_dt_med : 0.0)
    << std::endl;

  cloud_gaps_.clear();
  cloud_gap_min_ = std::numeric_limits<double>::infinity();
  cloud_gap_max_ = 0.0;
  cloud_backjump_cnt_ = 0;
}

}  // namespace hero_pkg::aim