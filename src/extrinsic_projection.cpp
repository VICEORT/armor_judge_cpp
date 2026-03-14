#include "extrinsic_projection.hpp"

#include <cmath>
#include <stdexcept>

namespace hero_pkg::aim
{

ExtrinsicProjection::ExtrinsicProjection(const ExtrinsicProjectionConfig& cfg)
: fx_(cfg.fx),
  fy_(cfg.fy),
  cx_(cfg.cx),
  cy_(cfg.cy),
  k1_(cfg.k1),
  k2_(cfg.k2),
  p1_(cfg.p1),
  p2_(cfg.p2),
  r_(cfg.extrinsic_r),
  t_(cfg.extrinsic_t),
  interp_(cfg.interp),
  armor_class_id_(cfg.armor_class_id),
  armor_bbox_min_conf_(cfg.armor_bbox_min_conf),
  enable_rgb_sampling_(cfg.enable_rgb_sampling),
  enable_armor_score_(cfg.enable_armor_score),
  min_projected_depth_(cfg.min_projected_depth),
  bbox_expand_ratio_(cfg.bbox_expand_ratio)
{
}

ProjectionResult ExtrinsicProjection::project(
  const sensor_msgs::msg::PointCloud2& cloud_msg,
  const FramePacket& frame) const
{
  ProjectionResult result;
  result.cloud_stamp = rclcpp::Time(cloud_msg.header.stamp);
  result.image_stamp = frame.stamp;

  cv::Mat img;
  bool is_rgb = false;
  if (!wrapImage(frame, img, is_rgb)) {
    return result;
  }

  const int img_w = img.cols;
  const int img_h = img.rows;

  result.armor_boxes = extractArmorBoxes(frame.detections, img_w, img_h);

  pcl::PointCloud<pcl::PointXYZI> cloud;
  pcl::fromROSMsg(cloud_msg, cloud);

  result.num_input_points = cloud.size();
  result.points.reserve(cloud.size());

  for (const auto& pt : cloud.points) {
    ProjectedPoint out;
    out.x_lidar = pt.x;
    out.y_lidar = pt.y;
    out.z_lidar = pt.z;
    out.intensity = pt.intensity;

    out.r = 100;
    out.g = 100;
    out.b = 100;

    out.finite_point =
      std::isfinite(pt.x) && std::isfinite(pt.y) && std::isfinite(pt.z);

    if (!out.finite_point) {
      result.points.push_back(out);
      continue;
    }

    const double xc = r_[0] * pt.x + r_[1] * pt.y + r_[2] * pt.z + t_[0];
    const double yc = r_[3] * pt.x + r_[4] * pt.y + r_[5] * pt.z + t_[1];
    const double zc = r_[6] * pt.x + r_[7] * pt.y + r_[8] * pt.z + t_[2];

    out.x_cam = static_cast<float>(xc);
    out.y_cam = static_cast<float>(yc);
    out.z_cam = static_cast<float>(zc);

    if (zc <= min_projected_depth_) {
      result.points.push_back(out);
      continue;
    }

    out.front_of_camera = true;
    result.num_front_points++;

    const double invz = 1.0 / zc;
    const double xn = xc * invz;
    const double yn = yc * invz;

    // Brown-Conrady distortion
    const double r2 = xn * xn + yn * yn;
    const double r4 = r2 * r2;
    const double rad = 1.0 + k1_ * r2 + k2_ * r4;

    const double x_d = xn * rad + 2.0 * p1_ * xn * yn + p2_ * (r2 + 2.0 * xn * xn);
    const double y_d = yn * rad + p1_ * (r2 + 2.0 * yn * yn) + 2.0 * p2_ * xn * yn;

    const float u = static_cast<float>(fx_ * x_d + cx_);
    const float v = static_cast<float>(fy_ * y_d + cy_);

    out.u = u;
    out.v = v;

    if (u >= 0.0f && u < static_cast<float>(img_w) &&
        v >= 0.0f && v < static_cast<float>(img_h)) {
      out.valid_projection = true;
      result.num_valid_projected_points++;

      if (enable_rgb_sampling_) {
        sampleColor(img, is_rgb, u, v, out.r, out.g, out.b);
      }

      if (enable_armor_score_) {
        const int hit_idx = hitArmorBox(u, v, result.armor_boxes);
        out.armor_box_index = hit_idx;
        out.armor_score = (hit_idx >= 0) ? 1.0f : 0.0f;
        if (out.armor_score > 0.5f) {
          result.num_armor_points++;
        }
      }
    }

    result.points.push_back(out);
  }

  result.ok = true;
  return result;
}

bool ExtrinsicProjection::wrapImage(
  const FramePacket& frame,
  cv::Mat& out,
  bool& is_rgb) const
{
  if (frame.has_cv_mat && !frame.image_mat.empty()) {
    out = frame.image_mat;
    is_rgb = true;  // 若 core 已传 cv::Mat，这里默认是 RGB/BGR 三通道图
    if (out.channels() == 1) {
      is_rgb = false;
    }
    return true;
  }

  if (!frame.image_msg) {
    return false;
  }

  const auto& msg = *frame.image_msg;

  if (msg.encoding == "rgb8") {
    out = cv::Mat(
      static_cast<int>(msg.height),
      static_cast<int>(msg.width),
      CV_8UC3,
      const_cast<uint8_t*>(msg.data.data()),
      static_cast<size_t>(msg.step)).clone();
    is_rgb = true;
    return true;
  }

  if (msg.encoding == "bgr8") {
    out = cv::Mat(
      static_cast<int>(msg.height),
      static_cast<int>(msg.width),
      CV_8UC3,
      const_cast<uint8_t*>(msg.data.data()),
      static_cast<size_t>(msg.step)).clone();
    is_rgb = false;
    return true;
  }

  if (msg.encoding == "mono8") {
    out = cv::Mat(
      static_cast<int>(msg.height),
      static_cast<int>(msg.width),
      CV_8UC1,
      const_cast<uint8_t*>(msg.data.data()),
      static_cast<size_t>(msg.step)).clone();
    is_rgb = false;
    return true;
  }

  return false;
}

void ExtrinsicProjection::sampleColor(
  const cv::Mat& img,
  bool is_rgb,
  float u,
  float v,
  uint8_t& r,
  uint8_t& g,
  uint8_t& b) const
{
  const int w = img.cols;
  const int h = img.rows;

  if (interp_ == "nearest") {
    const int x = std::max(0, std::min(w - 1, static_cast<int>(std::lround(u))));
    const int y = std::max(0, std::min(h - 1, static_cast<int>(std::lround(v))));

    if (img.channels() == 3) {
      const auto& pix = img.at<cv::Vec3b>(y, x);
      if (is_rgb) {
        r = pix[0]; g = pix[1]; b = pix[2];
      } else {
        b = pix[0]; g = pix[1]; r = pix[2];
      }
    } else {
      const uint8_t val = img.at<uint8_t>(y, x);
      r = g = b = val;
    }
    return;
  }

  const int x0 = std::max(0, std::min(w - 1, static_cast<int>(std::floor(u))));
  const int y0 = std::max(0, std::min(h - 1, static_cast<int>(std::floor(v))));
  const int x1 = std::max(0, std::min(w - 1, x0 + 1));
  const int y1 = std::max(0, std::min(h - 1, y0 + 1));

  const float ax = u - static_cast<float>(x0);
  const float ay = v - static_cast<float>(y0);

  auto lerp = [](float a, float c, float t) {
    return a + (c - a) * t;
  };

  if (img.channels() == 3) {
    const auto& p00 = img.at<cv::Vec3b>(y0, x0);
    const auto& p10 = img.at<cv::Vec3b>(y0, x1);
    const auto& p01 = img.at<cv::Vec3b>(y1, x0);
    const auto& p11 = img.at<cv::Vec3b>(y1, x1);

    auto bil = [&](int c) {
      const float v0 = lerp(static_cast<float>(p00[c]), static_cast<float>(p10[c]), ax);
      const float v1 = lerp(static_cast<float>(p01[c]), static_cast<float>(p11[c]), ax);
      return static_cast<uint8_t>(std::lround(lerp(v0, v1, ay)));
    };

    const uint8_t c0 = bil(0);
    const uint8_t c1 = bil(1);
    const uint8_t c2 = bil(2);

    if (is_rgb) {
      r = c0; g = c1; b = c2;
    } else {
      b = c0; g = c1; r = c2;
    }
  } else {
    const float p00 = img.at<uint8_t>(y0, x0);
    const float p10 = img.at<uint8_t>(y0, x1);
    const float p01 = img.at<uint8_t>(y1, x0);
    const float p11 = img.at<uint8_t>(y1, x1);

    const float v0 = lerp(p00, p10, ax);
    const float v1 = lerp(p01, p11, ax);
    const uint8_t val = static_cast<uint8_t>(std::lround(lerp(v0, v1, ay)));
    r = g = b = val;
  }
}

std::vector<ArmorBBox> ExtrinsicProjection::extractArmorBoxes(
  const std::vector<DetectionResult>& detections,
  int img_w,
  int img_h) const
{
  std::vector<ArmorBBox> armor_boxes;
  armor_boxes.reserve(detections.size());

  for (size_t i = 0; i < detections.size(); ++i) {
    const auto& det = detections[i];
    if (det.classID != armor_class_id_) {
      continue;
    }
    if (det.confidence < armor_bbox_min_conf_) {
      continue;
    }

    ArmorBBox item;
    item.box = expandBox(det.bbox, img_w, img_h, bbox_expand_ratio_);
    item.confidence = det.confidence;
    item.detection_index = static_cast<int>(i);

    if (item.box.width > 0 && item.box.height > 0) {
      armor_boxes.push_back(item);
    }
  }

  return armor_boxes;
}

cv::Rect ExtrinsicProjection::expandBox(
  const cv::Rect& box,
  int img_w,
  int img_h,
  double ratio) const
{
  if (ratio <= 1e-9) {
    cv::Rect clipped = box;
    clipped &= cv::Rect(0, 0, img_w, img_h);
    return clipped;
  }

  const double dw = box.width * ratio;
  const double dh = box.height * ratio;

  int x = static_cast<int>(std::floor(box.x - dw));
  int y = static_cast<int>(std::floor(box.y - dh));
  int w = static_cast<int>(std::ceil(box.width + 2.0 * dw));
  int h = static_cast<int>(std::ceil(box.height + 2.0 * dh));

  cv::Rect expanded(x, y, w, h);
  expanded &= cv::Rect(0, 0, img_w, img_h);
  return expanded;
}

int ExtrinsicProjection::hitArmorBox(
  float u,
  float v,
  const std::vector<ArmorBBox>& armor_boxes) const
{
  const cv::Point p(static_cast<int>(std::floor(u)), static_cast<int>(std::floor(v)));

  for (size_t i = 0; i < armor_boxes.size(); ++i) {
    if (armor_boxes[i].box.contains(p)) {
      return static_cast<int>(i);
    }
  }
  return -1;
}

}  // namespace hero_pkg::aim