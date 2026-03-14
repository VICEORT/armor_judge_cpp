#include "armor_bbox_counter.hpp"

#include <cmath>

namespace hero_pkg::aim
{

ArmorBBoxCountResult ArmorBBoxCounter::count(const PointCloudRefineInput& input) const
{
  ArmorBBoxCountResult out;
  out.ok = input.ok;
  out.num_points_in_armor_boxes.assign(input.armor_boxes.size(), 0);
  out.point_indices_in_armor_boxes.assign(input.armor_boxes.size(), {});

  if (!input.ok || input.armor_boxes.empty() || input.points.empty()) {
    return out;
  }

  for (std::size_t point_idx = 0; point_idx < input.points.size(); ++point_idx) {
    const auto& p = input.points[point_idx];
    if (!p.finite_point || !p.front_of_camera || !p.valid_projection) {
      continue;
    }

    const int hit_idx = p.armor_box_index;
    if (hit_idx >= 0 && static_cast<std::size_t>(hit_idx) < input.armor_boxes.size()) {
      const std::size_t box_idx = static_cast<std::size_t>(hit_idx);
      out.num_points_in_armor_boxes[box_idx]++;
      out.point_indices_in_armor_boxes[box_idx].push_back(point_idx);
      continue;
    }

    const cv::Point px(
      static_cast<int>(std::floor(p.u)),
      static_cast<int>(std::floor(p.v)));

    for (std::size_t i = 0; i < input.armor_boxes.size(); ++i) {
      if (input.armor_boxes[i].box.contains(px)) {
        out.num_points_in_armor_boxes[i]++;
        out.point_indices_in_armor_boxes[i].push_back(point_idx);
        break;
      }
    }
  }

  for (std::size_t n : out.num_points_in_armor_boxes) {
    out.num_armor_points += n;
  }

  return out;
}

}  // namespace hero_pkg::aim
