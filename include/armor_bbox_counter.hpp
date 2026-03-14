#pragma once

#include <cstddef>
#include <vector>

#include "point_cloud_refine_types.hpp"

namespace hero_pkg::aim
{

struct ArmorBBoxCountResult
{
  bool ok{false};
  std::size_t num_armor_points{0};
  std::vector<std::size_t> num_points_in_armor_boxes;
  std::vector<std::vector<std::size_t>> point_indices_in_armor_boxes;
};

class ArmorBBoxCounter
{
public:
  ArmorBBoxCountResult count(const PointCloudRefineInput& input) const;
};

}  // namespace hero_pkg::aim
