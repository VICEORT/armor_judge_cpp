#pragma once

#include <cstddef>
#include <limits>
#include <vector>

#include "point_cloud_refine_types.hpp"

namespace hero_pkg::aim
{

struct CloudRefineConfig
{
  double min_distance{0.05};
  double max_distance{12.0};

  bool enable_mad_filter{true};
  double mad_scale{3.0};

  std::size_t min_points_for_clustering{15};

  double cluster_tolerance{0.08};
  int min_cluster_size{5};
  int max_cluster_size{100000};
};

struct CloudRefineResult
{
  bool ok{false};

  std::size_t num_input_points{0};
  std::size_t num_after_outlier_filter{0};
  std::size_t num_after_cluster{0};

  bool clustering_applied{false};

  std::vector<std::size_t> selected_indices;

  double mean_distance{std::numeric_limits<double>::quiet_NaN()};

  // ★ 新增：均值中心坐标
  double mean_x{std::numeric_limits<double>::quiet_NaN()};
  double mean_y{std::numeric_limits<double>::quiet_NaN()};
  double mean_z{std::numeric_limits<double>::quiet_NaN()};

};

class CloudRefine
{
public:
  explicit CloudRefine(const CloudRefineConfig& cfg);

  CloudRefineResult process(const PointCloudRefineInput& input) const;

private:
  static double median(std::vector<double> values);

private:
  CloudRefineConfig cfg_;
};

}  // namespace hero_pkg::aim
