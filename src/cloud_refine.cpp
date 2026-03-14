#include "cloud_refine.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

namespace hero_pkg::aim
{

CloudRefine::CloudRefine(const CloudRefineConfig& cfg)
: cfg_(cfg)
{
}

double CloudRefine::median(std::vector<double> values)
{
  if (values.empty()) {
    return std::numeric_limits<double>::quiet_NaN();
  }

  const std::size_t n = values.size();
  const std::size_t mid = n / 2;
  std::nth_element(values.begin(), values.begin() + mid, values.end());
  double m = values[mid];

  if ((n % 2U) == 0U) {
    const auto max_it = std::max_element(values.begin(), values.begin() + mid);
    m = 0.5 * (m + *max_it);
  }

  return m;
}

CloudRefineResult CloudRefine::process(const PointCloudRefineInput& input) const
{
  CloudRefineResult out;

  std::vector<std::size_t> candidate_indices;
  std::vector<double> candidate_distances;
  candidate_indices.reserve(input.points.size());
  candidate_distances.reserve(input.points.size());

  for (std::size_t i = 0; i < input.points.size(); ++i) {
    const auto& p = input.points[i];

    if (!p.finite_point || !p.front_of_camera || !p.valid_projection || p.armor_score <= 0.5F) {
      continue;
    }

    const double d = std::sqrt(
      static_cast<double>(p.x_cam) * static_cast<double>(p.x_cam) +
      static_cast<double>(p.y_cam) * static_cast<double>(p.y_cam) +
      static_cast<double>(p.z_cam) * static_cast<double>(p.z_cam));

    if (d < cfg_.min_distance || d > cfg_.max_distance || !std::isfinite(d)) {
      continue;
    }

    candidate_indices.push_back(i);
    candidate_distances.push_back(d);
  }

  out.num_input_points = candidate_indices.size();
  if (candidate_indices.empty()) {
    return out;
  }

  std::vector<std::size_t> filtered_indices;
  std::vector<double> filtered_distances;
  filtered_indices.reserve(candidate_indices.size());
  filtered_distances.reserve(candidate_distances.size());

  if (cfg_.enable_mad_filter && candidate_distances.size() >= 3U) {
    const double med = median(candidate_distances);

    std::vector<double> abs_dev;
    abs_dev.reserve(candidate_distances.size());
    for (double d : candidate_distances) {
      abs_dev.push_back(std::abs(d - med));
    }

    const double mad = median(abs_dev);
    const double robust_sigma = 1.4826 * mad;

    if (robust_sigma > 1e-9) {
      for (std::size_t i = 0; i < candidate_distances.size(); ++i) {
        const double z = std::abs(candidate_distances[i] - med) / robust_sigma;
        if (z <= cfg_.mad_scale) {
          filtered_indices.push_back(candidate_indices[i]);
          filtered_distances.push_back(candidate_distances[i]);
        }
      }
    } else {
      filtered_indices = candidate_indices;
      filtered_distances = candidate_distances;
    }
  } else {
    filtered_indices = candidate_indices;
    filtered_distances = candidate_distances;
  }

  out.num_after_outlier_filter = filtered_indices.size();
  if (filtered_indices.empty()) {
    return out;
  }

  std::vector<std::size_t> final_indices = filtered_indices;

  if (filtered_indices.size() >= cfg_.min_points_for_clustering) {
    out.clustering_applied = true;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    cloud->points.reserve(filtered_indices.size());

    for (std::size_t idx : filtered_indices) {
      const auto& p = input.points[idx];
      cloud->points.emplace_back(p.x_cam, p.y_cam, p.z_cam);
    }

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    tree->setInputCloud(cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(cfg_.cluster_tolerance);
    ec.setMinClusterSize(cfg_.min_cluster_size);
    ec.setMaxClusterSize(cfg_.max_cluster_size);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);

    if (!cluster_indices.empty()) {
      const auto best_it = std::max_element(
        cluster_indices.begin(), cluster_indices.end(),
        [](const pcl::PointIndices& a, const pcl::PointIndices& b) {
          return a.indices.size() < b.indices.size();
        });

      std::vector<std::size_t> clustered;
      clustered.reserve(best_it->indices.size());
      for (int local_id : best_it->indices) {
        if (local_id >= 0 && static_cast<std::size_t>(local_id) < filtered_indices.size()) {
          clustered.push_back(filtered_indices[static_cast<std::size_t>(local_id)]);
        }
      }

      if (!clustered.empty()) {
        final_indices = std::move(clustered);
      }
    }
  }

  out.selected_indices = final_indices;
  out.num_after_cluster = out.selected_indices.size();

  if (!out.selected_indices.empty()) {

  double sum_x = 0.0;
  double sum_y = 0.0;
  double sum_z = 0.0;
  double sum_d = 0.0;

  for (std::size_t idx : out.selected_indices) {

    const auto& p = input.points[idx];

    const double x = static_cast<double>(p.x_cam);
    const double y = static_cast<double>(p.y_cam);
    const double z = static_cast<double>(p.z_cam);

    const double d = std::sqrt(x*x + y*y + z*z);

    sum_x += x;
    sum_y += y;
    sum_z += z;
    sum_d += d;
  }

  const double n = static_cast<double>(out.selected_indices.size());

  out.mean_x = sum_x / n;
  out.mean_y = sum_y / n;
  out.mean_z = sum_z / n;

  out.mean_distance = sum_d / n;

  out.ok = true;
}

  return out;
}

}  // namespace hero_pkg::aim
