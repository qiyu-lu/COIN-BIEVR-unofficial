#ifndef BIEVR_LIO_PREPROCESS_H_
#define BIEVR_LIO_PREPROCESS_H_

#include "bievr_lio/bievr_map.h"
#include "bievr_lio/common.h"
#include "bievr_lio/intensity.h"

namespace bievr {

struct PreprocessConfig {
  size_t n_samples = 1000;
  bool informed_sampling = true;
  double min_range = 0.5;              // meters
  double max_range = 100.0;            // meters
  double downsample_resolution = 0.1;  // meters
};

// `indices` optionally returns the index in points_raw of every downsampled point.
void voxelDownsample(const Pointcloud& points_raw, Pointcloud& points_down, double voxel_size,
                     std::vector<size_t>* indices = nullptr);

// `coarse_indices` / `fine_indices` optionally return the index in points_raw of every sampled
// point.
void sampleInformed(const BIEVRMap& map, const Transform& T_W_L, const Pointcloud& points_raw,
                    Pointcloud& points_coarse, Pointcloud& points_fine, double voxel_size,
                    size_t n_samples, std::vector<size_t>* coarse_indices = nullptr,
                    std::vector<size_t>* fine_indices = nullptr);

// Outcome of the degeneracy analysis behind the intensity point sampling.
struct IntensitySamplingInfo {
  V3 eigenvalues = V3::Zero();       // of the observed voxel normal distribution, ascending
  M3 eigenvectors = M3::Identity();  // matching eigenvectors as columns (map frame)
  int num_target_directions = 0;     // geometrically uninformative directions (0, 1 or 2)
  size_t num_observed_voxels = 0;
  std::vector<size_t> intensity_voxels;  // hashes of the selected intensity voxels
};

// Map-informed intensity point sampling. Finds the geometrically underconstrained directions
// from the normals of the map voxels the cloud observes under the registration prior T_W_L, and
// selects the voxels whose intensity maps are most informative along these directions. Returns
// the indices of the points with a valid intensity that fall into the selected voxels.
void sampleIntensity(const BIEVRMap& map, const Transform& T_W_L, const Pointcloud& points,
                     const std::vector<double>& intensities, const IntensityConfig& config,
                     std::vector<size_t>& selected, IntensitySamplingInfo* info = nullptr);

template <typename T>
concept HasStamp = requires(T t) {
  t.stamp;
  t.end_stamp;
};

template <typename PointcloudT>
void filterMinMaxRange(const PointcloudT& points_raw, PointcloudT& points_filtered,
                       const double min_range = 0.0,
                       const double max_range = std::numeric_limits<double>::max()) {
  bool filter_min = (min_range > 0.0);
  bool filter_max = (max_range < std::numeric_limits<double>::max());

  if (!filter_min && !filter_max) {
    points_filtered = points_raw;
    return;
  }

  // When a filter is disabled its bound is a no-op: min_range defaults to 0
  // (range_squared >= 0 always), and max_range defaults to double max whose
  // square overflows to +inf (range_squared <= inf always).
  const double min_range_squared = min_range * min_range;
  const double max_range_squared = max_range * max_range;

  size_t n_points = points_raw.size();
  std::vector<char> point_selected(n_points, 0);
  tbb::parallel_for(
      tbb::blocked_range<size_t>(0, n_points), [&](const tbb::blocked_range<size_t>& r) {
        for (size_t i = r.begin(); i != r.end(); ++i) {
          const auto& point = points_raw[i];
          double range_squared =
              point.x() * point.x() + point.y() * point.y() + point.z() * point.z();
          if (range_squared >= min_range_squared && range_squared <= max_range_squared) {
            point_selected[i] = 1;
          }
        }
      });

  // Extract selected point indices
  std::vector<size_t> selected_indices;
  selected_indices.reserve(n_points);
  for (size_t i = 0; i < n_points; ++i) {
    if (point_selected[i] == 1) {
      selected_indices.push_back(i);
    }
  }

  points_filtered.resize(selected_indices.size());
  tbb::parallel_for(tbb::blocked_range<size_t>(0, selected_indices.size()),
                    [&](const tbb::blocked_range<size_t>& r) {
                      for (size_t i = r.begin(); i != r.end(); ++i) {
                        points_filtered[i] = points_raw[selected_indices[i]];
                      }
                    });

  if constexpr (HasStamp<PointcloudT>) {
    points_filtered.stamp = points_raw.stamp;
    points_filtered.end_stamp = points_raw.end_stamp;
  }
}

}  // namespace bievr
#endif  // BIEVR_LIO_PREPROCESS_H_