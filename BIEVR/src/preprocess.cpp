#include "bievr_lio/preprocess.h"

#include <Eigen/Eigenvalues>

#include "unordered_dense/unordered_dense.h"

namespace bievr {

namespace {

struct VoxelDownsampleEntry {
  size_t hash;
  size_t idx;
  double dist;
};

struct VoxelHashIdx {
  size_t hash;
  size_t idx;
};

struct VoxelScore {
  double score;
  size_t hash;
  size_t idx;
};

}  // namespace

void voxelDownsample(const Pointcloud& points_raw, Pointcloud& points_down, double voxel_size,
                     std::vector<size_t>* indices) {
  std::vector<VoxelDownsampleEntry> voxel_entries(points_raw.size());

  tbb::parallel_for(
      tbb::blocked_range<size_t>(0, points_raw.size()), [&](const tbb::blocked_range<size_t>& r) {
        for (size_t idx = r.begin(); idx != r.end(); ++idx) {
          const Eigen::Vector3i voxel = (points_raw[idx] / voxel_size).array().floor().cast<int>();
          Eigen::Vector3d voxel_center =
              (voxel.cast<double>() + Eigen::Vector3d::Constant(0.5)) * voxel_size;
          double dist = (points_raw[idx] - voxel_center).squaredNorm();
          voxel_entries[idx] = {hashIndexVoxel(voxel.matrix()), idx, dist};
        }
      });

  // Sort by voxel hash, then by distance to voxel center (closest first).
  tbb::parallel_sort(voxel_entries.begin(), voxel_entries.end(),
                     [](const VoxelDownsampleEntry& a, const VoxelDownsampleEntry& b) {
                       return std::tie(a.hash, a.dist) < std::tie(b.hash, b.dist);
                     });

  std::vector<size_t> selected_indices;
  selected_indices.reserve(points_raw.size());

  size_t curr_hash = std::numeric_limits<size_t>::max();
  for (const auto& entry : voxel_entries) {
    if (entry.hash != curr_hash) {
      selected_indices.push_back(entry.idx);
      curr_hash = entry.hash;
    }
  }

  points_down.resize(selected_indices.size());
  tbb::parallel_for(tbb::blocked_range<size_t>(0, selected_indices.size()),
                    [&](const tbb::blocked_range<size_t>& r) {
                      for (size_t i = r.begin(); i != r.end(); ++i) {
                        points_down[i] = points_raw[selected_indices[i]];
                      }
                    });

  if (indices) {
    *indices = std::move(selected_indices);
  }
}

void sampleInformed(const BIEVRMap& map, const Transform& T_W_L, const Pointcloud& points_raw,
                    Pointcloud& points_coarse, Pointcloud& points_fine, double voxel_size,
                    size_t n_samples, std::vector<size_t>* coarse_indices,
                    std::vector<size_t>* fine_indices) {
  if (coarse_indices) coarse_indices->clear();
  if (fine_indices) fine_indices->clear();

  if (points_raw.size() <= n_samples) {
    points_coarse.resize(points_raw.size());
    points_coarse.data().topRows(3) = points_raw.data().topRows(3);
    if (coarse_indices) {
      coarse_indices->resize(points_raw.size());
      for (size_t idx = 0; idx < points_raw.size(); ++idx) (*coarse_indices)[idx] = idx;
    }
    return;
  }

  // Lookup hash for each point based on its transformed position from the registration prior.
  std::vector<VoxelHashIdx> voxel_entries(points_raw.size());
  tbb::parallel_for(tbb::blocked_range<size_t>(0, points_raw.size()),
                    [&](const tbb::blocked_range<size_t>& r) {
                      for (size_t idx = r.begin(); idx != r.end(); ++idx) {
                        Point p_w = T_W_L.linear() * points_raw[idx] + T_W_L.translation();
                        voxel_entries[idx] = {map.hashIndex(p_w), idx};
                      }
                    });

  tbb::parallel_sort(voxel_entries.begin(), voxel_entries.end(),
                     [](const VoxelHashIdx& a, const VoxelHashIdx& b) {
                       // Add tie to ensure deterministic order for points in the same voxel, even
                       // if they have the same hash
                       return std::tie(a.hash, a.idx) < std::tie(b.hash, b.idx);
                     });

  // Extract unique hashes of observed voxels
  std::vector<VoxelHashIdx> unique_voxels;
  unique_voxels.reserve(points_raw.size());
  std::vector<bool> voxel_has_extras;
  voxel_has_extras.reserve(points_raw.size());

  for (size_t i = 0; i < voxel_entries.size(); ++i) {
    if (i == 0 || voxel_entries[i].hash != voxel_entries[i - 1].hash) {
      unique_voxels.push_back(voxel_entries[i]);
      voxel_has_extras.push_back(false);
    } else {
      voxel_has_extras.back() = true;
    }
  }

  std::vector<VoxelScore> voxel_scores(unique_voxels.size());
  tbb::parallel_for(tbb::blocked_range<size_t>(0, unique_voxels.size()),
                    [&](const tbb::blocked_range<size_t>& r) {
                      for (size_t idx = r.begin(); idx != r.end(); ++idx) {
                        const auto& entry = unique_voxels[idx];
                        auto voxel = map.getVoxel(entry.hash);
                        double score =
                            (!voxel || !voxel_has_extras[idx]) ? 0.0 : voxel->mean_img_dist_;
                        voxel_scores[idx] = {score, entry.hash, entry.idx};
                      }
                    });

  // Sort by score, descending.
  tbb::parallel_sort(voxel_scores.begin(), voxel_scores.end(),
                     [](const VoxelScore& a, const VoxelScore& b) { return a.score > b.score; });

  ankerl::unordered_dense::set<size_t> informed_voxels;
  size_t n_select = std::min(n_samples, voxel_scores.size());
  informed_voxels.reserve(n_select);

  for (size_t idx = 0; idx < n_select; ++idx) {
    informed_voxels.insert(voxel_scores[idx].hash);
  }

  // Keep all points in informed voxels
  std::vector<size_t> informed_indices;
  informed_indices.reserve(points_raw.size());
  size_t curr_hash = std::numeric_limits<size_t>::max();
  bool curr_informed = false;
  for (const auto& entry : voxel_entries) {
    if (entry.hash != curr_hash) {
      curr_hash = entry.hash;
      curr_informed = informed_voxels.contains(entry.hash);
    }
    if (curr_informed) {
      informed_indices.push_back(entry.idx);
    }
  }

  points_fine.resize(informed_indices.size());
  tbb::parallel_for(tbb::blocked_range<size_t>(0, points_fine.size()),
                    [&](const tbb::blocked_range<size_t>& r) {
                      for (size_t idx = r.begin(); idx != r.end(); ++idx) {
                        points_fine[idx] = points_raw[informed_indices[idx]];
                      }
                    });

  // Keep one point per coarse voxel for the rest
  size_t n_informed = informed_voxels.size();
  size_t n_coarse = voxel_scores.size() - n_informed;
  points_coarse.resize(n_coarse);
  tbb::parallel_for(tbb::blocked_range<size_t>(n_informed, voxel_scores.size()),
                    [&](const tbb::blocked_range<size_t>& r) {
                      for (size_t idx = r.begin(); idx != r.end(); ++idx) {
                        points_coarse[idx - n_informed] = points_raw[voxel_scores[idx].idx];
                      }
                    });

  if (coarse_indices) {
    coarse_indices->resize(n_coarse);
    for (size_t idx = n_informed; idx < voxel_scores.size(); ++idx) {
      (*coarse_indices)[idx - n_informed] = voxel_scores[idx].idx;
    }
  }
  if (fine_indices) {
    *fine_indices = std::move(informed_indices);
  }
}

void sampleIntensity(const BIEVRMap& map, const Transform& T_W_L, const Pointcloud& points,
                     const std::vector<double>& intensities, const IntensityConfig& config,
                     std::vector<size_t>& selected, IntensitySamplingInfo* info) {
  selected.clear();
  IntensitySamplingInfo local_info;
  IntensitySamplingInfo& result = info ? *info : local_info;
  result = IntensitySamplingInfo();
  if (points.empty()) return;

  // Group the points by the voxel they fall into under the registration prior.
  std::vector<VoxelHashIdx> voxel_entries(points.size());
  tbb::parallel_for(tbb::blocked_range<size_t>(0, points.size()),
                    [&](const tbb::blocked_range<size_t>& r) {
                      for (size_t idx = r.begin(); idx != r.end(); ++idx) {
                        const Point p_w = T_W_L.linear() * points[idx] + T_W_L.translation();
                        voxel_entries[idx] = {map.hashIndex(p_w), idx};
                      }
                    });
  tbb::parallel_sort(voxel_entries.begin(), voxel_entries.end(),
                     [](const VoxelHashIdx& a, const VoxelHashIdx& b) {
                       return std::tie(a.hash, a.idx) < std::tie(b.hash, b.idx);
                     });

  // Start of every voxel group in voxel_entries, restricted to the voxels observed in the map.
  struct ObservedVoxel {
    const Voxel* voxel;
    size_t begin;
    size_t end;
  };
  std::vector<ObservedVoxel> observed;
  for (size_t begin = 0; begin < voxel_entries.size();) {
    size_t end = begin + 1;
    while (end < voxel_entries.size() && voxel_entries[end].hash == voxel_entries[begin].hash) {
      ++end;
    }
    if (const Voxel* voxel = map.getVoxel(voxel_entries[begin].hash)) {
      observed.push_back({voxel, begin, end});
    }
    begin = end;
  }
  result.num_observed_voxels = observed.size();
  if (observed.empty()) return;

  // The distribution of the observed voxel normals approximates the geometric information of
  // the registration. Directions with small eigenvalues are underconstrained.
  M3 A = M3::Zero();
  for (const ObservedVoxel& entry : observed) {
    const V3 normal = entry.voxel->T_C_W_.linear().row(2);
    A += normal * normal.transpose();
  }
  const Eigen::SelfAdjointEigenSolver<M3> solver(A);
  result.eigenvalues = solver.eigenvalues();
  result.eigenvectors = solver.eigenvectors();
  // One (e.g. tunnels) or two (e.g. flat areas) unconstrained directions.
  const bool two_directions =
      config.degeneracy_ratio * result.eigenvalues(0) > result.eigenvalues(1);
  result.num_target_directions = two_directions ? 2 : 1;

  // Contribution of every voxel: projection of the target directions onto the informative
  // directions of its intensity map. The eigenvector signs are arbitrary and the intensity
  // information is unsigned, so the directions are projected by magnitude.
  struct VoxelContribution {
    double contribution;
    size_t observed_idx;
  };
  std::vector<VoxelContribution> contributions;
  contributions.reserve(observed.size());
  for (size_t i = 0; i < observed.size(); ++i) {
    const Voxel& voxel = *observed[i].voxel;
    if (voxel.intensity_weights_.size() == 0) continue;
    double contribution = 0.0;
    for (int k = 0; k < result.num_target_directions; ++k) {
      const V3 direction_C = voxel.T_C_W_.linear() * result.eigenvectors.col(k);
      contribution += direction_C.head<2>().cwiseAbs().dot(voxel.intensity_info_);
    }
    if (contribution > 0.0) {
      contributions.push_back({contribution, i});
    }
  }

  const size_t n_select = std::min(config.num_voxels, contributions.size());
  std::partial_sort(contributions.begin(), contributions.begin() + n_select, contributions.end(),
                    [](const VoxelContribution& a, const VoxelContribution& b) {
                      return std::tie(b.contribution, a.observed_idx) <
                             std::tie(a.contribution, b.observed_idx);
                    });

  // Keep the points with a valid intensity inside the selected intensity voxels.
  result.intensity_voxels.reserve(n_select);
  for (size_t i = 0; i < n_select; ++i) {
    const ObservedVoxel& entry = observed[contributions[i].observed_idx];
    result.intensity_voxels.push_back(voxel_entries[entry.begin].hash);
    for (size_t j = entry.begin; j < entry.end; ++j) {
      const size_t idx = voxel_entries[j].idx;
      if (intensities[idx] >= 0.0) {
        selected.push_back(idx);
      }
    }
  }
}

}  // namespace bievr
