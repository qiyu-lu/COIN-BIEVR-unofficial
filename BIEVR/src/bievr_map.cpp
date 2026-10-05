#include "bievr_lio/bievr_map.h"

#include <Eigen/Eigenvalues>
#include <algorithm>
#include <cmath>
#include <utility>
#include <vector>

#include "bievr_lio/log++.h"
#include "bievr_lio/utils.h"

namespace bievr {

BIEVRMap::BIEVRMap(Config config) : config_(config) {
  // Precompute the offsets to the 8 corners of a voxel for quick access when projecting the voxel
  // into the image
  corner_offsets_ = Eigen::MatrixXd(3, 8);
  int count = 0;
  for (int dx : {0, 1}) {
    for (int dy : {0, 1}) {
      for (int dz : {0, 1}) {
        Eigen::Vector3d corner(dx, dy, dz);
        corner_offsets_.col(count) = corner;
        count++;
      }
    }
  }
  corner_offsets_ *= config_.voxel_size;

  // Store offsets for quick access when searching for neighboring voxels.
  neighbor_offsets_ = getNeighborOffsets(config_.voxel_size);

  // Precompute Gaussian kernel for smoothing
  double radius = 1.0;
  float sigma = computeSigmaFromRadius(radius);
  gauss_kernel_ = buildGaussianKernel(radius, sigma);

  // For quick access
  norm_tol_rad_ = M_PI * config_.norm_tol_deg / 180.0;
  inv_voxel_size_ = 1.0 / config_.voxel_size;
  inv_px_size_ = 1.0 / config_.px_size;
}

bool BIEVRMap::integratePoints(const Pointcloud& cloud, const std::vector<double>* ranges,
                               const IntensityView* intensities) {
  if (cloud.empty()) {
    LOG(I, "No points in cloud to map.");
    return false;
  }

  LOG(D, "Integrating " << cloud.size() << " points to the map.");

  // Each entry is {voxel hash, point (xyz, range/weight, intensity)}.
  std::vector<std::pair<size_t, MapPoint>> hashed_points(cloud.size());

  // Calculate Hash indices for each point
  tbb::parallel_for(tbb::blocked_range<size_t>(0, cloud.size()),
                    [&](const tbb::blocked_range<size_t>& r) {
                      for (size_t i = r.begin(); i != r.end(); ++i) {
                        hashed_points[i].second.head(3) = cloud[i];
                        if (ranges) {
                          hashed_points[i].second(3) = (*ranges)[i];  // weight by range
                        } else {
                          hashed_points[i].second(3) = 1;
                        }
                        hashed_points[i].second(4) =
                            intensities ? (*intensities)(i) : kInvalidIntensity;
                        hashed_points[i].first = hashIndex(hashed_points[i].second.head(3));
                      }
                    });

  // Group points by voxel; tie-break on x for a deterministic order within each voxel.
  tbb::parallel_sort(hashed_points.begin(), hashed_points.end(), [](const auto& a, const auto& b) {
    if (a.first != b.first) return a.first < b.first;
    return a.second(0) < b.second(0);
  });

  // Extract unique hash indices
  std::vector<size_t> hash_change_indices;
  hash_change_indices.reserve(hashed_points.size());
  if (!hashed_points.empty()) {
    size_t prev_hash = hashed_points[hashed_points.size() - 1].first;

    for (size_t i = 0; i < hashed_points.size(); ++i) {
      size_t current_hash = hashed_points[i].first;
      if (current_hash != prev_hash) {
        hash_change_indices.push_back(i);
        if (map_.find(current_hash) == map_.end()) {
          voxels_cache_.push_front(current_hash);
          auto res = map_.emplace(current_hash, VoxelEntry{Voxel(), voxels_cache_.begin()});
          res.first->second.voxel.pending_points_.reserve(8);
        }
        prev_hash = current_hash;
      }
    }
  }

  // Update voxels. Resolve each group's voxel iterator once here (in parallel) and reuse it for
  // the serial LRU update below, instead of looking the voxel up in the map again.
  std::vector<decltype(map_)::iterator> voxel_iters(hash_change_indices.size());
  tbb::parallel_for(
      tbb::blocked_range<size_t>(0, hash_change_indices.size()),
      [&](const tbb::blocked_range<size_t>& r) {
        std::vector<MapPoint> voxel_points;
        for (size_t i = r.begin(); i != r.end(); ++i) {
          int start_idx = hash_change_indices[i];
          int end_idx = (i + 1 < hash_change_indices.size()) ? hash_change_indices[i + 1]
                                                             : hashed_points.size();

          const size_t hash = hashed_points[start_idx].first;
          auto iter = map_.find(hash);
          voxel_iters[i] = iter;
          voxel_points.clear();
          voxel_points.reserve(end_idx - start_idx);
          for (size_t j = start_idx; j < end_idx; ++j) {
            iter->second.voxel.sum_ += hashed_points[j].second.head(3);
            iter->second.voxel.num_points_++;
            iter->second.voxel.outer_sum_ +=
                hashed_points[j].second.head(3) * hashed_points[j].second.head(3).transpose();
            voxel_points.emplace_back(hashed_points[j].second);
          }

          bool valid_before = iter->second.voxel.observed_;
          bool normal_change = updateNormal(iter->second.voxel);

          auto& pending = iter->second.voxel.pending_points_;
          if (!valid_before && iter->second.voxel.observed_) {
            // Just became observed: fold the points accumulated while unobserved into this scan's
            // batch so they get projected once, then release the buffer.
            voxel_points.insert(voxel_points.end(), pending.begin(), pending.end());
            pending.clear();
            pending.shrink_to_fit();
          } else if (!iter->second.voxel.observed_) {
            // Still unobserved: keep accumulating raw points until a normal exists.
            pending.insert(pending.end(), voxel_points.begin(), voxel_points.end());
          }

          updateBumpImage(voxel_points, iter->second.voxel, normal_change);
        }
      });

  // Update LRU Cache
  for (const auto& iter : voxel_iters) {
    voxels_cache_.splice(voxels_cache_.begin(), voxels_cache_, iter->second.lru_it);
  }

  // If max_size exceeded, remove least recently used voxels
  int removal_counter = 0;
  while (map_.size() > config_.max_size && !voxels_cache_.empty()) {
    map_.erase(voxels_cache_.back());
    voxels_cache_.pop_back();
    removal_counter++;
  }

  if (removal_counter > 0) {
    LOG(I, "BIEVRMap exceeded max_size. Removed " << removal_counter << " old voxels.");
  }
  return true;
}

bool BIEVRMap::updateNormal(Voxel& voxel) {
  if (voxel.num_points_ < kNMinValid) return false;

  Point mean = voxel.sum_ / static_cast<double>(voxel.num_points_);
  M3 covariance = (voxel.outer_sum_ - voxel.sum_ * mean.transpose()) /
                  (static_cast<double>(voxel.num_points_) - 1.0);

  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(covariance);

  // Smallest eigenvalue -> normal
  Eigen::Vector3d normal = solver.eigenvectors().col(0);  //

  bool update_normal = false;
  if (voxel.observed_) {
    // The previous normal is the Z axis of the old frame
    const Eigen::Vector3d& prev_normal = voxel.T_O_W_.linear().row(2);
    double angle = std::acos(std::abs(prev_normal.dot(normal)));
    if (angle > norm_tol_rad_) {
      update_normal = true;
    }
  } else {
    update_normal = true;
  }
  if (update_normal) {
    // Get the normal vector (Z axis of the new frame)
    Eigen::Vector3d z_axis = normal;
    Eigen::Vector3d x_axis = z_axis.unitOrthogonal();
    Eigen::Vector3d y_axis = z_axis.cross(x_axis).normalized();

    // Rotation matrix to align with plane
    Eigen::Matrix3d R_align;
    R_align.col(0) = x_axis;
    R_align.col(1) = y_axis;
    R_align.col(2) = z_axis;
    Transform T_W_O;
    T_W_O.linear() = R_align;
    T_W_O.translation() = mean;
    voxel.T_O_W_ = T_W_O.inverse();
  }

  voxel.observed_ = true;

  return update_normal;
}

bool BIEVRMap::updateBumpImage(const std::vector<MapPoint>& points, Voxel& voxel,
                               bool normal_change) {
  if (!voxel.observed_) {
    return false;
  }

  Eigen::MatrixXi changed;
  if (normal_change) {
    // If the normal has changed, we reproject the surface from the old image into the new image
    ImageBounds bounds = computeImageSize(voxel, points[0].head<3>());
    reprojectImage(voxel, bounds, changed);
  } else {
    changed = Eigen::MatrixXi::Zero(voxel.bump_img_.rows(), voxel.bump_img_.cols());
  }

  // Update the pixel values and weights based on the new points
  integratePoints(points, voxel, changed);

  Eigen::MatrixXi changed_dilated =
      Eigen::MatrixXi::Zero(voxel.bump_img_.rows(), voxel.bump_img_.cols());
  if (normal_change) {
    changed_dilated = changed;
  } else {
    // For the smoothing, we also need to consider pixels that are adjacent to the changed pixels
    dilateMask(changed, voxel.bump_weights_, changed_dilated);
  }

  if (config_.smooth) {
    maskedGaussianSmooth(voxel.bump_img_, voxel.bump_weights_, changed_dilated,
                         voxel.bump_smoothed_);
  } else {
    voxel.bump_smoothed_ = voxel.bump_img_;
  }

  computeScore(voxel);

  if (config_.intensity) {
    if (config_.smooth_intensity) {
      maskedGaussianSmooth(voxel.intensity_img_, voxel.intensity_weights_, changed_dilated,
                           voxel.intensity_smoothed_);
    } else {
      voxel.intensity_smoothed_ = voxel.intensity_img_;
    }
    computeIntensityInfo(voxel);
  }

  return true;
}

BIEVRMap::ImageBounds BIEVRMap::computeImageSize(const Voxel& voxel,
                                                 const Eigen::Vector3d& reference_point) const {
  Eigen::Vector3d p_origin = getVoxelOrigin(reference_point);
  Eigen::MatrixXd voxel_corners = corner_offsets_.colwise() + p_origin;
  // Project voxel corners on voxel plane
  Eigen::MatrixXd uv_convers = voxel.T_O_W_.matrix().block<2, 3>(0, 0) * voxel_corners +
                               voxel.T_O_W_.matrix().block<2, 1>(0, 3).replicate(1, 8);
  // Calculate minimum required image size to cover all corners
  ImageBounds bounds;
  bounds.u_min = uv_convers.row(0).minCoeff();
  double u_max = uv_convers.row(0).maxCoeff();
  bounds.v_min = uv_convers.row(1).minCoeff();
  double v_max = uv_convers.row(1).maxCoeff();
  bounds.width = static_cast<int>(std::ceil((u_max - bounds.u_min) * inv_px_size_) + 1);
  bounds.height = static_cast<int>(std::ceil((v_max - bounds.v_min) * inv_px_size_) + 1);
  return bounds;
}

void BIEVRMap::reprojectImage(Voxel& voxel, const ImageBounds& bounds, Eigen::MatrixXi& changed) {
  Eigen::MatrixXf bump_original = voxel.bump_img_;
  Eigen::MatrixXf weights_original = voxel.bump_weights_;
  Transform T_W_C_o = voxel.T_C_W_.inverse();
  voxel.bump_img_.resize(bounds.height, bounds.width);
  voxel.bump_smoothed_.resize(bounds.height, bounds.width);
  voxel.bump_weights_.resize(bounds.height, bounds.width);
  changed.resize(bounds.height, bounds.width);
  voxel.bump_img_.setZero();
  voxel.bump_smoothed_.setZero();
  voxel.bump_weights_.setZero();
  changed.setZero();
  // The intensity map lives in the same image plane and is reprojected along with the bump image.
  Eigen::MatrixXf intensity_original;
  Eigen::MatrixXf intensity_weights_original;
  if (config_.intensity) {
    intensity_original = voxel.intensity_img_;
    intensity_weights_original = voxel.intensity_weights_;
    voxel.intensity_img_.setZero(bounds.height, bounds.width);
    voxel.intensity_smoothed_.setZero(bounds.height, bounds.width);
    voxel.intensity_weights_.setZero(bounds.height, bounds.width);
  }
  Point p_o_planar(bounds.u_min, bounds.v_min, 0.0);
  Point p_w_o = voxel.T_O_W_.inverse() * p_o_planar;
  Transform T_W_C;
  T_W_C.matrix().topLeftCorner<3, 3>() = voxel.T_O_W_.linear().transpose();
  T_W_C.matrix().topRightCorner<3, 1>() = p_w_o;
  voxel.T_C_W_ = T_W_C.inverse();

  Transform T_C1_C0 = voxel.T_C_W_ * T_W_C_o;
  for (int i = 0; i < bump_original.rows(); ++i) {
    for (int j = 0; j < bump_original.cols(); ++j) {
      if (weights_original(i, j) == 0) continue;
      // Lift the point to 3D using the original bump value, then transform it into the new camera
      // frame and project it
      Point p_O = T_C1_C0 * Point(j * config_.px_size, i * config_.px_size, bump_original(i, j));
      int x = static_cast<int>(std::round(p_O(0) * inv_px_size_));
      int y = static_cast<int>(std::round(p_O(1) * inv_px_size_));

      if (x < 0 || x >= voxel.bump_img_.cols() || y < 0 || y >= voxel.bump_img_.rows()) {
        continue;
      }

      voxel.bump_img_(y, x) = p_O(2);
      voxel.bump_weights_(y, x) = weights_original(i, j);
      changed(y, x) = 1;
      if (config_.intensity && intensity_weights_original(i, j) > 0) {
        voxel.intensity_img_(y, x) = intensity_original(i, j);
        voxel.intensity_weights_(y, x) = intensity_weights_original(i, j);
      }
    }
  }
}

void BIEVRMap::integratePoints(const std::vector<MapPoint>& points, Voxel& voxel,
                               Eigen::MatrixXi& changed) {
  for (const auto& p : points) {
    Point p_O = voxel.T_C_W_.linear() * p.head(3) + voxel.T_C_W_.translation();

    int x = static_cast<int>(std::round(p_O(0) * inv_px_size_));
    int y = static_cast<int>(std::round(p_O(1) * inv_px_size_));

    double mean_old = voxel.bump_img_(y, x);
    double weight = voxel.bump_weights_(y, x);
    // Limit weighting so points close to the sensor don't get too powerful
    double weight_new = config_.weighted ? std::min(0.5, 1. / p(3)) : 1.;
    voxel.bump_weights_(y, x) += weight_new;
    voxel.bump_img_(y, x) = (mean_old * weight + weight_new * p_O(2)) / voxel.bump_weights_(y, x);
    changed(y, x) = 1;

    // The intensity map uses the same weighting as the bump image.
    if (config_.intensity && p(4) >= 0.0) {
      const double intensity_old = voxel.intensity_img_(y, x);
      const double intensity_weight = voxel.intensity_weights_(y, x);
      voxel.intensity_weights_(y, x) += weight_new;
      voxel.intensity_img_(y, x) = (intensity_old * intensity_weight + weight_new * p(4)) /
                                   voxel.intensity_weights_(y, x);
    }
  }
}

void BIEVRMap::dilateMask(const Eigen::MatrixXi& changed, const Eigen::MatrixXf& weights,
                          Eigen::MatrixXi& changed_dilated) {
  for (size_t i = 0; i < changed.rows(); ++i) {
    for (size_t j = 0; j < changed.cols(); ++j) {
      if (changed(i, j) != 1) continue;
      for (int k = -1; k < 2; k++) {
        int i_n = i + k;
        if (i_n < 0 || i_n >= changed.rows()) continue;
        for (int l = -1; l < 2; l++) {
          int j_n = j + l;
          if (j_n < 0 || j_n >= changed.cols()) continue;
          if (weights(i_n, j_n) <= 0.) continue;
          changed_dilated(i_n, j_n) = 1;
        }
      }
    }
  }
}

// Apply Gaussian smoothing only over valid pixels
void BIEVRMap::maskedGaussianSmooth(const Eigen::MatrixXf& image, const Eigen::MatrixXf& weights,
                                    const Eigen::MatrixXi& changed, Eigen::MatrixXf& image_smooth) {
  int rows = image.rows();
  int cols = image.cols();
  int radius = gauss_kernel_.rows() / 2;
  for (int y = 0; y < rows; ++y) {
    for (int x = 0; x < cols; ++x) {
      // Skip pixels not marked
      if (!changed(y, x)) {
        continue;
      }
      float weightedSum = 0.0f;
      float weightSum = 0.0f;

      for (int ky = -radius; ky <= radius; ++ky) {
        for (int kx = -radius; kx <= radius; ++kx) {
          int yy = y + ky;
          int xx = x + kx;

          if (xx < 0 || yy < 0 || xx >= cols || yy >= rows) continue;

          if (weights(yy, xx) > 0.) {
            float w = gauss_kernel_(ky + radius, kx + radius);
            weightedSum += image(yy, xx) * w;
            weightSum += w;
          }
        }
      }

      if (weightSum > 0.0f) {
        image_smooth(y, x) = weightedSum / weightSum;
      } else {
        image_smooth(y, x) = 0.0f;
      }
    }
  }
}

void BIEVRMap::computeScore(Voxel& voxel) {
  int total_count = 0;
  double sum_img_dist = 0.0;

  for (int i = 0; i < voxel.bump_img_.rows(); ++i) {
    for (int j = 0; j < voxel.bump_img_.cols(); ++j) {
      if (voxel.bump_weights_(i, j) > 0) {
        total_count++;
        double val = static_cast<double>(voxel.bump_smoothed_(i, j));
        sum_img_dist += std::abs(val);
      }
    }
  }

  voxel.mean_img_dist_ = sum_img_dist / total_count;

  // Discourage voxels with few observed pixels as they have lower probability of giving successful
  // correspondences
  if (total_count < 5) {
    voxel.mean_img_dist_ = 0;
  }
}

void BIEVRMap::computeIntensityInfo(Voxel& voxel) {
  const Eigen::MatrixXf& image = voxel.intensity_smoothed_;
  const Eigen::MatrixXf& weights = voxel.intensity_weights_;
  const int rows = image.rows();
  const int cols = image.cols();

  // Sum of the central-difference gradient magnitudes over all pixels whose neighbors exist.
  double sum_u = 0.0;
  double sum_v = 0.0;
  for (int j = 0; j < cols; ++j) {
    for (int i = 0; i < rows; ++i) {
      if (j > 0 && j + 1 < cols && weights(i, j - 1) > 0 && weights(i, j + 1) > 0) {
        sum_u += std::abs(image(i, j + 1) - image(i, j - 1));
      }
      if (i > 0 && i + 1 < rows && weights(i - 1, j) > 0 && weights(i + 1, j) > 0) {
        sum_v += std::abs(image(i + 1, j) - image(i - 1, j));
      }
    }
  }
  voxel.intensity_info_ << 0.5 * sum_u, 0.5 * sum_v;
}

const Voxel* BIEVRMap::getVoxel(size_t hash_idx) const {
  auto it = map_.find(hash_idx);
  if (it != map_.end() && it->second.voxel.observed_) {
    return &it->second.voxel;
  }
  return nullptr;
}

Eigen::Vector3i BIEVRMap::getVoxelIdx(const Eigen::Vector3d& point) const {
  Eigen::Vector3i idx;
  idx(0) = floor(point.x() * inv_voxel_size_);
  idx(1) = floor(point.y() * inv_voxel_size_);
  idx(2) = floor(point.z() * inv_voxel_size_);
  return idx;
}

Eigen::Vector3d BIEVRMap::getVoxelOrigin(const Eigen::Vector3d& point) const {
  int hx = static_cast<int>(std::floor(point.x() * inv_voxel_size_));
  int hy = static_cast<int>(std::floor(point.y() * inv_voxel_size_));
  int hz = static_cast<int>(std::floor(point.z() * inv_voxel_size_));
  return Eigen::Vector3d(hx * config_.voxel_size, hy * config_.voxel_size, hz * config_.voxel_size);
}

bool BIEVRMap::nearestVoxel(const Eigen::Vector3d& point, size_t& result) const {
  double min_dist = std::numeric_limits<double>::max();
  bool found = false;
  // Select nearest voxel based on distance to voxel centroid
  for (const auto& offset : neighbor_offsets_) {
    Eigen::Vector3d nearby_point = point + offset;
    size_t hash_idx = hashIndex(nearby_point);
    auto voxel = getVoxel(hash_idx);
    if (!voxel) continue;
    Eigen::Vector3d centroid = voxel->sum_ / static_cast<double>(voxel->num_points_);
    double sqdistance = (point - centroid).squaredNorm();
    if (sqdistance > min_dist) continue;
    min_dist = sqdistance;
    result = hash_idx;
    found = true;
  }
  return found;
}

}  // namespace bievr
