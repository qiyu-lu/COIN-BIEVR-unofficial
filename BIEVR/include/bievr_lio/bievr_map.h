#ifndef BIEVR_LIO_BIEVR_MAP_H_
#define BIEVR_LIO_BIEVR_MAP_H_

#include <tbb/parallel_for.h>
#include <tbb/parallel_sort.h>

#include <list>

#include "bievr_lio/common.h"
#include "unordered_dense/unordered_dense.h"

namespace bievr {

// Point as it is integrated into a voxel: xyz, range (row 3) and filtered intensity (row 4).
using MapPoint = Eigen::Matrix<double, 5, 1>;

struct Voxel {
  bool observed_{false};  // We consider a voxel observed if we have seen enough points inside it
  Transform T_C_W_ = Transform::Identity();
  Transform T_O_W_ = Transform::Identity();
  Eigen::MatrixXf bump_img_;
  Eigen::MatrixXf bump_smoothed_;
  Eigen::MatrixXf bump_weights_;
  // Intensity map: pixel-wise weighted mean of the filtered point intensities, stored in the same
  // image plane as the bump image. Pixels are valid where intensity_weights_ > 0. Only allocated
  // if the map stores intensities.
  Eigen::MatrixXf intensity_img_;
  Eigen::MatrixXf intensity_smoothed_;
  Eigen::MatrixXf intensity_weights_;
  // Sum of the gradient magnitudes of the intensity map along the image axes u and v, which
  // quantifies the intensity information along these directions.
  Eigen::Vector2d intensity_info_ = Eigen::Vector2d::Zero();
  M3 outer_sum_ = Eigen::Matrix3d::Zero();
  V3 sum_ = Eigen::Vector3d::Zero();
  size_t num_points_{0};
  double mean_img_dist_{0.0};
  // Raw points accumulated while the voxel is not yet observed (no normal yet, so they cannot be
  // projected into a bump image). Cleared once the voxel becomes observed.
  std::vector<MapPoint> pending_points_;
};

class BIEVRMap {
 public:
  struct Config {
    size_t max_size{10000000};  // maximum number of voxels to store
    double voxel_size{0.5};     // voxel side length
    double px_size{0.05};       // size of bump image pixels
    bool weighted = false;      // use range weighted update for bump image
    bool smooth = false;        // apply gaussian smoothing to bump image
    double norm_tol_deg{3.0};   // if normal changes more than this, reproject bump image
    bool intensity = false;     // store a voxel-wise intensity map next to the bump image
    bool smooth_intensity = true;  // apply gaussian smoothing to the intensity map
  };

  explicit BIEVRMap(Config config);

  // Integrates a cloud given in the map frame. `ranges` weights the pixel updates, `intensities`
  // holds the filtered point intensities for the intensity map (kInvalidIntensity where missing).
  bool integratePoints(const Pointcloud& input_cloud, const std::vector<double>* ranges = nullptr,
                       const IntensityView* intensities = nullptr);

  inline size_t hashIndex(const Point& point) const {
    Eigen::Vector3i voxel_idx = (point * inv_voxel_size_).array().floor().cast<int>();

    return hashIndexVoxel(voxel_idx);
  }
  size_t size() const { return map_.size(); }
  Eigen::Vector3i getVoxelIdx(const Point& point) const;
  const Voxel* getVoxel(const size_t hash_idx) const;
  bool nearestVoxel(const Point& point, size_t& result) const;

  const double& voxel_size = config_.voxel_size;
  const double& pixel_size = config_.px_size;
  const double& inv_px_size = inv_px_size_;

 private:
  struct ImageBounds {
    int width;
    int height;
    double u_min;
    double v_min;
  };

  // Voxel update / bump-image pipeline, in the order integratePoints invokes them.
  bool updateNormal(Voxel& voxel);

  bool updateBumpImage(const std::vector<MapPoint>& points, Voxel& voxel, bool normal_change);

  ImageBounds computeImageSize(const Voxel& voxel, const Point& reference_point) const;

  void reprojectImage(Voxel& voxel, const ImageBounds& bounds, Eigen::MatrixXi& changed);

  void integratePoints(const std::vector<MapPoint>& points, Voxel& voxel,
                       Eigen::MatrixXi& changed);

  void dilateMask(const Eigen::MatrixXi& changed, const Eigen::MatrixXf& weights,
                  Eigen::MatrixXi& changed_dilated);

  void maskedGaussianSmooth(const Eigen::MatrixXf& image, const Eigen::MatrixXf& weights,
                            const Eigen::MatrixXi& changed, Eigen::MatrixXf& image_smooth);

  void computeScore(Voxel& voxel);

  void computeIntensityInfo(Voxel& voxel);

  // Geometry helper.
  Eigen::Vector3d getVoxelOrigin(const Point& point) const;

  Eigen::MatrixXf gauss_kernel_;
  Config config_;
  double inv_voxel_size_{1.0};
  double inv_px_size_{0.05};            // inv size of bump image
  double norm_tol_rad_{3.0};            // if normal changes more than this, update bump image
  static constexpr int kNMinValid = 4;  // minimum points for a valid voxel

  struct VoxelEntry {
    Voxel voxel;
    std::list<size_t>::iterator lru_it;  // iterator into voxels_cache_
  };
  ankerl::unordered_dense::map<size_t, VoxelEntry> map_;
  std::list<size_t> voxels_cache_;

  Eigen::MatrixXd corner_offsets_;
  std::vector<Point> neighbor_offsets_;
};

using VoxelPtr = std::shared_ptr<Voxel>;
using ConstVoxelPtr = std::shared_ptr<const Voxel>;

}  // namespace bievr
#endif  // BIEVR_LIO_BIEVR_MAP_H_