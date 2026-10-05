#ifndef BIEVR_LIO_INTENSITY_H_
#define BIEVR_LIO_INTENSITY_H_

#include <limits>
#include <vector>

#include "bievr_lio/common.h"

namespace bievr {

struct IntensityConfig {
  bool enabled = false;

  // --- Intensity processing (sensor specific) ---
  // Layout of the intermediate intensity image the brightness normalization runs on. Points are
  // mapped to it with a spherical projection: the width spans 360 deg of azimuth, the height the
  // elevations between fov_down and fov_up.
  int image_width = 1024;
  int image_height = 128;
  double fov_up_deg = 45.0;
  double fov_down_deg = -45.0;
  // Ouster point-to-pixel lookup: column shift of every row of the organized cloud. If set and
  // the cloud holds image_width * image_height points, the image is built from the point index
  // (destaggered) instead of the spherical projection.
  std::vector<int> pixel_shift_by_row;
  // Factor applied to the raw intensity before filtering.
  double scale = 1.0;
  // Window [px] of the brightness map used for the normalization.
  int window_width = 41;
  int window_height = 7;
  // Constant s that scales the normalized intensity to [0, 255].
  double brightness_scale = 140.0;
  // Line artifact removal for Ouster sensors (COIN-LIO): the signal passing a vertical high-pass
  // and a horizontal low-pass FIR filter is removed from the image. Disabled if a kernel is empty.
  std::vector<double> line_highpass;
  std::vector<double> line_lowpass;
  // Points outside of this range [m] do not get an intensity.
  double min_range = 0.0;
  double max_range = std::numeric_limits<double>::max();

  // --- Map-informed intensity point sampling ---
  // Number of voxels with the strongest contribution that are used as intensity voxels.
  size_t num_voxels = 100;
  // Two geometric directions count as unconstrained if ratio * lambda_1 > lambda_2.
  double degeneracy_ratio = 10.0;
};

// Filters the raw LiDAR intensities so that they are consistent within and between scans: the
// points are projected into an intensity image, which is normalized by its local brightness
// (the mean over the non-empty pixels of a large window around every pixel).
class IntensityProcessor {
 public:
  explicit IntensityProcessor(const IntensityConfig& config);

  // Computes the filtered intensity in [0, 255] of every point of a cloud given in the LiDAR
  // frame. Points without a usable intensity are set to kInvalidIntensity.
  void process(const StampedIntensityPointcloud& cloud, Intensities& filtered) const;

 private:
  using Image = Eigen::Array<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;

  void removeLines(Image& image) const;
  void computeBrightness(const Image& image, const Image& count, Image& brightness) const;

  IntensityConfig config_;
  std::vector<int> column_shift_;  // pixel_shift_by_row wrapped into [0, image_width)
  bool remove_lines_ = false;
  double fov_up_rad_ = 0.0;
  double inv_fov_rad_ = 0.0;
};

}  // namespace bievr

#endif  // BIEVR_LIO_INTENSITY_H_
