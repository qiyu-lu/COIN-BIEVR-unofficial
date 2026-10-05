#include "bievr_lio/intensity.h"

#include <tbb/blocked_range.h>
#include <tbb/parallel_for.h>

#include <algorithm>
#include <cmath>

#include "bievr_lio/log++.h"

namespace bievr {

namespace {

// Invalid returns are stored as points at the origin.
constexpr double kMinValidRangeSquared = 1e-6;

using RowArray = Eigen::Array<float, 1, Eigen::Dynamic>;
using ImageD = Eigen::Array<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;

// Mirrors an index back into [0, n) without repeating the border element.
inline int reflect(int i, int n) {
  if (n == 1) return 0;
  while (i < 0 || i >= n) {
    i = i < 0 ? -i : 2 * (n - 1) - i;
  }
  return i;
}

inline int wrap(int i, int n) { return ((i % n) + n) % n; }

}  // namespace

IntensityProcessor::IntensityProcessor(const IntensityConfig& config) : config_(config) {
  column_shift_.reserve(config_.pixel_shift_by_row.size());
  for (const int shift : config_.pixel_shift_by_row) {
    column_shift_.push_back(wrap(shift, config_.image_width));
  }
  remove_lines_ = !config_.line_highpass.empty() && !config_.line_lowpass.empty();
  fov_up_rad_ = config_.fov_up_deg * M_PI / 180.0;
  inv_fov_rad_ = 180.0 / (M_PI * (config_.fov_up_deg - config_.fov_down_deg));
}

void IntensityProcessor::process(const StampedIntensityPointcloud& cloud,
                                 Intensities& filtered) const {
  const size_t n = cloud.size();
  const int w = config_.image_width;
  const int h = config_.image_height;
  filtered.setConstant(n, kInvalidIntensity);

  // The lookup only applies to organized clouds that match the image layout.
  const bool lookup =
      static_cast<int>(column_shift_.size()) == h && n == static_cast<size_t>(w) * h;
  if (!column_shift_.empty() && !lookup) {
    LOG_FIRST(W, 1,
              "Pointcloud with " << n << " points does not match the " << w << "x" << h
                                 << " intensity image layout; using the spherical projection.");
  }

  // Pixel of every point (-1 if it has none).
  std::vector<int> pixels(n, -1);
  tbb::parallel_for(tbb::blocked_range<size_t>(0, n), [&](const tbb::blocked_range<size_t>& r) {
    for (size_t i = r.begin(); i != r.end(); ++i) {
      const auto point = cloud[i];
      const double range_squared = point.head<3>().squaredNorm();
      if (!(range_squared > kMinValidRangeSquared)) continue;

      int row, col;
      if (lookup) {
        row = static_cast<int>(i / w);
        col = (static_cast<int>(i % w) + column_shift_[row]) % w;
      } else {
        const double azimuth = std::atan2(point.y(), point.x());
        const double elevation = std::asin(point.z() / std::sqrt(range_squared));
        col = wrap(static_cast<int>(std::floor(-w / (2.0 * M_PI) * azimuth + 0.5 * w)), w);
        row = static_cast<int>(std::floor((fov_up_rad_ - elevation) * inv_fov_rad_ * h));
        if (row < 0 || row >= h) continue;
      }
      pixels[i] = row * w + col;
    }
  });

  // Intensity image: mean intensity of the points of every pixel.
  const IntensityView intensities = cloud.intensities();
  const float scale = static_cast<float>(config_.scale);
  Image image = Image::Zero(h, w);
  Image count = Image::Zero(h, w);
  for (size_t i = 0; i < n; ++i) {
    if (pixels[i] < 0) continue;
    image.data()[pixels[i]] += scale * static_cast<float>(intensities(i));
    count.data()[pixels[i]] += 1.f;
  }
  image = (count > 0.f).select(image / count, 0.f);

  if (remove_lines_) {
    removeLines(image);
  }

  Image brightness;
  computeBrightness(image, count, brightness);

  // Irregular scan patterns can put several points into one pixel. They keep their own intensity
  // unless the image itself was built from the lookup or filtered.
  const bool use_pixel_value = lookup || remove_lines_;
  const double min_range_squared = config_.min_range * config_.min_range;
  const double max_range_squared = config_.max_range * config_.max_range;
  const double s = config_.brightness_scale;
  tbb::parallel_for(tbb::blocked_range<size_t>(0, n), [&](const tbb::blocked_range<size_t>& r) {
    for (size_t i = r.begin(); i != r.end(); ++i) {
      if (pixels[i] < 0) continue;
      const double range_squared = cloud[i].head<3>().squaredNorm();
      if (range_squared < min_range_squared || range_squared > max_range_squared) continue;
      const double value = use_pixel_value ? static_cast<double>(image.data()[pixels[i]])
                                           : config_.scale * intensities(i);
      filtered(i) = std::min(255.0, s * value / (brightness.data()[pixels[i]] + 1.0));
    }
  });
}

void IntensityProcessor::removeLines(Image& image) const {
  const int h = image.rows();
  const int w = image.cols();
  const std::vector<double>& highpass = config_.line_highpass;
  const std::vector<double>& lowpass = config_.line_lowpass;
  const int n_high = static_cast<int>(highpass.size());
  const int n_low = static_cast<int>(lowpass.size());

  // Vertical high-pass.
  Image high(h, w);
  tbb::parallel_for(tbb::blocked_range<int>(0, h), [&](const tbb::blocked_range<int>& r) {
    for (int row = r.begin(); row != r.end(); ++row) {
      RowArray out = RowArray::Zero(w);
      for (int k = 0; k < n_high; ++k) {
        out += static_cast<float>(highpass[k]) * image.row(reflect(row + k - n_high / 2, h));
      }
      high.row(row) = out;
    }
  });

  // Horizontal low-pass (the image wraps around in azimuth). What passes both filters are the
  // line artifacts, which are removed from the image.
  tbb::parallel_for(tbb::blocked_range<int>(0, h), [&](const tbb::blocked_range<int>& r) {
    RowArray padded(w + n_low - 1);
    for (int row = r.begin(); row != r.end(); ++row) {
      for (int c = 0; c < padded.size(); ++c) {
        padded(c) = high(row, wrap(c - n_low / 2, w));
      }
      RowArray low = RowArray::Zero(w);
      for (int k = 0; k < n_low; ++k) {
        low += static_cast<float>(lowpass[k]) * padded.segment(k, w);
      }
      image.row(row) = (image.row(row) - low).max(0.f);
    }
  });
}

void IntensityProcessor::computeBrightness(const Image& image, const Image& count,
                                           Image& brightness) const {
  const int h = image.rows();
  const int w = image.cols();
  const int half_w = std::min(config_.window_width / 2, (w - 1) / 2);
  const int half_h = config_.window_height / 2;

  // Horizontal box sums over the non-empty pixels (the image wraps around in azimuth).
  ImageD sum_h(h, w);
  ImageD count_h(h, w);
  tbb::parallel_for(tbb::blocked_range<int>(0, h), [&](const tbb::blocked_range<int>& r) {
    std::vector<double> prefix_sum(w + 2 * half_w + 1, 0.0);
    std::vector<double> prefix_count(w + 2 * half_w + 1, 0.0);
    for (int row = r.begin(); row != r.end(); ++row) {
      for (int c = 0; c < w + 2 * half_w; ++c) {
        const int col = wrap(c - half_w, w);
        const bool valid = count(row, col) > 0.f;
        prefix_sum[c + 1] = prefix_sum[c] + (valid ? image(row, col) : 0.f);
        prefix_count[c + 1] = prefix_count[c] + (valid ? 1.0 : 0.0);
      }
      for (int col = 0; col < w; ++col) {
        sum_h(row, col) = prefix_sum[col + 2 * half_w + 1] - prefix_sum[col];
        count_h(row, col) = prefix_count[col + 2 * half_w + 1] - prefix_count[col];
      }
    }
  });

  // Vertical box sums (truncated at the upper and lower image border).
  ImageD prefix_sum = ImageD::Zero(h + 1, w);
  ImageD prefix_count = ImageD::Zero(h + 1, w);
  for (int row = 0; row < h; ++row) {
    prefix_sum.row(row + 1) = prefix_sum.row(row) + sum_h.row(row);
    prefix_count.row(row + 1) = prefix_count.row(row) + count_h.row(row);
  }
  brightness.resize(h, w);
  tbb::parallel_for(tbb::blocked_range<int>(0, h), [&](const tbb::blocked_range<int>& r) {
    for (int row = r.begin(); row != r.end(); ++row) {
      const int lo = std::max(0, row - half_h);
      const int hi = std::min(h, row + half_h + 1);
      for (int col = 0; col < w; ++col) {
        const double n_valid = prefix_count(hi, col) - prefix_count(lo, col);
        brightness(row, col) = n_valid > 0.0 ? static_cast<float>(
                                                   (prefix_sum(hi, col) - prefix_sum(lo, col)) /
                                                   n_valid)
                                             : 0.f;
      }
    }
  });
}

}  // namespace bievr
