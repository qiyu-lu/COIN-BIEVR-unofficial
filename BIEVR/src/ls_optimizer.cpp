// This Levenberg-Marquardt Optimizer builds upon the code used in the nanogicp module in DLIO
// https://github.com/vectr-ucla/direct_lidar_inertial_odometry

#include "bievr_lio/ls_optimizer.h"

#include <tbb/enumerable_thread_specific.h>
#include <tbb/parallel_reduce.h>

#include "bievr_lio/log++.h"
#include "bievr_lio/utils.h"

namespace bievr {

LsqRegistration::LsqRegistration(const BIEVRMap& map, const Pointcloud& source,
                                 const RegistrationConfig& config,
                                 const std::vector<double>* intensities)
    : config_(config), map_(map), points_j_(source), intensities_j_(intensities) {}

Transform LsqRegistration::computeTransformation(const Transform& T_W_L_init) {
  Transform x0 = T_W_L_init;

  skew_points_j_.resize(points_j_.size());
  tbb::parallel_for(tbb::blocked_range<size_t>(0, points_j_.size()),
                    [&](const tbb::blocked_range<size_t>& r) {
                      for (size_t i = r.begin(); i != r.end(); ++i) {
                        skew_points_j_[i] = skew(points_j_[i]);
                      }
                    });

  if (config_.lm_debug_print) {
    LOG(I, "***************** optimize *****************");
  }

  for (int i = 0; i < config_.max_iterations && !converged_; i++) {
    Transform delta;
    if (!stepLm(x0, delta)) {
      LOG(W, "lm not converged!!");
      break;
    }
    converged_ = isConverged(delta);
  }

  return x0;
}

bool LsqRegistration::isConverged(const Transform& delta) const {
  Eigen::Matrix3d R = delta.linear() - Eigen::Matrix3d::Identity();
  Eigen::Vector3d t = delta.translation();

  Eigen::Matrix3d r_delta = 1.0 / config_.rotation_epsilon * R.array().abs();
  Eigen::Vector3d t_delta = 1.0 / config_.transformation_epsilon * t.array().abs();

  return std::max(r_delta.maxCoeff(), t_delta.maxCoeff()) < 1;
}

double LsqRegistration::linearize(const Transform& T_W_L, Matrix66* H, Vector6* b) {
  const bool compute_jacobians = (H != nullptr && b != nullptr);

  Accumulator identity_accumulator;
  identity_accumulator.huber_delta = config_.huber_delta;

  Accumulator total = tbb::parallel_deterministic_reduce(
      tbb::blocked_range<size_t>(0, points_j_.size()),
      identity_accumulator,  // identity
      [&](const tbb::blocked_range<size_t>& r, Accumulator local_acc) -> Accumulator {
        for (size_t i = r.begin(); i != r.end(); ++i) {
          Point p_W = T_W_L.linear() * points_j_[i] + T_W_L.translation();
          size_t hash = map_.hashIndex(p_W);
          const Voxel* voxel = map_.getVoxel(hash);
          if (!voxel) {
            if (!map_.nearestVoxel(p_W, hash)) continue;
            voxel = map_.getVoxel(hash);
            if (!voxel) continue;
          }

          const double inv_size = map_.inv_px_size;
          const auto& T_C_W = voxel->T_C_W_;

          Rotation R_o_j = T_C_W.linear() * T_W_L.linear();
          const Point p_o = T_C_W * p_W;

          const double x = p_o.x() * inv_size;
          const double y = p_o.y() * inv_size;

          // Photometric residual of intensity points: difference between the filtered point
          // intensity and the intensity map of the voxel.
          const double intensity_j = intensities_j_ ? (*intensities_j_)[i] : kInvalidIntensity;
          const bool photometric = intensity_j >= 0.0 && voxel->intensity_weights_.size() > 0;

          double I = 0.0;

          if (!compute_jacobians) {
            if (!config_.img_residual || getSubPixelValue(voxel, x, y, I)) {
              const double r = p_o.z() - I;
              local_acc.add(r, nullptr);
            }
            double I_photo = 0.0;
            if (photometric && getSubPixelValue(voxel->intensity_smoothed_,
                                                voxel->intensity_weights_, x, y, I_photo)) {
              local_acc.addPhotometric(config_.photo_scale * (intensity_j - I_photo), nullptr);
            }
            continue;
          }

          Eigen::Matrix<double, 3, 6> SE3_Jac;
          SE3_Jac.block<3, 3>(0, 3) = R_o_j;  // d p_o / d t
          SE3_Jac.block<3, 3>(0, 0).noalias() = -R_o_j * skew_points_j_[i];

          double dIdx = 0.0;
          double dIdy = 0.0;
          if (!config_.img_residual || sampleValueAndGradient(voxel, x, y, I, dIdx, dIdy)) {
            Eigen::Matrix<double, 1, 2> I_Jac = Eigen::Matrix<double, 1, 2>::Zero();
            if (config_.img_jacobian) {
              I_Jac(0, 0) = dIdx;
              I_Jac(0, 1) = dIdy;
              I_Jac *= inv_size;
            }

            Row6 J = SE3_Jac.row(2) - (I_Jac * SE3_Jac.topRows<2>());

            const double r = p_o.z() - I;
            local_acc.add(r, &J);
          }

          double I_photo = 0.0;
          double dIdx_photo = 0.0;
          double dIdy_photo = 0.0;
          if (photometric &&
              sampleValueAndGradient(voxel->intensity_smoothed_, voxel->intensity_weights_, x, y,
                                     I_photo, dIdx_photo, dIdy_photo)) {
            Eigen::Matrix<double, 1, 2> I_Jac;
            I_Jac << dIdx_photo, dIdy_photo;
            I_Jac *= config_.photo_scale * inv_size;

            Row6 J = -(I_Jac * SE3_Jac.topRows<2>());

            const double r = config_.photo_scale * (intensity_j - I_photo);
            local_acc.addPhotometric(r, &J);
          }
        }

        return local_acc;
      },
      [](const Accumulator& a, const Accumulator& b) -> Accumulator {
        Accumulator out = a;
        out.merge(b);
        return out;
      });

  if (compute_jacobians) {
    *H = total.H;
    *b = total.b;
    // Remember how many points contributed correspondences in this (Jacobian)
    // linearization so the pipeline can report the effective point count.
    num_effective_points_ = total.count;
    num_photometric_points_ = total.photo_count;
  }

  return total.error_sum;
}

namespace {

Eigen::Quaterniond so3_exp(const Eigen::Vector3d& omega) {
  double theta_sq = omega.dot(omega);

  double theta;
  double imag_factor;
  double real_factor;
  if (theta_sq < 1e-10) {
    theta = 0;
    double theta_quad = theta_sq * theta_sq;
    imag_factor = 0.5 - 1.0 / 48.0 * theta_sq + 1.0 / 3840.0 * theta_quad;
    real_factor = 1.0 - 1.0 / 8.0 * theta_sq + 1.0 / 384.0 * theta_quad;
  } else {
    theta = std::sqrt(theta_sq);
    double half_theta = 0.5 * theta;
    imag_factor = std::sin(half_theta) / theta;
    real_factor = std::cos(half_theta);
  }

  return Eigen::Quaterniond(real_factor, imag_factor * omega.x(), imag_factor * omega.y(),
                            imag_factor * omega.z());
}

}  // namespace

bool LsqRegistration::stepLm(Transform& x0, Transform& delta) {
  Matrix66 H;
  Vector6 b;
  double y0 = linearize(x0, &H, &b);

  if (lm_lambda_ < 0.0) {
    lm_lambda_ = config_.lm_init_lambda_factor * H.diagonal().array().abs().maxCoeff();
  }

  double nu = 2.0;
  for (int i = 0; i < config_.lm_max_iterations; i++) {
    Eigen::LDLT<Matrix66> solver(H + lm_lambda_ * Matrix66::Identity());
    Vector6 d = solver.solve(-b);
    delta.setIdentity();
    delta.linear() = so3_exp(d.head<3>()).toRotationMatrix();
    delta.translation() = d.tail<3>();

    Transform xi = x0 * delta;
    double yi = linearize(xi);
    double rho = (y0 - yi) / (d.dot(lm_lambda_ * d - b));

    if (rho < 0) {
      if (isConverged(delta)) {
        return true;
      }

      lm_lambda_ *= nu;
      nu *= 2;
      continue;
    }

    x0 = xi;
    lm_lambda_ *= std::max(1.0 / 3.0, 1 - std::pow(2 * rho - 1, 3));
    return true;
  }

  return false;
}

}  // namespace bievr