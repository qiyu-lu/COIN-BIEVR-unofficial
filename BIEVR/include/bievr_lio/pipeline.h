#ifndef BIEVR_LIO_PIPELINE_H_
#define BIEVR_LIO_PIPELINE_H_

#include <typeindex>

#include "bievr_lio/bievr_map.h"
#include "bievr_lio/imu_integrator.h"
#include "bievr_lio/intensity.h"
#include "bievr_lio/log++.h"
#include "bievr_lio/ls_optimizer.h"
#include "bievr_lio/preprocess.h"
#include "bievr_lio/utils.h"

namespace bievr {

class Pipeline {
 public:
  struct Config {
    Config() = default;
    PreprocessConfig preprocess;
    ImuConfig imu;
    RegistrationConfig registration;
    BIEVRMap::Config map;
    IntensityConfig intensity;
    bool print_timing = false;
    bool publish_all_clouds = false;
    bool print_debug = false;      // when true, lower the log level to show DEBUG messages
    bool print_dashboard = false;  // when true, print the fancy live status dashboard
    // Path to the ASCII art shown at the top of the dashboard (e.g. bievr_ascii.txt).
    std::string dashboard_ascii_path = "";
    Transform T_I_L = Transform::Identity();  // Transform from LiDAR to IMU frame
    std::string map_frame = "map";
    std::string body_frame = "body";
    std::string log_path = "";
    // File the map is written to by saveMap() (binary PCD with one point per map pixel).
    std::string map_path = "";

    size_t min_points_for_map_init = 100;
    size_t map_size_running_threshold = 5;
    size_t informed_sample_count = 300;
    size_t min_map_size_for_imu_opt = 100;
    size_t min_imu_integrators_for_opt = 5;
    double gravity_prior_weight = 5.0;
  };

  explicit Pipeline(const Config& config);
  virtual ~Pipeline() = default;

  void processFrame(const std::vector<ImuMeasurement>& imu_data,
                    const StampedIntensityPointcloud& pointcloud);

  // Writes the map to Config::map_path as a binary PCD (x, y, z, intensity): one point per valid
  // pixel, lifted onto the voxel surface, with the value of the intensity map (0 where the pixel
  // has no intensity). Does nothing if no path is configured.
  bool saveMap() const;

  template <typename T>
  void registerPublisher(std::function<void(const T&, const Header&, const std::string& topic,
                                            const std::string& child_frame)>
                             func) {
    auto wrapper = [func](const void* val, const Header& header, const std::string& topic,
                          const std::string& child_frame) {
      func(*static_cast<const T*>(val), header, topic, child_frame);
    };
    publishers_[typeid(T)] = wrapper;
  }

 private:
  enum class Phase { NeedBias, NeedMap, Running };

  // Pipeline helpers
  bool initializeBias(const std::vector<ImuMeasurement>& imu_data, const Pointcloud& pointcloud,
                      const IntensityView& intensities);
  void tryInitMap(uint64_t stamp, const State& x_j_pred, const Transform& T_W_I_init,
                  const Pointcloud& undistorted, const IntensityView& intensities,
                  std::vector<double>& ranges, const Header& header);
  // Points used for the registration. With intensity enabled, `filtered` is the union of the
  // geometric samples and the intensity points; `filtered_intensities` then holds the filtered
  // intensity of the intensity points (kInvalidIntensity for all other points).
  struct SourceSamples {
    Pointcloud filtered;
    Pointcloud coarse;
    Pointcloud fine;
    Pointcloud intensity;
    std::vector<double> filtered_intensities;
    std::vector<double> intensity_values;  // aligned with `intensity`
    IntensitySamplingInfo intensity_info;
  };
  void sampleSource(const Pointcloud& undistorted, const IntensityView& intensities,
                    const Transform& T_W_I_init, SourceSamples& samples) const;

  // State and optimization management
  bool addState(const uint64_t time, const Quaternion& quat, const V3& p, const V3& v);
  bool addImuIntegrator(ImuIntegratorPtr imu_integrator);
  bool optimizeInertialWindow();

  // Publishing
  template <typename T>
  void publish(const T& value, const Header& header, const std::string& topic,
               const std::string& child_frame = "") const {
    auto it = publishers_.find(typeid(T));
    if (it != publishers_.end()) {
      it->second(static_cast<const void*>(&value), header, topic, child_frame);
    } else {
      LOG(E, "No publisher registered in pipeline for type.");
    }
  }

  void publishFrame(const Header& header, const Transform& T_W_I, const Pointcloud& full_registered,
                    const SourceSamples& samples, const Pointcloud& undistorted,
                    const IntensityView& intensities);
  void publishLatestState(const Header& header);
  void publishDebugClouds(const SourceSamples& samples, const Pointcloud& undistorted_cloud,
                          const IntensityView& intensities, const Transform& T_W_I,
                          const Header& header);

  // Intensity maps of the given voxels as a cloud (one point per valid pixel), for visualization.
  IntensityPointcloud intensityVoxelCloud(const std::vector<size_t>& voxel_hashes) const;

  // Logging
  void logTUM(double timestamp, const Transform& pose);

  Config config_;
  std::map<uint64_t, State> states_;
  std::map<uint64_t, ImuIntegratorPtr> imu_integrators_;

  size_t seq_counter_ = 0;
  Phase phase_ = Phase::NeedBias;
  std::unique_ptr<BiasInitializer> bias_initializer_;
  std::unique_ptr<IntensityProcessor> intensity_processor_;
  std::shared_ptr<BIEVRMap> map_;
  V3 acc_bias_ = V3::Zero();
  V3 gyro_bias_ = V3::Zero();
  V3 gravity_dir_ = V3(0, 0, 1);
  // Latest gyro reading, used to report the angular velocity in the odometry twist.
  V3 latest_gyro_ = V3::Zero();
  // Accelerometer scale resolved during bias estimation (1 if raw, g if the IMU
  // reports gravity-normalized accelerations). Applied to all incoming IMU data.
  double imu_acc_scale_ = 1.0;

  using PublishFunction =
      std::function<void(const void*, const Header&, const std::string&, const std::string&)>;
  std::unordered_map<std::type_index, PublishFunction> publishers_;
  std::shared_ptr<std::ofstream> tum_log_;

  // Accumulated state for the live status dashboard (printDashboard in utils).
  DashboardState dashboard_;
};
}  // namespace bievr

#endif  // BIEVR_LIO_PIPELINE_H_
