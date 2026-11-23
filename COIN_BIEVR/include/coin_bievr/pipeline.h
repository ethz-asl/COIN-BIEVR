#ifndef COIN_BIEVR_PIPELINE_H_
#define COIN_BIEVR_PIPELINE_H_

#include <typeindex>

#include "coin_bievr/base_intensity_projector.h"
#include "coin_bievr/coin_bievr_map.h"
#include "coin_bievr/imu_integrator.h"
#include "coin_bievr/intensity_mesh.h"
#include "coin_bievr/log++.h"
#include "coin_bievr/ls_optimizer.h"
#include "coin_bievr/preprocess.h"
#include "coin_bievr/utils.h"

namespace coin_bievr {

class Pipeline {
 public:
  struct Config {
    Config() = default;
    PreprocessConfig preprocess;
    ImuConfig imu;
    RegistrationConfig registration;
    COINBIEVRMap::Config map;
    ProjectorConfig projector;
    bool print_timing = false;
    bool publish_all_clouds = false;
    bool publish_images = false;   // publish the projected intensity images
    // Publish the geometrically degenerate directions estimated for the photometric
    // sampling. Only computed when the intensity residual is enabled.
    bool publish_degeneracy = false;
    // Publish the voxel intensity images around the robot as a mesh. Needs the
    // intensity residual (otherwise the map has no intensity images) and an
    // interface that can convert IntensityMesh (nvblox_msgs in ROS).
    bool publish_intensity_map = false;
    double intensity_map_period_s = 1.0;  // minimum time between two updates
    IntensityMeshBuilder::Config intensity_map;
    bool print_debug = false;      // when true, lower the log level to show DEBUG messages
    bool print_dashboard = false;  // when true, print the fancy live status dashboard
    // Path to the ASCII art shown at the top of the dashboard (e.g. coin_bievr_ascii.txt).
    std::string dashboard_ascii_path = "";
    // Lines per animation frame in the ASCII art file. 0 = show the file as one static image.
    int dashboard_ascii_frame_lines = 17;
    Transform T_I_L = Transform::Identity();  // Transform from LiDAR to IMU frame
    std::string map_frame = "map";
    std::string body_frame = "body";
    std::string log_path = "";

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
  // Passing both intensity arguments turns on the photometric selection, which
  // adds points to `filtered` beyond what the geometric samplers picked and fills
  // `filtered_intensity` with -1 for every point that is not one of them.
  void sampleSource(const Pointcloud& undistorted, const Transform& T_W_I_init,
                    Pointcloud& filtered, Pointcloud& coarse, Pointcloud& fine,
                    const IntensityView* intensities = nullptr,
                    Intensities* filtered_intensity = nullptr,
                    DegeneracyInfo* degeneracy = nullptr) const;

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
                    const Pointcloud& source_filtered, const Pointcloud& source_coarse,
                    const Pointcloud& source_fine, const Pointcloud& undistorted,
                    const IntensityView& intensities,
                    const Intensities* source_intensity = nullptr);
  void publishLatestState(const Header& header);
  void publishIntensityMap(const Header& header, const Transform& T_W_I);

  template <typename T>
  bool hasPublisher() const {
    return publishers_.find(typeid(T)) != publishers_.end();
  }
  void publishDebugClouds(const Pointcloud& source_filtered, const Pointcloud& source_coarse,
                          const Pointcloud& source_fine, const Pointcloud& undistorted_cloud,
                          const IntensityView& intensities, const Transform& T_W_I,
                          const Header& header,
                          const Intensities* source_intensity = nullptr);

  // Logging
  void logTUM(double timestamp, const Transform& pose);

  Config config_;
  std::map<uint64_t, State> states_;
  std::map<uint64_t, ImuIntegratorPtr> imu_integrators_;

  size_t seq_counter_ = 0;
  Phase phase_ = Phase::NeedBias;
  std::unique_ptr<BiasInitializer> bias_initializer_;
  std::unique_ptr<BaseIntensityProjector> projector_;
  std::shared_ptr<COINBIEVRMap> map_;
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

  // Intensity map visualization (publish_intensity_map).
  std::unique_ptr<IntensityMeshBuilder> intensity_mesh_builder_;
  uint64_t last_intensity_map_stamp_ = 0;
  bool intensity_map_warned_ = false;
};
}  // namespace coin_bievr

#endif  // COIN_BIEVR_PIPELINE_H_
