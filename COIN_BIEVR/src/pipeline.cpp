#include "coin_bievr/pipeline.h"

#include <fstream>
#include <sstream>

#include "coin_bievr/inertial_factor.h"
#include "coin_bievr/prior_factor.h"
#include "coin_bievr/timing.h"
#include "coin_bievr/undistort.h"
#include "coin_bievr/utils.h"

namespace coin_bievr {

Pipeline::Pipeline(const Config& config) : config_(config) {
  map_ = std::make_shared<COINBIEVRMap>(config_.map);
  projector_ = createProjector(config_.projector);
  if (config_.publish_intensity_map) {
    intensity_mesh_builder_ = std::make_unique<IntensityMeshBuilder>(config_.intensity_map);
  }

  if (!config_.log_path.empty()) {
    LOG(I, "Logging to " << config_.log_path);
    tum_log_ = std::make_shared<std::ofstream>(config_.log_path, std::ios::trunc);
    if (!tum_log_->is_open()) {
      LOG(E, "Error opening output file.");
    }
  }

  if (config_.print_dashboard && !config_.dashboard_ascii_path.empty()) {
    std::ifstream ascii_file(config_.dashboard_ascii_path);
    if (ascii_file.is_open()) {
      std::vector<std::string> lines;
      std::string line;
      while (std::getline(ascii_file, line)) {
        if (!line.empty() && line.back() == '\r') line.pop_back();  // tolerate CRLF files
        lines.push_back(line);
      }
      // Split into animation frames, indented so the art is centered on the dashboard.
      dashboard_.ascii_frames = splitAsciiFrames(lines, config_.dashboard_ascii_frame_lines);
      if (dashboard_.ascii_frames.size() == 1 && config_.dashboard_ascii_frame_lines > 0 &&
          lines.size() != static_cast<size_t>(config_.dashboard_ascii_frame_lines)) {
        LOG(W, "Dashboard ASCII art has " << lines.size() << " lines, not a multiple of "
                                          << config_.dashboard_ascii_frame_lines
                                          << "; showing it as a static image.");
      }
    } else {
      LOG(W, "Could not open dashboard ASCII art at " << config_.dashboard_ascii_path);
    }
  }

  if (config_.print_dashboard) {
    printDashboardBanner(dashboard_.ascii_frames.empty() ? "" : dashboard_.ascii_frames.front(),
                         "COIN-BIEVR  WAITING FOR DATA");
  }
}

void Pipeline::processFrame(const std::vector<ImuMeasurement>& imu_data,
                            const StampedIntensityPointcloud& points_L) {
  timing::Timer step_timer("step");

  if (imu_data.empty()) {
    LOG(W, "No IMU data to process.");
    return;
  }

  // Cache the latest gyro reading so the odometry twist can report the current
  // angular velocity (already expressed in the body/IMU frame).
  latest_gyro_ = imu_data.back().gyro;

  // Photometric normalization has to run on the raw scan: the Ouster projector
  // maps points to pixels through their scan index, which range filtering
  // destroys. The copy only happens when a projector is configured.
  StampedIntensityPointcloud points_projected_L;
  const StampedIntensityPointcloud* points_in_L = &points_L;
  if (projector_) {
    timing::Timer projector_timer("00_projection");
    points_projected_L = points_L;
    BaseIntensityProjector::DebugImages debug_images;
    projector_->processIntensity(points_projected_L,
                                 config_.publish_images ? &debug_images : nullptr);
    points_in_L = &points_projected_L;
    projector_timer.Stop();

    if (config_.publish_images) {
      // An image has no pose; the frame is only there so it lines up with the
      // rest of the debug output in RViz.
      const Header image_header{points_L.end_stamp, static_cast<uint32_t>(seq_counter_),
                                config_.body_frame};
      publish(debug_images.raw, image_header, "intensity_raw");
      publish(debug_images.processed, image_header, "intensity_processed");
    }
  }

  // Remove points outside of intended range
  timing::Timer filter_timer("01_filter");
  StampedIntensityPointcloud points_filtered_L;
  filterMinMaxRange(*points_in_L, points_filtered_L, config_.preprocess.min_range,
                    config_.preprocess.max_range);
  // Points below the intensity floor are dropped from the scan outright, so they
  // constrain neither the registration nor the map. The rebind keeps the disabled
  // path free of a second full-cloud copy.
  StampedIntensityPointcloud points_bright_L;
  const StampedIntensityPointcloud* points_kept_L = &points_filtered_L;
  if (config_.preprocess.min_intensity >= 0.0) {
    filterMinIntensity(points_filtered_L, points_bright_L, config_.preprocess.min_intensity);
    points_kept_L = &points_bright_L;
  }
  // The per-point time and intensity stay in filtered_L; we only take zero-copy
  // views onto those rows. Undistortion consumes the time view, intensity rides
  // through to publishing. Both views remain valid for the whole frame and stay
  // aligned by index with the spatial clouds derived below, since every transform
  // downstream preserves point order.
  const StampedIntensityPointcloud& filtered_L = *points_kept_L;
  const TimeView times = filtered_L.times();
  const IntensityView intensities = filtered_L.intensities();
  // Transform only the spatial coordinates into the IMU frame (single 3xN write);
  // time and intensity are never rewritten.
  const Pointcloud points_filtered_I = transformPoints(config_.T_I_L, filtered_L);
  filter_timer.Stop();

  if (phase_ == Phase::NeedBias) {
    // Estimate initial biases and orientation based on zero velocity assumption.
    // This also resolves imu_acc_scale_ (normalization detection).
    if (!initializeBias(imu_data, points_filtered_I, intensities)) return;
  }

  // Apply the accelerometer scale resolved during bias estimation if necessary.
  std::vector<ImuMeasurement> imu_scaled;
  const std::vector<ImuMeasurement>* imu_ptr = &imu_data;
  if (imu_acc_scale_ != 1.0) {
    imu_scaled = imu_data;
    for (auto& imu : imu_scaled) imu.acc *= imu_acc_scale_;
    imu_ptr = &imu_scaled;
  }
  const std::vector<ImuMeasurement>& imu = *imu_ptr;

  const State& x_i = states_.rbegin()->second;
  const Header header{filtered_L.end_stamp, static_cast<uint32_t>(x_i.id + 1),
                      config_.map_frame};

  // Propagate state based on IMU data
  timing::Timer preint_timer("02_preint");
  ImuIntegratorPtr imu_integrator =
      std::make_shared<ImuIntegrator>(config_.imu, acc_bias_, gyro_bias_);
  imu_integrator->integrate(imu);
  V3 G = gravity_dir_ * kGMagnitude;
  State x_j_pred;
  imu_integrator->predict(x_i.quat, x_i.p, x_i.v, G, x_j_pred.quat, x_j_pred.p, x_j_pred.v);
  preint_timer.Stop();

  if (points_filtered_I.empty()) {
    LOG(W, "No pointcloud data to process.");
    addState(imu.back().stamp, x_j_pred.quat, x_j_pred.p, x_j_pred.v);
    publishLatestState(header);
    return;
  }

  // Undistort pointcloud based on IMU integration
  timing::Timer undistortion_timer("03_undistortion");
  Pointcloud points_undistorted_I;
  undistortCloud(imu_integrator, x_i, points_filtered_I, times, filtered_L.stamp, G,
                 points_undistorted_I);
  undistortion_timer.Stop();

  std::vector<double> ranges;
  calculateRanges(points_undistorted_I, ranges);
  const Transform T_W_I_init(x_j_pred.quat, x_j_pred.p);
  if (phase_ == Phase::NeedMap) {
    tryInitMap(imu_data.back().stamp, x_j_pred, T_W_I_init, points_undistorted_I, intensities,
               ranges, header);
    return;
  }

  // Select points from the source cloud that will be used for registration
  timing::Timer voxel_timer("04_sampling");
  const bool use_intensity = config_.registration.intensity_residual;
  Pointcloud source_filtered, source_coarse, source_fine;
  Intensities source_intensity;
  DegeneracyInfo degeneracy;
  sampleSource(points_undistorted_I, T_W_I_init, source_filtered, source_coarse, source_fine,
               use_intensity ? &intensities : nullptr,
               use_intensity ? &source_intensity : nullptr,
               config_.publish_degeneracy ? &degeneracy : nullptr);
  voxel_timer.Stop();

  // Perform the actual registration
  timing::Timer align_timer("05_registration");
  LsqRegistration optimizer(*map_, source_filtered, config_.registration,
                            use_intensity ? &source_intensity : nullptr);
  const Transform T_W_I = optimizer.computeTransformation(T_W_I_init);
  const int n_effective_points = optimizer.numEffectivePoints();
  align_timer.Stop();

  // Transform the full cloud using the estimated pose and add it to the map
  timing::Timer map_timer("06_map");
  const Pointcloud points_registered = T_W_I * points_undistorted_I;
  map_->integratePoints(points_registered, &ranges, &intensities);
  map_timer.Stop();

  // Bookkeeping and optimization of the intertial part of the state
  addState(imu.back().stamp, T_W_I.quaternion(), T_W_I.translation(), x_j_pred.v);

  timing::Timer imu_opt("07_imu_opt");
  if (map_->size() > config_.min_map_size_for_imu_opt) {
    addImuIntegrator(imu_integrator);
  }
  optimizeInertialWindow();
  imu_opt.Stop();

  // Publish results
  timing::Timer pub_time("08_publish");
  publishFrame(header, T_W_I, points_registered, source_filtered, source_coarse, source_fine,
               points_undistorted_I, intensities, use_intensity ? &source_intensity : nullptr);
  if (config_.publish_intensity_map) publishIntensityMap(header, T_W_I);
  if (config_.publish_degeneracy && degeneracy.valid) {
    // Drawn 2 m behind the IMU (along its -x axis), so the arrows do not overlap
    // with the robot and the clouds around it.
    degeneracy.origin = T_W_I * V3(-2.0, 0.0, 0.0);
    // Eigenvector signs are arbitrary, so point each direction forward (along
    // the IMU's x axis) to keep the arrows from flipping between frames.
    const V3 forward = T_W_I.linear().col(0);
    for (int k = 0; k < 3; ++k) {
      if (degeneracy.eigenvectors.col(k).dot(forward) < 0.0) {
        degeneracy.eigenvectors.col(k) *= -1.0;
      }
    }
    publish(degeneracy, header, "degeneracy");
  }
  pub_time.Stop();

  step_timer.Stop();

  if (!config_.log_path.empty()) {
    logTUM(nsToS(points_L.end_stamp), T_W_I);
  }

  if (config_.print_timing) {
    LOG(I, "Timings:\n" << timing::Timing::Print());
  }

  if (config_.print_dashboard) {
    printDashboard(dashboard_, header.stamp, T_W_I, x_j_pred.v, acc_bias_, gyro_bias_,
                   timing::Timing::GetMeanSeconds("step"), timing::Timing::GetMaxSeconds("step"),
                   n_effective_points);
  }
}

bool Pipeline::initializeBias(const std::vector<ImuMeasurement>& imu_data,
                              const Pointcloud& pointcloud, const IntensityView& intensities) {
  if (!bias_initializer_) {
    bias_initializer_ =
        std::make_unique<BiasInitializer>(config_.imu.t_init, config_.imu.normalized);
  }
  for (const auto& imu : imu_data) {
    if (bias_initializer_->addMeasurement(imu)) break;
  }
  if (!bias_initializer_->isReady()) return false;

  acc_bias_ = bias_initializer_->accBias();
  gyro_bias_ = bias_initializer_->gyroBias();
  imu_acc_scale_ = bias_initializer_->accScale();
  const Rotation R_est = bias_initializer_->initialOrientation();
  bias_initializer_.reset();

  LOG(I, "Bias initialized: " << acc_bias_.transpose() << " " << gyro_bias_.transpose());
  const Eigen::Vector3d euler = R_est.eulerAngles(2, 1, 0);  // yaw, pitch, roll
  LOG(I, "Initial Pitch: " << (180. / M_PI) * euler[1]);
  LOG(I, "Initial Roll: " << (180. / M_PI) * euler[2]);

  State x_init;
  x_init.quat = R_est;
  x_init.p = V3::Zero();
  x_init.v = V3::Zero();

  addState(imu_data.back().stamp, x_init.quat, x_init.p, x_init.v);

  if (pointcloud.empty()) {
    phase_ = Phase::NeedMap;
  } else {
    const Transform T_W_I_init(x_init.quat, x_init.p);
    std::vector<double> ranges(pointcloud.size(), 0.f);
    for (size_t i = 0; i < pointcloud.size(); ++i) {
      ranges[i] = pointcloud[i].head<3>().norm();
    }
    map_->integratePoints(T_W_I_init * pointcloud, &ranges, &intensities);
    phase_ = Phase::Running;
  }
  return true;
}

void Pipeline::tryInitMap(uint64_t stamp, const State& x_j_pred, const Transform& T_W_I_init,
                          const Pointcloud& undistorted, const IntensityView& intensities,
                          std::vector<double>& ranges, const Header& header) {
  if (undistorted.size() < config_.min_points_for_map_init) return;
  const Pointcloud registered = T_W_I_init * undistorted;
  map_->integratePoints(registered, &ranges, &intensities);
  addState(stamp, x_j_pred.quat, x_j_pred.p, x_j_pred.v);
  publishLatestState(header);
  publish(IntensityPointcloud(registered, intensities), header, "points/registered");
  if (map_->size() > config_.map_size_running_threshold) {
    phase_ = Phase::Running;
  }
}

void Pipeline::sampleSource(const Pointcloud& undistorted, const Transform& T_W_I_init,
                            Pointcloud& filtered, Pointcloud& coarse, Pointcloud& fine,
                            const IntensityView* intensities,
                            Intensities* filtered_intensity,
                            DegeneracyInfo* degeneracy) const {
  const bool use_intensity = intensities != nullptr && filtered_intensity != nullptr;
  Pointcloud source_down;
  std::vector<size_t> down_indices;
  voxelDownsample(undistorted, source_down, config_.preprocess.downsample_resolution,
                  use_intensity ? &down_indices : nullptr);

  // Downsampling only selects points, so the indices carry each kept point's
  // intensity across without widening the cloud type.
  Intensities down_intensity;
  std::vector<size_t> idx_intensity;
  if (use_intensity) {
    down_intensity.resize(down_indices.size());
    for (size_t i = 0; i < down_indices.size(); ++i) {
      down_intensity(i) = (*intensities)(down_indices[i]);
    }
    sampleTopIntensityVoxels(*map_, T_W_I_init, source_down, down_intensity, config_.preprocess,
                             idx_intensity, degeneracy);
  }

  if (!config_.preprocess.informed_sampling) {
    filtered = source_down;
    if (use_intensity) {
      // Every downsampled point already registers, so the selection only decides
      // which of them additionally carry a photometric residual.
      *filtered_intensity = Intensities::Constant(source_down.size(), -1.0);
      for (size_t i : idx_intensity) (*filtered_intensity)(i) = down_intensity(i);
    }
    return;
  }

  std::vector<size_t> coarse_indices;
  std::vector<size_t> fine_indices;
  sampleInformed(*map_, T_W_I_init, source_down, coarse, fine,
                 config_.preprocess.downsample_resolution, config_.informed_sample_count,
                 use_intensity ? &coarse_indices : nullptr, use_intensity ? &fine_indices : nullptr);

  if (!use_intensity) {
    filtered = fine + coarse;
    return;
  }

  // The photometric selection runs on the full downsampled cloud, so it reaches
  // voxels informed sampling threw away — which is the whole point, since that
  // ranking is geometric and in a tunnel it discards exactly the flat, textured
  // surfaces the photometric term exists to exploit. Its points lead the cloud and
  // the geometric picks are deduplicated against them, so nothing appears twice.
  ankerl::unordered_dense::set<size_t> intensity_set(idx_intensity.begin(), idx_intensity.end());
  std::vector<size_t> geometric_indices;
  geometric_indices.reserve(fine_indices.size() + coarse_indices.size());
  for (size_t i : fine_indices) {
    if (!intensity_set.contains(i)) geometric_indices.push_back(i);
  }
  for (size_t i : coarse_indices) {
    if (!intensity_set.contains(i)) geometric_indices.push_back(i);
  }

  const size_t n_intensity = idx_intensity.size();
  filtered.resize(n_intensity + geometric_indices.size());
  filtered_intensity->resize(filtered.size());
  for (size_t i = 0; i < n_intensity; ++i) {
    filtered[i] = source_down[idx_intensity[i]];
    (*filtered_intensity)(i) = down_intensity(idx_intensity[i]);
  }
  for (size_t i = 0; i < geometric_indices.size(); ++i) {
    filtered[n_intensity + i] = source_down[geometric_indices[i]];
    (*filtered_intensity)(n_intensity + i) = -1.0;
  }
}

bool Pipeline::addState(const uint64_t time, const Quaternion& quat, const V3& p, const V3& v) {
  if (states_.find(time) != states_.end()) {
    LOG(D, "Time " << time << " already exists in the Pipeline. Skipping.");
    return false;
  }

  states_[time] = {seq_counter_++, quat, p, v};

  const double allowed_oldest_time_s = nsToS(time) - config_.imu.window_length_s;
  if (allowed_oldest_time_s < 0) return true;

  const uint64_t allowed_oldest_time = sToNs(allowed_oldest_time_s);
  for (auto it = states_.begin(); it != states_.end();) {
    // Remove states that are outside of the window length, along with their corresponding IMU
    // integrators
    if (it->first >= allowed_oldest_time) break;
    imu_integrators_.erase(it->first);
    it = states_.erase(it);
  }
  return true;
}

bool Pipeline::addImuIntegrator(ImuIntegratorPtr imu_integrator) {
  uint64_t time = imu_integrator->getFirstTime();
  if (states_.find(time) == states_.end()) {
    LOG(D, "Time " << time << " does not exist in the Pipeline. Skipping.");
    return false;
  }

  imu_integrators_[time] = imu_integrator;
  return true;
}

bool Pipeline::optimizeInertialWindow() {
  const size_t n_states = states_.size();
  const size_t n_imu = imu_integrators_.size();
  if (n_states == 0 || n_imu == 0) {
    LOG(D, "No states or IMU integrators to optimize.");
    return false;
  }

  // Skip if there are not enough IMU integrators to prevent instable optimization
  if (n_imu < config_.min_imu_integrators_for_opt) return false;

  ceres::Problem problem;
  ceres::Solver::Options options;
  options.logging_type = ceres::SILENT;
  ceres::LossFunction* loss_function = new ceres::TrivialLoss();

  bool first = true;
  for (auto integrator_it : imu_integrators_) {
    ImuIntegratorPtr& imu_integrator = integrator_it.second;
    const uint64_t t_i = imu_integrator->getFirstTime();
    const uint64_t t_j = imu_integrator->getLastTime();
    State& state_i = states_.at(t_i);
    State& state_j = states_.at(t_j);

    ceres::CostFunction* cost_function = new InertialFactor(imu_integrator);

    // Add preintegrated IMU cost factors between consecutive states, with shared bias and gravity
    // parameters
    problem.AddResidualBlock(cost_function, loss_function, state_i.quat.coeffs().data(),
                             state_i.p.data(), state_i.v.data(), state_j.quat.coeffs().data(),
                             state_j.p.data(), state_j.v.data(), acc_bias_.data(),
                             gyro_bias_.data(), gravity_dir_.data());
    // Keep poses fixed as we only want to optimize over velocities, biases and gravity direction
    problem.SetParameterBlockConstant(state_i.quat.coeffs().data());
    problem.SetParameterBlockConstant(state_j.quat.coeffs().data());
    problem.SetParameterBlockConstant(state_i.p.data());
    problem.SetParameterBlockConstant(state_j.p.data());

    if (first) {
      problem.SetParameterBlockConstant(state_i.v.data());
      first = false;
    }
  }

  // Add constant prior to add some rigidity to the optimization problem and prevent gravity from
  // drifting
  ceres::CostFunction* prior_cost_function =
      new PriorFactor(gravity_dir_, config_.gravity_prior_weight);
  problem.AddResidualBlock(prior_cost_function, loss_function, gravity_dir_.data());
  ceres::Manifold* gravity_manifold = new ceres::SphereManifold<3>();
  problem.SetManifold(gravity_dir_.data(), gravity_manifold);

  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

  return true;
}

void Pipeline::publishFrame(const Header& header, const Transform& T_W_I,
                            const Pointcloud& full_registered, const Pointcloud& source_filtered,
                            const Pointcloud& source_coarse, const Pointcloud& source_fine,
                            const Pointcloud& undistorted, const IntensityView& intensities,
                            const Intensities* source_intensity) {
  if (config_.publish_all_clouds) {
    publishDebugClouds(source_filtered, source_coarse, source_fine, undistorted, intensities, T_W_I,
                       header, source_intensity);
  }
  publish(IntensityPointcloud(full_registered, intensities), header, "points/registered");
  publishLatestState(header);
}

void Pipeline::publishIntensityMap(const Header& header, const Transform& T_W_I) {
  if (!intensity_mesh_builder_) return;
  // Publishers are registered by the interface after construction, so this can
  // only be checked here. Interfaces built without a mesh message (e.g. ROS
  // without nvblox_msgs) do not register IntensityMesh.
  if (!config_.map.use_intensity || !hasPublisher<IntensityMesh>()) {
    if (!intensity_map_warned_) {
      LOG(W, "publish_intensity_map is enabled, but "
                 << (config_.map.use_intensity
                         ? "this interface was built without intensity map support (nvblox_msgs)."
                         : "the intensity residual is disabled, so the map has no intensity "
                           "images.")
                 << " Not publishing the intensity map.");
      intensity_map_warned_ = true;
    }
    return;
  }
  const uint64_t period_ns = static_cast<uint64_t>(config_.intensity_map_period_s * 1e9);
  if (last_intensity_map_stamp_ != 0 && header.stamp >= last_intensity_map_stamp_ &&
      header.stamp - last_intensity_map_stamp_ < period_ns) {
    return;
  }
  last_intensity_map_stamp_ = header.stamp;

  timing::Timer timer("08_publish/intensity_map");
  const IntensityMesh mesh = intensity_mesh_builder_->update(*map_, T_W_I.translation());
  timer.Stop();
  if (mesh.blocks.empty()) return;
  // Conversion to the ROS message, serialization and sending, timed separately
  // from building the mesh.
  timing::Timer send_timer("08_publish/intensity_map_send");
  publish(mesh, header, "intensity_map");
  send_timer.Stop();
}

void Pipeline::publishLatestState(const Header& header) {
  if (states_.empty()) {
    LOG(W, "No states to publish.");
    return;
  }

  const State& latest_state = states_.rbegin()->second;
  Odometry odom;
  odom.pose = Transform(latest_state.quat, latest_state.p);
  // The state velocity is expressed in the world frame; rotate it into the body
  // frame so it matches the odometry message's child frame. The angular velocity
  // comes straight from the latest gyro measurement (already in the body frame).
  odom.linear_velocity = latest_state.quat.conjugate() * latest_state.v;
  odom.angular_velocity = latest_gyro_;
  publish(odom, header, "odom", config_.body_frame);
  publish(acc_bias_, header, "bias/acc");
  publish(gyro_bias_, header, "bias/gyro");
}

void Pipeline::publishDebugClouds(const Pointcloud& source_filtered,
                                  const Pointcloud& source_coarse, const Pointcloud& source_fine,
                                  const Pointcloud& undistorted_cloud,
                                  const IntensityView& intensities, const Transform& T_W_I,
                                  const Header& header, const Intensities* source_intensity) {
  Pointcloud source_registered = T_W_I * source_filtered;
  Pointcloud fine_registered = T_W_I * source_fine;
  Pointcloud coarse_registered = T_W_I * source_coarse;
  publish(fine_registered, header, "points/fine");
  publish(coarse_registered, header, "points/coarse");
  publish(source_registered, header, "points/effective");
  if (source_intensity) {
    // sampleSource marks the points that carry a photometric residual with their
    // intensity and every other point with -1, so this is exactly the selection.
    const Intensities& values = *source_intensity;
    const size_t n_selected = static_cast<size_t>((values.array() >= 0.0).count());
    IntensityPointcloud selected;
    selected.resize(n_selected);
    for (size_t i = 0, k = 0; i < source_registered.size(); ++i) {
      if (values(i) < 0.0) continue;
      selected.data().col(k).head<3>() = source_registered[i];
      selected.data()(3, k) = values(i);
      ++k;
    }
    publish(selected, header, "points/intensity_selected");
  }
  Header body_header = header;
  body_header.frame = config_.body_frame;
  // The undistorted cloud keeps its original point order, so the snapshotted
  // intensity row still lines up with it.
  publish(IntensityPointcloud(undistorted_cloud, intensities), body_header, "points/undistorted");
}

void Pipeline::logTUM(double timestamp, const Transform& pose) {
  const Quaternion q(pose.linear());
  const V3& t = pose.translation();
  if (!tum_log_->is_open()) {
    LOG(E, "Error: Output file is not open for logging.");
    return;
  }

  (*tum_log_) << std::fixed;
  (*tum_log_) << timestamp << " " << t.x() << " " << t.y() << " " << t.z() << " " << q.x() << " "
              << q.y() << " " << q.z() << " " << q.w() << "\n";
}

}  // namespace coin_bievr
