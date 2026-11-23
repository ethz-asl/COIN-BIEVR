// Adapted from COIN-LIO (https://github.com/ethz-asl/COIN-LIO)
// Copyright (c) 2024, Patrick Pfreundschuh, BSD-3-Clause License

#include "coin_bievr/spherical_projector.h"

#include <tbb/blocked_range.h>
#include <tbb/parallel_for.h>

#include <cmath>

namespace coin_bievr {

SphericalProjector::SphericalProjector(const SphericalProjectorConfig& config, const ProcessConfig& process_config)
    : BaseIntensityProjector(process_config),
      width_(config.width),
      height_(config.height),
      fov_up_rad_(config.fov_up_deg * M_PI / 180.0),
      fov_down_rad_(config.fov_down_deg * M_PI / 180.0),
      phi_offset_rad_(config.phi_offset_deg * M_PI / 180.0) {}

bool SphericalProjector::projectPoint(const Point& point, int& u, int& v) const {
  const double r_xy = std::sqrt(point.x() * point.x() + point.y() * point.y());
  double phi = std::atan2(point.y(), point.x()) + phi_offset_rad_;
  if (phi > M_PI) phi -= 2.0 * M_PI;
  if (phi < -M_PI) phi += 2.0 * M_PI;
  const double theta = std::atan2(point.z(), r_xy);

  if (theta > fov_up_rad_ || theta < fov_down_rad_) return false;

  const double fov_range = fov_up_rad_ - fov_down_rad_;
  u = static_cast<int>((phi + M_PI) / (2.0 * M_PI) * width_) % width_;
  v = static_cast<int>((fov_up_rad_ - theta) / fov_range * height_);
  return u >= 0 && u < width_ && v >= 0 && v < height_;
}

void SphericalProjector::processIntensity(StampedIntensityPointcloud& cloud, DebugImages* debug) const {
  if (cloud.empty()) return;

  const size_t n = cloud.size();
  std::vector<int> proj_u(n, -1);
  std::vector<int> proj_v(n, -1);

  tbb::parallel_for(tbb::blocked_range<size_t>(0, n), [&](const tbb::blocked_range<size_t>& r) {
    for (size_t j = r.begin(); j != r.end(); ++j) {
      int u = 0;
      int v = 0;
      if (!projectPoint(cloud[j].head<3>(), u, v)) continue;
      proj_u[j] = u;
      proj_v[j] = v;
    }
  });

  cv::Mat image = cv::Mat::zeros(height_, width_, CV_32FC1);
  for (size_t j = 0; j < n; ++j) {
    if (proj_u[j] < 0) continue;
    image.ptr<float>(proj_v[j])[proj_u[j]] =
        cloud[j](StampedIntensityPointcloud::kIntensityRow) * config_.intensity_scale;
  }

  applyMasks(image, 0.f);
  applyPostProjection(image, debug);

  tbb::parallel_for(tbb::blocked_range<size_t>(0, n), [&](const tbb::blocked_range<size_t>& r) {
    for (size_t j = r.begin(); j != r.end(); ++j) {
      // Points outside the vertical FoV have no pixel; -1 marks them as carrying
      // no photometric information, so the map and the residual skip them.
      cloud[j](StampedIntensityPointcloud::kIntensityRow) =
          proj_u[j] < 0 ? -1.f : image.ptr<float>(proj_v[j])[proj_u[j]];
    }
  });
}

}  // namespace coin_bievr
