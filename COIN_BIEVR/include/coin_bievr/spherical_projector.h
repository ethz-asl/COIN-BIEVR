// Adapted from COIN-LIO (https://github.com/ethz-asl/COIN-LIO)
// Copyright (c) 2024, Patrick Pfreundschuh, BSD-3-Clause License

#ifndef COIN_BIEVR_SPHERICAL_PROJECTOR_H_
#define COIN_BIEVR_SPHERICAL_PROJECTOR_H_

#include "coin_bievr/base_intensity_projector.h"

namespace coin_bievr {

// Spherical projector: azimuth and elevation map onto a uniform equirectangular
// grid spanning 360 deg horizontally and [fov_down, fov_up] vertically. Needs no
// sensor-specific model, so it works for any spinning or solid-state LiDAR.
class SphericalProjector : public BaseIntensityProjector {
 public:
  SphericalProjector(const SphericalProjectorConfig& config, const ProcessConfig& process_config);

  void processIntensity(StampedIntensityPointcloud& cloud,
                        DebugImages* debug = nullptr) const override;

  // Returns false if the point falls outside the vertical field of view.
  bool projectPoint(const Point& point, int& u, int& v) const;

 private:
  int width_;
  int height_;
  double fov_up_rad_;
  double fov_down_rad_;
  double phi_offset_rad_;
};

}  // namespace coin_bievr
#endif  // COIN_BIEVR_SPHERICAL_PROJECTOR_H_
