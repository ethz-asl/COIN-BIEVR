// Adapted from COIN-LIO (https://github.com/ethz-asl/COIN-LIO)
// Copyright (c) 2024, Patrick Pfreundschuh, BSD-3-Clause License

#ifndef COIN_BIEVR_OUSTER_PROJECTOR_H_
#define COIN_BIEVR_OUSTER_PROJECTOR_H_

#include "coin_bievr/base_intensity_projector.h"

namespace coin_bievr {

// Ouster projector. Rather than projecting geometrically, it maps each
// point to its pixel through the scan index, which is exact but requires the
// full organized scan (rows * cols points, in sensor order).
class OusterProjector : public BaseIntensityProjector {
 public:
  OusterProjector(const OusterProjectorConfig& config, const ProcessConfig& process_config);

  void processIntensity(StampedIntensityPointcloud& cloud,
                        DebugImages* debug = nullptr) const override;

 private:
  size_t rows_;
  size_t cols_;
  bool valid_{false};
  std::vector<int> idx_to_v_;  // scan index -> image row
  std::vector<int> idx_to_u_;  // scan index -> image column (destaggered, shifted)
};

}  // namespace coin_bievr
#endif  // COIN_BIEVR_OUSTER_PROJECTOR_H_
