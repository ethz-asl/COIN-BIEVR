// Adapted from COIN-LIO (https://github.com/ethz-asl/COIN-LIO)
// Copyright (c) 2024, Patrick Pfreundschuh, BSD-3-Clause License

#include "coin_bievr/ouster_projector.h"

#include <tbb/blocked_range.h>
#include <tbb/parallel_for.h>

#include "coin_bievr/log++.h"
#include "coin_bievr/timing.h"


namespace coin_bievr {

OusterProjector::OusterProjector(const OusterProjectorConfig& config,
                                       const ProcessConfig& process_config)
    : BaseIntensityProjector(process_config), rows_(config.rows), cols_(config.cols) {
  if (rows_ == 0 || cols_ == 0) {
    LOG(E, "Ouster projector needs a non-zero image size. Photometric preprocessing disabled.");
    return;
  }
  if (config_.destagger && config_.offset_lut.size() < rows_) {
    LOG(E, "Ouster projector needs an offset_lut with one entry per row ("
               << rows_ << "), got " << config_.offset_lut.size()
               << ". Photometric preprocessing disabled.");
    return;
  }

  const int cols = static_cast<int>(cols_);
  idx_to_v_.assign(rows_ * cols_, 0);
  idx_to_u_.assign(rows_ * cols_, 0);
  for (size_t i = 0; i < rows_; ++i) {
    for (int j = 0; j < cols; ++j) {
      // The sensor emits each row shifted by a per-row offset; undoing it gives
      // the scan index that point (i, j) arrives at.
      const int destaggered = config_.destagger ? (j + cols - config_.offset_lut[i]) % cols : j;
      const size_t idx = i * cols_ + destaggered;
      int u = j - config_.u_shift;
      if (u < 0) u += cols;
      if (u >= cols) u -= cols;
      idx_to_v_[idx] = static_cast<int>(i);
      idx_to_u_[idx] = u;
    }
  }
  valid_ = true;
}

void OusterProjector::processIntensity(StampedIntensityPointcloud& cloud,
                                          DebugImages* debug) const {
  if (!valid_ || cloud.empty()) return;

  const size_t n = cloud.size();
  if (n != rows_ * cols_) {
    LOG(E, "Ouster projector expects the full organized scan of "
               << rows_ * cols_ << " points, got " << n << ". Skipping photometric preprocessing.");
    return;
  }

  timing::Timer proj_timer("00_projection/project");
  cv::Mat image = cv::Mat::zeros(static_cast<int>(rows_), static_cast<int>(cols_), CV_32FC1);
  tbb::parallel_for(tbb::blocked_range<size_t>(0, n), [&](const tbb::blocked_range<size_t>& r) {
    for (size_t j = r.begin(); j < r.end(); ++j) {
      image.ptr<float>(idx_to_v_[j])[idx_to_u_[j]] =
          cloud[j](StampedIntensityPointcloud::kIntensityRow) * config_.intensity_scale;
    }
  });
  proj_timer.Stop();

  timing::Timer post_timer("00_projection/post");
  applyPostProjection(image, debug);
  // Masked regions are invalidated after processing so they still contribute to
  // the local statistics but never reach the map.
  applyMasks(image, -1.f);
  post_timer.Stop();

  timing::Timer rewrite_timer("00_projection/rewrite");

  tbb::parallel_for(tbb::blocked_range<size_t>(0, n), [&](const tbb::blocked_range<size_t>& r) {
    for (size_t j = r.begin(); j != r.end(); ++j) {
      cloud[j](StampedIntensityPointcloud::kIntensityRow) =
          image.ptr<float>(idx_to_v_[j])[idx_to_u_[j]];
    }
  });
  rewrite_timer.Stop();
}

}  // namespace coin_bievr
