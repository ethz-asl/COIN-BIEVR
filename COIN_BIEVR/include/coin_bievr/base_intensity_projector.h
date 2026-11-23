// Adapted from COIN-LIO (https://github.com/ethz-asl/COIN-LIO)
// Copyright (c) 2024, Patrick Pfreundschuh, BSD-3-Clause License

#ifndef COIN_BIEVR_BASE_INTENSITY_PROJECTOR_H_
#define COIN_BIEVR_BASE_INTENSITY_PROJECTOR_H_

#include <opencv2/core.hpp>

#include <array>
#include <memory>
#include <string>
#include <vector>

#include "coin_bievr/common.h"

namespace coin_bievr {

// Per-scan photometric normalization. A projector renders the scan into an
// intensity image, normalizes it, and writes the result back into the intensity
// row of the cloud, so it is a pure filter: nothing downstream has to know it ran.
// Subclasses supply only the projection geometry.
class BaseIntensityProjector {
 public:
  struct ProcessConfig {
    bool destagger = true;         // Ouster only: undo the per-row column stagger
    bool remove_lines = true;      // vertical high-pass + horizontal low-pass
    bool brightness_filter = true; // local brightness normalization
    bool blur = true;              // masked 3x3 Gaussian
    double intensity_scale = 1.0;
    std::vector<int> window = {41, 7};      // [width, height] of the brightness filter
    std::vector<double> high_pass_fir;      // vertical FIR for line removal
    std::vector<double> low_pass_fir;       // horizontal FIR for line removal
    std::vector<std::array<int, 4>> masks;  // occlusion masks, [x, y, width, height]
    std::vector<int> offset_lut;            // Ouster destagger LUT, one entry per row
    int u_shift = 0;                        // Ouster column shift
  };

  // Snapshots of the projected image before and after normalization, both
  // clamped to [0, 255]. Only filled when a caller asks for them, so the normal
  // path never pays for the copies. Eigen rather than cv::Mat because this is the
  // one part of the projector the publishers see, and they take Eigen images.
  struct DebugImages {
    Eigen::MatrixXf raw;        // straight out of the projection
    Eigen::MatrixXf processed;  // after line removal, brightness filter and blur
  };

  virtual ~BaseIntensityProjector() = default;

  // Replaces the intensity row of cloud with its normalized version.
  virtual void processIntensity(StampedIntensityPointcloud& cloud,
                                DebugImages* debug = nullptr) const = 0;

 protected:
  explicit BaseIntensityProjector(const ProcessConfig& config);

  // removeLines -> filterBrightness -> blur -> clamp to 255, in place.
  // Images are CV_32FC1 throughout.
  void applyPostProjection(cv::Mat& image, DebugImages* debug = nullptr) const;

  void applyMasks(cv::Mat& image, float value) const;

  ProcessConfig config_;

 private:
  void removeLines(cv::Mat& image) const;
  void filterBrightness(cv::Mat& image) const;

  cv::Mat high_pass_fir_;  // Nx1 column kernel, applied vertically
  cv::Mat low_pass_fir_;   // Mx1 column kernel, transposed to 1xM for horizontal use
  cv::Size window_size_{41, 7};
};

// Geometry of the Ouster-style projector. It maps points by scan index, so the
// image dimensions are all it needs; the stagger and shift live in ProcessConfig.
struct OusterProjectorConfig {
  size_t rows = 128;
  size_t cols = 1024;
};

// Geometry of the spherical projector: a uniform equirectangular grid. Horizontal FoV is always
// 360 deg.
struct SphericalProjectorConfig {
  int width = 1024;
  int height = 64;
  double fov_up_deg = 15.0;
  double fov_down_deg = -15.0;
  // Azimuth offset applied before the column mapping; use -90 when y points
  // forward and x points right.
  double phi_offset_deg = 0.0;
};

struct ProjectorConfig {
  std::string type = "";  // "" disables photometric preprocessing, else "ouster" or "spherical"
  BaseIntensityProjector::ProcessConfig process;
  OusterProjectorConfig ouster;
  SphericalProjectorConfig spherical;
};

// Returns nullptr when config.type is empty; logs and returns nullptr on an
// unknown type.
std::unique_ptr<BaseIntensityProjector> createProjector(const ProjectorConfig& config);

}  // namespace coin_bievr
#endif  // COIN_BIEVR_BASE_INTENSITY_PROJECTOR_H_
