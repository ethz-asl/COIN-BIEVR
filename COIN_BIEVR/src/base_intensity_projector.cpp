// Adapted from COIN-LIO (https://github.com/ethz-asl/COIN-LIO)
// Copyright (c) 2024, Patrick Pfreundschuh, BSD-3-Clause License

#include "coin_bievr/base_intensity_projector.h"

#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cmath>

#include "coin_bievr/spherical_projector.h"
#include "coin_bievr/ouster_projector.h"
#include "coin_bievr/log++.h"
#include "coin_bievr/timing.h"


namespace coin_bievr {

namespace {

// Nx1 CV_32F column kernel from the configured coefficients. Empty in, empty out,
// which is what the remove_lines guard below tests.
cv::Mat toKernel(const std::vector<double>& coefficients) {
  if (coefficients.empty()) return cv::Mat();
  cv::Mat kernel(static_cast<int>(coefficients.size()), 1, CV_32F);
  for (int i = 0; i < kernel.rows; ++i) {
    kernel.at<float>(i, 0) = static_cast<float>(coefficients[i]);
  }
  return kernel;
}

// Debug snapshots cross into Eigen here, the one place the projector's images are
// handed to the publishers. Only reached when a caller asks for them.
void toEigen(const cv::Mat& src, Eigen::MatrixXf& dst) {
  dst.resize(src.rows, src.cols);
  for (int y = 0; y < src.rows; ++y) {
    const float* row = src.ptr<float>(y);
    for (int x = 0; x < src.cols; ++x) dst(y, x) = row[x];
  }
}

}  // namespace

BaseIntensityProjector::BaseIntensityProjector(const ProcessConfig& config) : config_(config) {
  high_pass_fir_ = toKernel(config_.high_pass_fir);
  low_pass_fir_ = toKernel(config_.low_pass_fir);
  if (config_.window.size() >= 2) {
    window_size_ = cv::Size(config_.window[0], config_.window[1]);
  }
}

void BaseIntensityProjector::applyMasks(cv::Mat& image, float value) const {
  for (const auto& mask : config_.masks) {
    const int x = std::clamp(mask[0], 0, image.cols);
    const int y = std::clamp(mask[1], 0, image.rows);
    const int w = std::clamp(mask[2], 0, image.cols - x);
    const int h = std::clamp(mask[3], 0, image.rows - y);
    if (w <= 0 || h <= 0) continue;
    image(cv::Rect(x, y, w, h)).setTo(value);
  }
}

void BaseIntensityProjector::removeLines(cv::Mat& image) const {
  // Column kernel filters vertically; its transpose filters horizontally.
  cv::Mat high_passed;
  cv::filter2D(image, high_passed, CV_32F, high_pass_fir_);
  cv::Mat low_passed;
  cv::filter2D(high_passed, low_passed, CV_32F, low_pass_fir_.t());
  image -= low_passed;
  image.setTo(0, image < 0);
}

void BaseIntensityProjector::filterBrightness(cv::Mat& image) const {
  const cv::Mat src = image.clone();

  cv::Mat sum;
  cv::boxFilter(src, sum, CV_32F, window_size_, cv::Point(-1, -1), /*normalize=*/false,
                cv::BORDER_REFLECT);

  // Only pixels that actually carry a measurement contribute to the local mean.
  cv::Mat mask;
  cv::threshold(src, mask, 0.0, 1.0, cv::THRESH_BINARY);
  cv::Mat count;
  cv::boxFilter(mask, count, CV_32F, window_size_, cv::Point(-1, -1), /*normalize=*/false,
                cv::BORDER_REFLECT);

  const cv::Mat brightness = sum / (count + 1.0f) + 1.0f;
  cv::divide(src, brightness, image);
  image *= 140.0f;
}

void BaseIntensityProjector::applyPostProjection(cv::Mat& image, DebugImages* debug) const {
  // Clamped rather than rescaled, so a mis-set intensity_scale shows up as
  // clipping instead of being hidden by an autoscale.
  if (debug) {
    cv::Mat clamped;
    cv::threshold(image, clamped, 255.0, 255.0, cv::THRESH_TRUNC);
    cv::max(clamped, 0.0, clamped);
    toEigen(clamped, debug->raw);
  }

  if (config_.remove_lines && !high_pass_fir_.empty() && !low_pass_fir_.empty()) {
    timing::Timer lines_timer("00_projection/lines");
    removeLines(image);
    lines_timer.Stop();
  }

  if (config_.brightness_filter) {
    timing::Timer brightness_timer("00_projection/brightness");
    filterBrightness(image);
    brightness_timer.Stop();
  }

  if (config_.blur) {
    timing::Timer blur_timer("00_projection/blur");
    cv::Mat mask;
    cv::threshold(image, mask, 0.0, 1.0, cv::THRESH_BINARY);

    cv::Mat image_blur, mask_blur;
    cv::GaussianBlur(image, image_blur, cv::Size(3, 3), 0);
    cv::GaussianBlur(mask, mask_blur, cv::Size(3, 3), 0);

    // Normalizing by the blurred mask keeps empty pixels from dragging the
    // average down; pixels with no valid neighbour stay empty.
    cv::divide(image_blur, mask_blur, image, 1.0, CV_32F);
    image.setTo(0.0f, mask_blur < 1e-6f);
    blur_timer.Stop();
  }

  cv::threshold(image, image, 255.0, 255.0, cv::THRESH_TRUNC);

  // Captured before the occlusion masks are burned in, so the debug view shows
  // what the normalization produced rather than what the map ends up ignoring.
  if (debug) toEigen(image, debug->processed);
}

std::unique_ptr<BaseIntensityProjector> createProjector(const ProjectorConfig& config) {
  if (config.type.empty()) return nullptr;
  if (config.type == "ouster") {
    return std::make_unique<OusterProjector>(config.ouster, config.process);
  }
  if (config.type == "spherical") {
    return std::make_unique<SphericalProjector>(config.spherical, config.process);
  }
  LOG(E, "Unknown intensity projector type '" << config.type
                                              << "'. Photometric preprocessing disabled.");
  return nullptr;
}

}  // namespace coin_bievr
