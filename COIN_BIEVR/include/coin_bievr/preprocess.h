#ifndef COIN_BIEVR_PREPROCESS_H_
#define COIN_BIEVR_PREPROCESS_H_

#include "coin_bievr/coin_bievr_map.h"
#include "coin_bievr/common.h"

namespace coin_bievr {

struct PreprocessConfig {
  size_t n_samples = 1000;
  bool informed_sampling = true;
  double min_range = 0.5;              // meters
  double max_range = 100.0;            // meters
  double downsample_resolution = 0.1;  // meters
  // Photometric point selection, only consulted when the intensity residual is on.
  double min_intensity = -1.0;               // drop points below this, < 0 disables
  size_t n_intensity_voxels = 100;           // voxels whose points keep the residual
  double intensity_max_range = 0.0;          // meters, 0 disables the range limit
  double intensity_min_incidence_deg = 0.0;  // 0 disables the grazing-angle limit
};

// The optional index out-params report, for each emitted point, which point of
// points_raw it came from. Both samplers select existing points rather than
// averaging them, so the mapping is exact. Callers use it to carry per-point side
// channels (e.g. intensity) through sampling without widening the cloud type.
void voxelDownsample(const Pointcloud& points_raw, Pointcloud& points_down, double voxel_size,
                     std::vector<size_t>* selected_indices = nullptr);

void sampleInformed(const COINBIEVRMap& map, const Transform& T_W_L, const Pointcloud& points_raw,
                    Pointcloud& points_coarse, Pointcloud& points_fine, double voxel_size,
                    size_t n_samples, std::vector<size_t>* coarse_indices = nullptr,
                    std::vector<size_t>* fine_indices = nullptr);

// Geometric degeneracy estimated by sampleTopIntensityVoxels from the normals of
// the voxels the scan touches (H = sum n n^T). Eigenvalues are ascending, so the
// first `n_degenerate` eigenvector columns are the poorly constrained
// directions. Eigenvector signs are arbitrary.
struct DegeneracyInfo {
  bool valid = false;
  V3 origin = V3::Zero();                          // where to draw it, map frame
  V3 eigenvalues = V3::Zero();                     // ascending
  Eigen::Matrix3d eigenvectors = Eigen::Matrix3d::Identity();  // columns, map frame
  int n_degenerate = 0;                            // 1 (line) or 2 (plane)
};

// Finds the voxels where the photometric residual can actually add information:
// those whose intensity texture runs along the direction the geometric residual
// leaves unconstrained (e.g. the axis of a tunnel). Appends to out_indices every
// point of `points` falling in one of them, so the count is "points in the top
// voxels" rather than a fixed number of points. `points` is index-aligned with
// `intensities` and lives in the frame T_W_L maps into the map frame.
void sampleTopIntensityVoxels(const COINBIEVRMap& map, const Transform& T_W_L, const Pointcloud& points,
                              const Intensities& intensities, const PreprocessConfig& config,
                              std::vector<size_t>& out_indices,
                              DegeneracyInfo* degeneracy = nullptr);

template <typename T>
concept HasStamp = requires(T t) {
  t.stamp;
  t.end_stamp;
};

template <typename PointcloudT>
void filterMinMaxRange(const PointcloudT& points_raw, PointcloudT& points_filtered,
                       const double min_range = 0.0,
                       const double max_range = std::numeric_limits<double>::max()) {
  bool filter_min = (min_range > 0.0);
  bool filter_max = (max_range < std::numeric_limits<double>::max());

  if (!filter_min && !filter_max) {
    points_filtered = points_raw;
    return;
  }

  // When a filter is disabled its bound is a no-op: min_range defaults to 0
  // (range_squared >= 0 always), and max_range defaults to double max whose
  // square overflows to +inf (range_squared <= inf always).
  const double min_range_squared = min_range * min_range;
  const double max_range_squared = max_range * max_range;

  size_t n_points = points_raw.size();
  std::vector<char> point_selected(n_points, 0);
  tbb::parallel_for(
      tbb::blocked_range<size_t>(0, n_points), [&](const tbb::blocked_range<size_t>& r) {
        for (size_t i = r.begin(); i != r.end(); ++i) {
          const auto& point = points_raw[i];
          double range_squared =
              point.x() * point.x() + point.y() * point.y() + point.z() * point.z();
          if (range_squared >= min_range_squared && range_squared <= max_range_squared) {
            point_selected[i] = 1;
          }
        }
      });

  // Extract selected point indices
  std::vector<size_t> selected_indices;
  selected_indices.reserve(n_points);
  for (size_t i = 0; i < n_points; ++i) {
    if (point_selected[i] == 1) {
      selected_indices.push_back(i);
    }
  }

  points_filtered.resize(selected_indices.size());
  tbb::parallel_for(tbb::blocked_range<size_t>(0, selected_indices.size()),
                    [&](const tbb::blocked_range<size_t>& r) {
                      for (size_t i = r.begin(); i != r.end(); ++i) {
                        points_filtered[i] = points_raw[selected_indices[i]];
                      }
                    });

  if constexpr (HasStamp<PointcloudT>) {
    points_filtered.stamp = points_raw.stamp;
    points_filtered.end_stamp = points_raw.end_stamp;
  }
}

// Drops points whose intensity is below min_intensity, removing them from the scan
// entirely: they contribute neither a geometric residual nor map content. Returns
// unfiltered on a negative threshold. Photometric normalization has to have run
// first, since the threshold is expressed in normalized intensity units.
inline void filterMinIntensity(const StampedIntensityPointcloud& points_raw,
                               StampedIntensityPointcloud& points_filtered,
                               const double min_intensity) {
  if (min_intensity < 0.0) {
    points_filtered = points_raw;
    return;
  }

  const size_t n_points = points_raw.size();
  std::vector<char> point_selected(n_points, 0);
  tbb::parallel_for(tbb::blocked_range<size_t>(0, n_points),
                    [&](const tbb::blocked_range<size_t>& r) {
                      for (size_t i = r.begin(); i != r.end(); ++i) {
                        if (points_raw[i](StampedIntensityPointcloud::kIntensityRow) >=
                            min_intensity) {
                          point_selected[i] = 1;
                        }
                      }
                    });

  std::vector<size_t> selected_indices;
  selected_indices.reserve(n_points);
  for (size_t i = 0; i < n_points; ++i) {
    if (point_selected[i] == 1) {
      selected_indices.push_back(i);
    }
  }

  points_filtered.resize(selected_indices.size());
  tbb::parallel_for(tbb::blocked_range<size_t>(0, selected_indices.size()),
                    [&](const tbb::blocked_range<size_t>& r) {
                      for (size_t i = r.begin(); i != r.end(); ++i) {
                        points_filtered[i] = points_raw[selected_indices[i]];
                      }
                    });

  points_filtered.stamp = points_raw.stamp;
  points_filtered.end_stamp = points_raw.end_stamp;
}

}  // namespace coin_bievr
#endif  // COIN_BIEVR_PREPROCESS_H_