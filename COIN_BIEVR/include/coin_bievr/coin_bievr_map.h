#ifndef COIN_BIEVR_BIEVR_MAP_H_
#define COIN_BIEVR_BIEVR_MAP_H_

#include <tbb/parallel_for.h>
#include <tbb/parallel_sort.h>

#include <cmath>
#include <cstdint>
#include <list>
#include <utility>
#include <vector>

#include "coin_bievr/common.h"
#include "unordered_dense/unordered_dense.h"

namespace coin_bievr {

// One point as carried through the map update: xyz, the range-derived update weight
// in row 3, and the (already photometrically normalized) intensity in row 4. The
// points get sorted by voxel hash, so every channel has to travel with the point.
using MapPoint = Eigen::Matrix<double, 5, 1>;

struct Voxel {
  bool observed_{false};  // We consider a voxel observed if we have seen enough points inside it
  Transform T_C_W_ = Transform::Identity();
  Transform T_O_W_ = Transform::Identity();
  Eigen::MatrixXf bump_img_;
  Eigen::MatrixXf bump_smoothed_;
  Eigen::MatrixXf bump_weights_;
  // Intensity image, in lockstep with bump_img_: same size, same pixels, and
  // bump_weights_ doubles as its validity mask. Empty unless Config::use_intensity.
  Eigen::MatrixXf intensity_img_;
  Eigen::MatrixXf intensity_smoothed_;
  // Intensity structure tensor S = sum(g g^T) over the voxel image, with g the
  // gradient in voxel-frame (x, y). For any in-plane direction d, d^T S d =
  // sum((g . d)^2) is how much texture the photometric term can exploit along d.
  // Quadratic in d, so it is independent of d's sign. Zero unless
  // Config::use_intensity.
  Eigen::Matrix2d int_struct_ = Eigen::Matrix2d::Zero();
  M3 outer_sum_ = Eigen::Matrix3d::Zero();
  V3 sum_ = Eigen::Vector3d::Zero();
  size_t num_points_{0};
  double mean_img_dist_{0.0};
  // Incremented whenever the voxel images change, so consumers such as the
  // intensity map visualization can tell which voxels need to be resent.
  uint32_t version_{0};
  // Raw points accumulated while the voxel is not yet observed (no normal yet, so they cannot be
  // projected into a bump image). Cleared once the voxel becomes observed.
  std::vector<MapPoint> pending_points_;
};

class COINBIEVRMap {
 public:
  struct Config {
    size_t max_size{10000000};  // maximum number of voxels to store
    double voxel_size{0.5};     // voxel side length
    double px_size{0.05};       // size of bump image pixels
    bool weighted = false;      // use range weighted update for bump image
    bool smooth = false;        // apply gaussian smoothing to bump image
    double norm_tol_deg{3.0};   // if normal changes more than this, reproject bump image
    bool use_intensity = false;    // maintain a per-voxel intensity image alongside the bump image
    bool smooth_intensity = true;  // apply gaussian smoothing to the intensity image
  };

  explicit COINBIEVRMap(Config config);

  // intensities, when given, must be index-aligned with input_cloud.
  bool integratePoints(const Pointcloud& input_cloud, const std::vector<double>* ranges = nullptr,
                       const IntensityView* intensities = nullptr);

  inline size_t hashIndex(const Point& point) const {
    Eigen::Vector3i voxel_idx = (point * inv_voxel_size_).array().floor().cast<int>();

    return hashIndexVoxel(voxel_idx);
  }
  size_t size() const { return map_.size(); }
  Eigen::Vector3i getVoxelIdx(const Point& point) const;
  const Voxel* getVoxel(const size_t hash_idx) const;
  bool nearestVoxel(const Point& point, size_t& result) const;

  // Every observed voxel whose cell center lies within `radius` of `center`, as
  // (grid index, voxel). Walks the grid cells around `center` rather than the
  // whole map, so the cost depends on the radius, not the map size. The x-slices
  // are searched in parallel; the result is in a deterministic order. The
  // pointers stay valid until the map is next modified.
  std::vector<std::pair<Eigen::Vector3i, const Voxel*>> observedVoxelsInRadius(
      const Point& center, double radius) const {
    const Eigen::Vector3i c = (center * inv_voxel_size_).array().floor().cast<int>();
    const int r = static_cast<int>(std::ceil(radius * inv_voxel_size_));
    const double radius_sq = radius * radius;
    std::vector<std::vector<std::pair<Eigen::Vector3i, const Voxel*>>> slices(2 * r + 1);
    tbb::parallel_for(0, 2 * r + 1, [&](int s) {
      const int dx = s - r;
      for (int dy = -r; dy <= r; ++dy) {
        for (int dz = -r; dz <= r; ++dz) {
          const Eigen::Vector3i idx = c + Eigen::Vector3i(dx, dy, dz);
          const Point cell_center = (idx.cast<double>().array() + 0.5) * config_.voxel_size;
          if ((cell_center - center).squaredNorm() > radius_sq) continue;
          const auto it = map_.find(hashIndexVoxel(idx));
          if (it == map_.end() || !it->second.voxel.observed_) continue;
          slices[s].emplace_back(idx, &it->second.voxel);
        }
      }
    });
    std::vector<std::pair<Eigen::Vector3i, const Voxel*>> voxels;
    for (auto& slice : slices) voxels.insert(voxels.end(), slice.begin(), slice.end());
    return voxels;
  }

  const double& voxel_size = config_.voxel_size;
  const double& pixel_size = config_.px_size;
  const double& inv_px_size = inv_px_size_;

 private:
  struct ImageBounds {
    int width;
    int height;
    double u_min;
    double v_min;
  };

  // Voxel update / bump-image pipeline, in the order integratePoints invokes them.
  bool updateNormal(Voxel& voxel);

  bool updateBumpImage(const std::vector<MapPoint>& points, Voxel& voxel, bool normal_change);

  ImageBounds computeImageSize(const Voxel& voxel, const Point& reference_point) const;

  void reprojectImage(Voxel& voxel, const ImageBounds& bounds, Eigen::MatrixXi& changed);

  void integratePoints(const std::vector<MapPoint>& points, Voxel& voxel, Eigen::MatrixXi& changed);

  void dilateMask(const Eigen::MatrixXi& changed, const Eigen::MatrixXf& weights,
                  Eigen::MatrixXi& changed_dilated);

  void maskedGaussianSmooth(const Eigen::MatrixXf& image, const Eigen::MatrixXf& weights,
                            const Eigen::MatrixXi& changed, Eigen::MatrixXf& image_smooth);

  void computeScore(Voxel& voxel);

  void computeIntensityGradient(Voxel& voxel);

  // Geometry helper.
  Eigen::Vector3d getVoxelOrigin(const Point& point) const;

  Eigen::MatrixXf gauss_kernel_;
  Config config_;
  double inv_voxel_size_{1.0};
  double inv_px_size_{0.05};            // inv size of bump image
  double norm_tol_rad_{3.0};            // if normal changes more than this, update bump image
  static constexpr int kNMinValid = 4;  // minimum points for a valid voxel

  struct VoxelEntry {
    Voxel voxel;
    std::list<size_t>::iterator lru_it;  // iterator into voxels_cache_
  };
  ankerl::unordered_dense::map<size_t, VoxelEntry> map_;
  std::list<size_t> voxels_cache_;

  Eigen::MatrixXd corner_offsets_;
  std::vector<Point> neighbor_offsets_;
};

using VoxelPtr = std::shared_ptr<Voxel>;
using ConstVoxelPtr = std::shared_ptr<const Voxel>;

}  // namespace coin_bievr
#endif  // COIN_BIEVR_BIEVR_MAP_H_