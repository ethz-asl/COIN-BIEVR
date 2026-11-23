#include "coin_bievr/intensity_mesh.h"

#include <tbb/blocked_range.h>
#include <tbb/parallel_for.h>

#include <algorithm>

namespace coin_bievr {

IntensityMesh IntensityMeshBuilder::update(const COINBIEVRMap& map, const Point& viewpoint) {
  IntensityMesh mesh;
  mesh.block_size = map.voxel_size;
  const bool full = config_.full_refresh_every > 0 &&
                    update_count_ % static_cast<size_t>(config_.full_refresh_every) == 0;
  ++update_count_;

  const auto voxels = map.observedVoxelsInRadius(viewpoint, config_.radius);

  // Decide which voxels need a new mesh (cheap, serial), then build those in parallel.
  decltype(shown_) current;
  current.reserve(voxels.size());
  std::vector<size_t> to_build;   // indices into `voxels`
  std::vector<char> was_shown;    // per entry of `to_build`
  std::vector<char> positive;     // side to face, per entry of `to_build`
  for (size_t k = 0; k < voxels.size(); ++k) {
    const auto& [idx, voxel] = voxels[k];
    if (voxel->intensity_smoothed_.size() == 0) continue;
    const size_t key = hashIndexVoxel(idx);
    const bool side = facesPositive(*voxel, viewpoint);
    current[key] = {idx, voxel->version_, side};
    const auto it = shown_.find(key);
    const bool shown = it != shown_.end();
    // Rebuild if the images changed or the robot moved to the other side of the plane.
    if (!full && shown && it->second.version == voxel->version_ &&
        it->second.positive_side == side) {
      continue;
    }
    to_build.push_back(k);
    was_shown.push_back(shown);
    positive.push_back(side);
  }

  std::vector<IntensityMeshBlock> built(to_build.size());
  tbb::parallel_for(tbb::blocked_range<size_t>(0, to_build.size()),
                    [&](const tbb::blocked_range<size_t>& r) {
                      for (size_t i = r.begin(); i != r.end(); ++i) {
                        const auto& [idx, voxel] = voxels[to_build[i]];
                        built[i] = buildBlock(map, idx, *voxel, positive[i]);
                      }
                    });
  mesh.blocks.reserve(built.size());
  for (size_t i = 0; i < built.size(); ++i) {
    // An empty block removes the voxel, which only matters if it was shown.
    if (!built[i].vertices.empty() || was_shown[i]) mesh.blocks.push_back(std::move(built[i]));
  }

  const auto add_removal = [&mesh](const Eigen::Vector3i& idx) {
    IntensityMeshBlock removal;
    removal.index = idx;
    mesh.blocks.push_back(std::move(removal));
  };
  // Voxels that left the radius or were evicted from the map.
  for (const auto& [key, entry] : shown_) {
    if (current.contains(key)) continue;
    add_removal(entry.index);
    removed_since_refresh_[key] = entry.index;
  }
  if (full) {
    // Repeat earlier removals, in case one of those messages was dropped.
    for (const auto& [key, idx] : removed_since_refresh_) {
      if (!current.contains(key) && !shown_.contains(key)) add_removal(idx);
    }
    removed_since_refresh_.clear();
  }

  shown_ = std::move(current);
  return mesh;
}

bool IntensityMeshBuilder::facesPositive(const Voxel& voxel, const Point& viewpoint) {
  // The image frame's z axis in the map frame (row 2 of R_C_W) is the plane normal.
  const V3 normal = voxel.T_C_W_.linear().row(2).transpose();
  const V3 centroid = voxel.num_points_ > 0
                          ? V3(voxel.sum_ / static_cast<double>(voxel.num_points_))
                          : V3(voxel.T_C_W_.inverse().translation());
  return normal.dot(viewpoint - centroid) >= 0.0;
}

IntensityMeshBlock IntensityMeshBuilder::buildBlock(const COINBIEVRMap& map,
                                                    const Eigen::Vector3i& idx, const Voxel& voxel,
                                                    bool positive_side) const {
  IntensityMeshBlock block;
  block.index = idx;

  const Eigen::MatrixXf& intensity = voxel.intensity_smoothed_;
  const Eigen::MatrixXf& height = voxel.bump_smoothed_;
  const Eigen::MatrixXf& weights = voxel.bump_weights_;
  const Eigen::Index rows = std::min({intensity.rows(), height.rows(), weights.rows()});
  const Eigen::Index cols = std::min({intensity.cols(), height.cols(), weights.cols()});
  if (rows == 0 || cols == 0) return block;

  // Voxel image frame C: columns run along x, rows along y, pixel (i, j) sits at
  // (j * px, i * px), and z is the height above the plane.
  const Transform T_W_C = voxel.T_C_W_.inverse();
  const Eigen::Matrix3d R_W_C = T_W_C.linear();

  // The plane normal has no inherent sign; show the side the viewpoint is on.
  const V3 normal = positive_side ? V3(R_W_C.col(2)) : V3(-R_W_C.col(2));
  const Eigen::Vector3f normal_f = normal.cast<float>();

  // Back-face culling keeps triangles that run counter-clockwise when seen from
  // the front. The image axes may form a left-handed frame with the normal, so
  // pick the winding from the actual geometry rather than assuming one.
  const bool ccw = R_W_C.col(0).cross(R_W_C.col(1)).dot(normal) > 0.0;

  const double px = map.pixel_size;
  const double half = 0.5 * px;
  // Every filled pixel is drawn, including those that reach past the voxel's
  // cell (the image covers the projected voxel corners). Neighbouring voxels can
  // therefore overlap slightly at their borders, but clipping to the cell leaves
  // holes where neither voxel's image covers the boundary.

  for (Eigen::Index i = 0; i < rows; ++i) {
    for (Eigen::Index j = 0; j < cols; ++j) {
      if (weights(i, j) <= 0.0f) continue;
      const double x = static_cast<double>(j) * px;
      const double y = static_cast<double>(i) * px;
      const double z = static_cast<double>(height(i, j));  // height above the plane

      const float value = std::clamp(intensity(i, j) / 255.0f, 0.0f, 1.0f);
      const int32_t base = static_cast<int32_t>(block.vertices.size());
      for (const auto& [dx, dy] : {std::pair{-half, -half}, std::pair{half, -half},
                                   std::pair{half, half}, std::pair{-half, half}}) {
        block.vertices.push_back((T_W_C * Point(x + dx, y + dy, z)).cast<float>());
        block.normals.push_back(normal_f);
        block.intensities.push_back(value);
      }
      if (ccw) {
        block.triangles.insert(block.triangles.end(),
                               {base, base + 1, base + 2, base, base + 2, base + 3});
      } else {
        block.triangles.insert(block.triangles.end(),
                               {base, base + 2, base + 1, base, base + 3, base + 2});
      }
    }
  }
  return block;
}

}  // namespace coin_bievr
