#ifndef COIN_BIEVR_INTENSITY_MESH_H_
#define COIN_BIEVR_INTENSITY_MESH_H_

#include <cstdint>
#include <utility>
#include <vector>

#include "coin_bievr/coin_bievr_map.h"
#include "coin_bievr/common.h"
#include "unordered_dense/unordered_dense.h"

namespace coin_bievr {

// One voxel's intensity image as a triangle mesh. A block without vertices tells
// the viewer to remove that voxel.
struct IntensityMeshBlock {
  Eigen::Vector3i index = Eigen::Vector3i::Zero();  // voxel grid index
  std::vector<Eigen::Vector3f> vertices;            // map frame
  std::vector<Eigen::Vector3f> normals;             // one per vertex
  std::vector<float> intensities;                   // one per vertex, in [0, 1]
  std::vector<int32_t> triangles;                   // vertex indices, 3 per triangle
};

// Mesh of the voxel intensity images, split into one block per voxel so a viewer
// can update voxels individually. Mirrors the layout of nvblox_msgs/Mesh.
struct IntensityMesh {
  double block_size = 0.5;  // voxel side length [m]
  std::vector<IntensityMeshBlock> blocks;
};

// Turns the intensity images of the voxels around a viewpoint into an
// incremental mesh. Each call returns only what changed since the previous one:
// voxels that are new or were updated, plus empty blocks for voxels that left
// the radius or were evicted from the map. Every `full_refresh_every`-th call
// resends every voxel in the radius, plus the removals of all voxels that left
// since the previous full refresh, so a viewer that connects late or missed a
// message catches up. It deliberately never asks the viewer to clear: the ROS1
// nvblox RViz plugin forgets its objects on clear without destroying them,
// which leaves stale geometry and crashes RViz once a block is sent again.
//
// Each observed pixel becomes a quad of two triangles, colored by its intensity.
// The visible side of every voxel always follows the current viewpoint: each
// update re-checks the side of all voxels in the radius and resends those whose
// side changed, even if their images did not. The triangles are wound so their
// front faces point toward the viewpoint: a
// viewer that culls back faces (like the nvblox RViz plugin) then shows the
// surfaces from the side the robot sees them and lets you look through them
// from behind.
class IntensityMeshBuilder {
 public:
  struct Config {
    double radius = 10.0;         // voxels whose center is within this distance [m]
    int full_refresh_every = 10;  // resend everything every n-th update, 0 disables
  };

  explicit IntensityMeshBuilder(const Config& config) : config_(config) {}

  IntensityMesh update(const COINBIEVRMap& map, const Point& viewpoint);

 private:
  // Whether the viewpoint lies on the positive side of the voxel plane (along the
  // image frame's z axis).
  static bool facesPositive(const Voxel& voxel, const Point& viewpoint);

  IntensityMeshBlock buildBlock(const COINBIEVRMap& map, const Eigen::Vector3i& idx,
                                const Voxel& voxel, bool positive_side) const;

  struct Shown {
    Eigen::Vector3i index;
    uint32_t version;    // voxel version the mesh was built from
    bool positive_side;  // side it was built to face
  };

  Config config_;
  // Voxels currently shown by the viewer, by hash.
  ankerl::unordered_dense::map<size_t, Shown> shown_;
  // Voxels removed since the last full refresh, resent then in case a removal got lost.
  ankerl::unordered_dense::map<size_t, Eigen::Vector3i> removed_since_refresh_;
  size_t update_count_ = 0;
};

}  // namespace coin_bievr

#endif  // COIN_BIEVR_INTENSITY_MESH_H_
