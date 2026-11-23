#include "coin_bievr/preprocess.h"

#include <algorithm>
#include <cmath>
#include <numeric>

#include <Eigen/Eigenvalues>

#include "unordered_dense/unordered_dense.h"

namespace coin_bievr {

namespace {

struct VoxelDownsampleEntry {
  size_t hash;
  size_t idx;
  double dist;
};

struct VoxelHashIdx {
  size_t hash;
  size_t idx;
};

struct VoxelScore {
  double score;
  size_t hash;
  size_t idx;
};

}  // namespace

void voxelDownsample(const Pointcloud& points_raw, Pointcloud& points_down, double voxel_size,
                     std::vector<size_t>* out_indices) {
  std::vector<VoxelDownsampleEntry> voxel_entries(points_raw.size());

  tbb::parallel_for(
      tbb::blocked_range<size_t>(0, points_raw.size()), [&](const tbb::blocked_range<size_t>& r) {
        for (size_t idx = r.begin(); idx != r.end(); ++idx) {
          const Eigen::Vector3i voxel = (points_raw[idx] / voxel_size).array().floor().cast<int>();
          Eigen::Vector3d voxel_center =
              (voxel.cast<double>() + Eigen::Vector3d::Constant(0.5)) * voxel_size;
          double dist = (points_raw[idx] - voxel_center).squaredNorm();
          voxel_entries[idx] = {hashIndexVoxel(voxel.matrix()), idx, dist};
        }
      });

  // Sort by voxel hash, then by distance to voxel center (closest first).
  tbb::parallel_sort(voxel_entries.begin(), voxel_entries.end(),
                     [](const VoxelDownsampleEntry& a, const VoxelDownsampleEntry& b) {
                       return std::tie(a.hash, a.dist) < std::tie(b.hash, b.dist);
                     });

  std::vector<size_t> selected_indices;
  selected_indices.reserve(points_raw.size());

  size_t curr_hash = std::numeric_limits<size_t>::max();
  for (const auto& entry : voxel_entries) {
    if (entry.hash != curr_hash) {
      selected_indices.push_back(entry.idx);
      curr_hash = entry.hash;
    }
  }

  points_down.resize(selected_indices.size());
  tbb::parallel_for(tbb::blocked_range<size_t>(0, selected_indices.size()),
                    [&](const tbb::blocked_range<size_t>& r) {
                      for (size_t i = r.begin(); i != r.end(); ++i) {
                        points_down[i] = points_raw[selected_indices[i]];
                      }
                    });

  if (out_indices) *out_indices = std::move(selected_indices);
}

void sampleInformed(const COINBIEVRMap& map, const Transform& T_W_L, const Pointcloud& points_raw,
                    Pointcloud& points_coarse, Pointcloud& points_fine, double voxel_size,
                    size_t n_samples, std::vector<size_t>* out_coarse_indices,
                    std::vector<size_t>* out_fine_indices) {
  if (points_raw.size() <= n_samples) {
    points_coarse.resize(points_raw.size());
    points_coarse.data().topRows(3) = points_raw.data().topRows(3);
    if (out_coarse_indices) {
      out_coarse_indices->resize(points_raw.size());
      std::iota(out_coarse_indices->begin(), out_coarse_indices->end(), 0);
    }
    if (out_fine_indices) out_fine_indices->clear();
    return;
  }

  // Lookup hash for each point based on its transformed position from the registration prior.
  std::vector<VoxelHashIdx> voxel_entries(points_raw.size());
  tbb::parallel_for(tbb::blocked_range<size_t>(0, points_raw.size()),
                    [&](const tbb::blocked_range<size_t>& r) {
                      for (size_t idx = r.begin(); idx != r.end(); ++idx) {
                        Point p_w = T_W_L.linear() * points_raw[idx] + T_W_L.translation();
                        voxel_entries[idx] = {map.hashIndex(p_w), idx};
                      }
                    });

  tbb::parallel_sort(voxel_entries.begin(), voxel_entries.end(),
                     [](const VoxelHashIdx& a, const VoxelHashIdx& b) {
                       // Add tie to ensure deterministic order for points in the same voxel, even
                       // if they have the same hash
                       return std::tie(a.hash, a.idx) < std::tie(b.hash, b.idx);
                     });

  // Extract unique hashes of observed voxels
  std::vector<VoxelHashIdx> unique_voxels;
  unique_voxels.reserve(points_raw.size());
  std::vector<bool> voxel_has_extras;
  voxel_has_extras.reserve(points_raw.size());

  for (size_t i = 0; i < voxel_entries.size(); ++i) {
    if (i == 0 || voxel_entries[i].hash != voxel_entries[i - 1].hash) {
      unique_voxels.push_back(voxel_entries[i]);
      voxel_has_extras.push_back(false);
    } else {
      voxel_has_extras.back() = true;
    }
  }

  std::vector<VoxelScore> voxel_scores(unique_voxels.size());
  tbb::parallel_for(tbb::blocked_range<size_t>(0, unique_voxels.size()),
                    [&](const tbb::blocked_range<size_t>& r) {
                      for (size_t idx = r.begin(); idx != r.end(); ++idx) {
                        const auto& entry = unique_voxels[idx];
                        auto voxel = map.getVoxel(entry.hash);
                        double score =
                            (!voxel || !voxel_has_extras[idx]) ? 0.0 : voxel->mean_img_dist_;
                        voxel_scores[idx] = {score, entry.hash, entry.idx};
                      }
                    });

  // Sort by score, descending.
  tbb::parallel_sort(voxel_scores.begin(), voxel_scores.end(),
                     [](const VoxelScore& a, const VoxelScore& b) { return a.score > b.score; });

  ankerl::unordered_dense::set<size_t> informed_voxels;
  size_t n_select = std::min(n_samples, voxel_scores.size());
  informed_voxels.reserve(n_select);

  for (size_t idx = 0; idx < n_select; ++idx) {
    informed_voxels.insert(voxel_scores[idx].hash);
  }

  // Keep all points in informed voxels
  std::vector<size_t> informed_indices;
  informed_indices.reserve(points_raw.size());
  size_t curr_hash = std::numeric_limits<size_t>::max();
  bool curr_informed = false;
  for (const auto& entry : voxel_entries) {
    if (entry.hash != curr_hash) {
      curr_hash = entry.hash;
      curr_informed = informed_voxels.contains(entry.hash);
    }
    if (curr_informed) {
      informed_indices.push_back(entry.idx);
    }
  }

  points_fine.resize(informed_indices.size());
  tbb::parallel_for(tbb::blocked_range<size_t>(0, points_fine.size()),
                    [&](const tbb::blocked_range<size_t>& r) {
                      for (size_t idx = r.begin(); idx != r.end(); ++idx) {
                        points_fine[idx] = points_raw[informed_indices[idx]];
                      }
                    });

  // Keep one point per coarse voxel for the rest
  size_t n_informed = informed_voxels.size();
  size_t n_coarse = voxel_scores.size() - n_informed;
  points_coarse.resize(n_coarse);
  std::vector<size_t> coarse_indices;
  if (out_coarse_indices) coarse_indices.resize(n_coarse);
  tbb::parallel_for(tbb::blocked_range<size_t>(n_informed, voxel_scores.size()),
                    [&](const tbb::blocked_range<size_t>& r) {
                      for (size_t idx = r.begin(); idx != r.end(); ++idx) {
                        points_coarse[idx - n_informed] = points_raw[voxel_scores[idx].idx];
                        if (out_coarse_indices) {
                          coarse_indices[idx - n_informed] = voxel_scores[idx].idx;
                        }
                      }
                    });

  if (out_fine_indices) *out_fine_indices = std::move(informed_indices);
  if (out_coarse_indices) *out_coarse_indices = std::move(coarse_indices);
}

void sampleTopIntensityVoxels(const COINBIEVRMap& map, const Transform& T_W_L, const Pointcloud& points,
                              const Intensities& intensities, const PreprocessConfig& config,
                              std::vector<size_t>& out_indices, DegeneracyInfo* degeneracy) {
  if (degeneracy) degeneracy->valid = false;
  out_indices.clear();
  if (points.empty()) return;

  const double max_range_sq = config.intensity_max_range * config.intensity_max_range;

  std::vector<VoxelHashIdx> entries(points.size());
  std::vector<char> usable(points.size(), 0);
  tbb::parallel_for(
      tbb::blocked_range<size_t>(0, points.size()), [&](const tbb::blocked_range<size_t>& r) {
        for (size_t idx = r.begin(); idx != r.end(); ++idx) {
          const Point p_w = T_W_L.linear() * points[idx] + T_W_L.translation();
          entries[idx] = {map.hashIndex(p_w), idx};
          // A negative intensity means the projector had no pixel for this point.
          usable[idx] = intensities(idx) >= 0.0 &&
                        (max_range_sq <= 0.0 || points[idx].squaredNorm() <= max_range_sq);
        }
      });
  tbb::parallel_sort(entries.begin(), entries.end(),
                     [](const VoxelHashIdx& a, const VoxelHashIdx& b) {
                       return std::tie(a.hash, a.idx) < std::tie(b.hash, b.idx);
                     });

  // Entries are sorted by hash, so each voxel owns a contiguous run.
  struct VoxelRun {
    const Voxel* voxel;
    size_t begin;
    size_t end;
  };
  std::vector<VoxelRun> runs;
  runs.reserve(entries.size());

  // H = sum(n n^T) over every observed voxel this scan touches. Directions that no
  // plane normal supports are the ones geometry cannot resolve. It deliberately
  // spans all touched voxels rather than only the eligible ones, so the range
  // limit below cannot bias which direction comes out as degenerate.
  M3 H = M3::Zero();
  for (size_t begin = 0; begin < entries.size();) {
    size_t end = begin + 1;
    while (end < entries.size() && entries[end].hash == entries[begin].hash) ++end;

    const Voxel* voxel = map.getVoxel(entries[begin].hash);
    if (voxel && voxel->observed_) {
      const V3 normal = voxel->T_O_W_.linear().row(2);
      H += normal * normal.transpose();

      for (size_t i = begin; i < end; ++i) {
        if (usable[entries[i].idx]) {
          runs.push_back({voxel, begin, end});
          break;
        }
      }
    }
    begin = end;
  }

  const Eigen::SelfAdjointEigenSolver<M3> solver(H);
  // Eigenvalues come out ascending. When the two smallest are of the same order
  // the degeneracy spans a plane rather than a line, so score both directions.
  // They are scored separately and summed rather than combined into one vector:
  // eigenvector signs are arbitrary, so e0 + e1 and e0 - e1 are equally valid
  // and would rank voxels differently.
  const int n_degenerate = solver.eigenvalues()(1) < 10.0 * solver.eigenvalues()(0) ? 2 : 1;
  if (degeneracy) {
    degeneracy->valid = true;
    degeneracy->origin = T_W_L.translation();
    degeneracy->eigenvalues = solver.eigenvalues();
    degeneracy->eigenvectors = solver.eigenvectors();
    degeneracy->n_degenerate = n_degenerate;
  }

  const double cos_min_incidence = std::sin(config.intensity_min_incidence_deg * M_PI / 180.0);
  const V3 sensor_W = T_W_L.translation();

  std::vector<std::pair<double, size_t>> scores;  // (score, index into runs)
  scores.reserve(runs.size());
  for (size_t u = 0; u < runs.size(); ++u) {
    const Voxel* voxel = runs[u].voxel;
    if (cos_min_incidence > 0.0) {
      // Surfaces seen edge-on smear several map pixels into one measurement, so
      // their photometric residual is unreliable.
      const V3 centroid = voxel->sum_ / static_cast<double>(voxel->num_points_);
      const V3 to_voxel = (centroid - sensor_W).normalized();
      const V3 normal = voxel->T_O_W_.linear().row(2);
      if (std::abs(to_voxel.dot(normal)) < cos_min_incidence) continue;
    }
    // a^T S a is the texture along the in-plane part of each degenerate direction.
    // The out-of-plane part moves along the normal, which does not shift the
    // voxel image, so the photometric term cannot observe it.
    double score = 0.0;
    for (int k = 0; k < n_degenerate; ++k) {
      const Eigen::Vector2d a =
          (voxel->T_C_W_.linear() * solver.eigenvectors().col(k)).head<2>();
      score += a.dot(voxel->int_struct_ * a);
    }
    if (score > 0.0) scores.emplace_back(score, u);
  }

  const size_t n_select = std::min(config.n_intensity_voxels, scores.size());
  std::partial_sort(
      scores.begin(), scores.begin() + n_select, scores.end(),
      [](const std::pair<double, size_t>& a, const std::pair<double, size_t>& b) {
        return a.first > b.first;
      });
  scores.resize(n_select);
  // Emit in voxel order rather than score order, so the registration cloud keeps
  // the spatial locality the per-point map lookups benefit from.
  std::sort(scores.begin(), scores.end(),
            [](const std::pair<double, size_t>& a, const std::pair<double, size_t>& b) {
              return a.second < b.second;
            });

  for (const auto& scored : scores) {
    const VoxelRun& run = runs[scored.second];
    for (size_t i = run.begin; i < run.end; ++i) {
      const size_t idx = entries[i].idx;
      if (usable[idx]) out_indices.push_back(idx);
    }
  }
}

}  // namespace coin_bievr
