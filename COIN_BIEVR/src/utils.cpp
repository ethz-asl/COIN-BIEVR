#include "coin_bievr/utils.h"

#include <algorithm>
#include <cmath>
#include <ctime>
#include <iomanip>
#include <iostream>
#include <sstream>

namespace coin_bievr {

// Width of the dashboard table (characters between the border bars).
constexpr int kDashWidth = 55;

// Horizontal border of the dashboard table.
std::string dashRule() { return "|" + std::string(kDashWidth, '-') + "|"; }

// A content row left-aligned and padded to the fixed table width.
std::string dashRow(const std::string& text) {
  std::ostringstream os;
  os << "| " << std::left << std::setw(kDashWidth - 1) << text << "|";
  return os.str();
}

// Fixed-width string for a double, matching the COIN-LIO dashboard formatting.
std::string dashFmt(double value) {
  std::ostringstream os;
  os << std::fixed << std::setprecision(3) << value;
  return os.str();
}
// Seconds rendered as milliseconds with a single decimal.
std::string dashMs(double seconds) {
  std::ostringstream os;
  os << std::fixed << std::setprecision(1) << seconds * 1e3;
  return os.str();
}

void printDashboardBanner(const std::string& ascii, const std::string& message) {
  std::ostringstream out;
  out << "\033[2J\033[1;1H";  // clear screen and move cursor home
  if (!ascii.empty()) {
    out << ascii << "\n";
  }
  out << dashRule() << "\n";
  out << dashRow(message) << "\n";
  out << dashRule();
  std::cout << out.str() << std::endl;
}

// Number of terminal columns a line occupies: UTF-8 code points, skipping ANSI
// escape sequences (e.g. 24-bit color codes). Assumes single-width characters.
int visibleWidth(const std::string& line) {
  int width = 0;
  for (size_t i = 0; i < line.size(); ++i) {
    const unsigned char c = line[i];
    if (c == 0x1b) {  // ESC [ ... final byte in @..~
      if (i + 1 < line.size() && line[i + 1] == '[') {
        i += 2;
        while (i < line.size() && (line[i] < 0x40 || line[i] > 0x7e)) ++i;
      }
      continue;
    }
    if ((c & 0xC0) != 0x80) ++width;  // count lead bytes only
  }
  return width;
}

std::vector<std::string> splitAsciiFrames(const std::vector<std::string>& lines, int frame_lines) {
  std::vector<std::string> frames;
  if (lines.empty()) return frames;
  const size_t n = lines.size();
  const size_t per_frame =
      (frame_lines > 0 && n % static_cast<size_t>(frame_lines) == 0) ? frame_lines : n;
  // Center the art on the full table width (content plus the two border bars).
  int art_width = 0;
  for (const auto& line : lines) art_width = std::max(art_width, visibleWidth(line));
  const std::string pad(std::max((kDashWidth + 2 - art_width) / 2, 0), ' ');
  for (size_t start = 0; start < n; start += per_frame) {
    std::string frame;
    for (size_t i = start; i < start + per_frame; ++i) {
      if (i > start) frame += "\n";
      frame += pad + lines[i];
    }
    frames.push_back(std::move(frame));
  }
  return frames;
}

void printDashboard(DashboardState& state, uint64_t stamp_ns, const Transform& T_W_I,
                    const V3& velocity, const V3& acc_bias, const V3& gyro_bias, double comp_mean_s,
                    double comp_max_s, int n_effective_points) {
  // Accumulate the travelled distance from the previous reported position.
  const V3 position = T_W_I.translation();
  if (state.has_last_position) {
    state.trajectory_length += (position - state.last_position).norm();
  }
  state.last_position = position;
  state.has_last_position = true;

  const std::string rule = dashRule();
  const auto& row = dashRow;

  const Eigen::Vector3d euler = T_W_I.rotation().eulerAngles(0, 1, 2) * (180.0 / M_PI);

  // Header line: sensor time and elapsed time since the first dashboard frame.
  const double current_time_s = nsToS(stamp_ns);
  if (state.first_time_s < 0.0) state.first_time_s = current_time_s;
  const double elapsed_s = current_time_s - state.first_time_s;
  const std::time_t current_time = static_cast<std::time_t>(current_time_s);
  char time_buf[32] = "";
  std::strftime(time_buf, sizeof(time_buf), "%Y-%m-%d %H:%M:%S", std::localtime(&current_time));

  std::ostringstream out;
  out << "\033[2J\033[1;1H";  // clear screen and move cursor home
  if (!state.ascii_frames.empty()) {
    // Advance the animation by one frame per dashboard print.
    out << state.ascii_frames[state.frame_index % state.ascii_frames.size()] << "\n";
    ++state.frame_index;
  }
  out << rule << "\n";
  out << row("COIN-BIEVR  " + std::string(time_buf) + "  Elapsed: " + dashFmt(elapsed_s) + " s")
      << "\n";
  out << rule << "\n";
  out << row("Position (x,y,z)    [m]   : " + dashFmt(position.x()) + " " + dashFmt(position.y()) +
             " " + dashFmt(position.z()))
      << "\n";
  out << row("Orientation (r,p,y) [deg] : " + dashFmt(euler.x()) + " " + dashFmt(euler.y()) + " " +
             dashFmt(euler.z()))
      << "\n";
  out << row("Velocity (x,y,z)    [m/s] : " + dashFmt(velocity.x()) + " " + dashFmt(velocity.y()) +
             " " + dashFmt(velocity.z()))
      << "\n";
  out << row("Trajectory Length   [m]   : " + dashFmt(state.trajectory_length)) << "\n";
  out << rule << "\n";
  out << row("Acc. Bias (x,y,z) [m/s2]  : " + dashFmt(acc_bias.x()) + " " + dashFmt(acc_bias.y()) +
             " " + dashFmt(acc_bias.z()))
      << "\n";
  out << row("Gyro Bias (x,y,z) [rad/s] : " + dashFmt(gyro_bias.x()) + " " +
             dashFmt(gyro_bias.y()) + " " + dashFmt(gyro_bias.z()))
      << "\n";
  out << rule << "\n";
  out << row("Effective Points      [#] : " + std::to_string(n_effective_points)) << "\n";
  out << row("Computation Time     [ms] : Avg: " + dashMs(comp_mean_s) +
             " Max: " + dashMs(comp_max_s))
      << "\n";
  out << rule;

  std::cout << out.str() << std::endl;
}

Eigen::MatrixXf buildGaussianKernel(int radius, float sigma) {
  int size = 2 * radius + 1;
  Eigen::MatrixXf kernel(size, size);
  float sum = 0.0f;

  for (int y = -radius; y <= radius; ++y) {
    for (int x = -radius; x <= radius; ++x) {
      float value = std::exp(-(x * x + y * y) / (2 * sigma * sigma));
      kernel(y + radius, x + radius) = value;
      sum += value;
    }
  }
  kernel /= sum;  // normalize
  return kernel;
}

float computeSigmaFromRadius(int radius) {
  int ksize = 2 * radius + 1;
  return 0.3f * ((ksize - 1) * 0.5f - 1) + 0.8f;
}

// Offsets to the 7-connected neighbors of a voxel: the voxel itself plus ±voxel_size along each
// axis. Order is {0}, +x, -x, +y, -y, +z, -z.
std::vector<Point> getNeighborOffsets(double voxel_size) {
  std::vector<Point> offsets;
  offsets.reserve(7);
  offsets.push_back(Point::Zero());
  for (int axis = 0; axis < 3; ++axis) {
    for (double sign : {1.0, -1.0}) {
      Point offset = Point::Zero();
      offset(axis) = sign * voxel_size;
      offsets.push_back(offset);
    }
  }
  return offsets;
}

}  // namespace coin_bievr
