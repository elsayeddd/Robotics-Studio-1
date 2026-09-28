#include "embertrace_core/known_free_grid.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace embertrace_core
{

std::optional<std::pair<int, int>> KnownFreeGrid::cell(const double x, const double y) const
{
  if (!std::isfinite(x) || !std::isfinite(y) ||
    !std::isfinite(origin_x) || !std::isfinite(origin_y) ||
    !std::isfinite(origin_yaw) || !std::isfinite(resolution) ||
    resolution <= 0.0 || width == 0U || height == 0U ||
    data.size() != static_cast<std::size_t>(width) * height)
  {
    return std::nullopt;
  }
  const double dx = x - origin_x;
  const double dy = y - origin_y;
  const double col = (std::cos(origin_yaw) * dx + std::sin(origin_yaw) * dy) / resolution;
  const double row = (-std::sin(origin_yaw) * dx + std::cos(origin_yaw) * dy) / resolution;
  if (!std::isfinite(col) || !std::isfinite(row) ||
    col < 0.0 || row < 0.0 || col >= width || row >= height)
  {
    return std::nullopt;
  }
  return std::pair<int, int>{static_cast<int>(col), static_cast<int>(row)};
}

std::pair<double, double> KnownFreeGrid::centre(const int col, const int row) const
{
  const double local_x = (col + 0.5) * resolution;
  const double local_y = (row + 0.5) * resolution;
  return {
    origin_x + std::cos(origin_yaw) * local_x - std::sin(origin_yaw) * local_y,
    origin_y + std::sin(origin_yaw) * local_x + std::cos(origin_yaw) * local_y};
}

bool KnownFreeGrid::is_free(
  const double x, const double y, const double clearance_m, const int max_occupancy) const
{
  const auto centre_cell = cell(x, y);
  if (!centre_cell || clearance_m < 0.0) {
    return false;
  }
  // Check the square around the footprint, including cells at its edge. This
  // deliberately overestimates a circular robot's footprint at diagonals.
  const int radius = static_cast<int>(std::ceil(clearance_m / resolution)) + 1;
  for (int row = centre_cell->second - radius; row <= centre_cell->second + radius; ++row) {
    for (int col = centre_cell->first - radius; col <= centre_cell->first + radius; ++col) {
      const auto [cell_x, cell_y] = centre(col, row);
      if (std::hypot(cell_x - x, cell_y - y) > clearance_m + resolution * 0.7072) {
        continue;
      }
      if (row < 0 || col < 0 || row >= static_cast<int>(height) ||
        col >= static_cast<int>(width))
      {
        return false;
      }
      const int value = data[static_cast<std::size_t>(row) * width + col];
      if (value < 0 || value > max_occupancy) {
        return false;
      }
    }
  }
  return true;
}

std::optional<std::pair<double, double>> KnownFreeGrid::nearest_free(
  const double x, const double y, const double max_displacement_m,
  const double clearance_m, const int max_occupancy) const
{
  if (!std::isfinite(x) || !std::isfinite(y) ||
    !std::isfinite(max_displacement_m) || !std::isfinite(clearance_m) ||
    !std::isfinite(origin_x) || !std::isfinite(origin_y) ||
    !std::isfinite(origin_yaw) || !std::isfinite(resolution) ||
    resolution <= 0.0 || width == 0U || height == 0U ||
    max_displacement_m < 0.0 ||
    data.size() != static_cast<std::size_t>(width) * height)
  {
    return std::nullopt;
  }
  if (is_free(x, y, clearance_m, max_occupancy)) {
    return std::pair<double, double>{x, y};
  }
  // Bound the search in grid coordinates; scanning the entire SLAM map for a
  // short goal would otherwise be costly on every sensor callback.
  const double dx = x - origin_x;
  const double dy = y - origin_y;
  const double local_x = std::cos(origin_yaw) * dx + std::sin(origin_yaw) * dy;
  const double local_y = -std::sin(origin_yaw) * dx + std::cos(origin_yaw) * dy;
  const int min_col = std::max(0, static_cast<int>(
    std::floor((local_x - max_displacement_m) / resolution)) - 1);
  const int max_col = std::min(static_cast<int>(width) - 1, static_cast<int>(
    std::ceil((local_x + max_displacement_m) / resolution)) + 1);
  const int min_row = std::max(0, static_cast<int>(
    std::floor((local_y - max_displacement_m) / resolution)) - 1);
  const int max_row = std::min(static_cast<int>(height) - 1, static_cast<int>(
    std::ceil((local_y + max_displacement_m) / resolution)) + 1);
  std::optional<std::pair<double, double>> best;
  double best_distance = std::numeric_limits<double>::infinity();
  for (int row = min_row; row <= max_row; ++row) {
    for (int col = min_col; col <= max_col; ++col) {
      const auto candidate = centre(col, row);
      const double distance = std::hypot(candidate.first - x, candidate.second - y);
      if (distance > max_displacement_m || distance >= best_distance) {
        continue;
      }
      if (is_free(candidate.first, candidate.second, clearance_m, max_occupancy)) {
        best = candidate;
        best_distance = distance;
      }
    }
  }
  return best;
}

}  // namespace embertrace_core
