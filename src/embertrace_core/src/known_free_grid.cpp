#include "embertrace_core/known_free_grid.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace embertrace_core {

std::optional<std::pair<int, int>> KnownFreeGrid::cell(const double x, const double y) const {
  if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(origin_x) ||
      !std::isfinite(origin_y) || !std::isfinite(origin_yaw) || !std::isfinite(resolution) ||
      resolution <= 0.0 || width == 0U || height == 0U || width > 100000U || height > 100000U ||
      data.size() != static_cast<std::size_t>(width) * height) {
    return std::nullopt;
  }
  const double dx = x - origin_x;
  const double dy = y - origin_y;
  const double col = (std::cos(origin_yaw) * dx + std::sin(origin_yaw) * dy) / resolution;
  const double row = (-std::sin(origin_yaw) * dx + std::cos(origin_yaw) * dy) / resolution;
  if (!std::isfinite(col) || !std::isfinite(row) || col < 0.0 || row < 0.0 || col >= width ||
      row >= height) {
    return std::nullopt;
  }
  return std::pair<int, int>{static_cast<int>(col), static_cast<int>(row)};
}

std::pair<double, double> KnownFreeGrid::centre(const int col, const int row) const {
  const double local_x = (col + 0.5) * resolution;
  const double local_y = (row + 0.5) * resolution;
  return {origin_x + std::cos(origin_yaw) * local_x - std::sin(origin_yaw) * local_y,
          origin_y + std::sin(origin_yaw) * local_x + std::cos(origin_yaw) * local_y};
}

bool KnownFreeGrid::is_free(const double x, const double y, const double clearance_m,
                            const int max_occupancy) const {
  const auto centre_cell = cell(x, y);
  if (!centre_cell || !std::isfinite(clearance_m) || clearance_m < 0.0 ||
      clearance_m / resolution >
          std::hypot(static_cast<double>(width), static_cast<double>(height)) ||
      max_occupancy < 0 || max_occupancy > 99) {
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
      if (row < 0 || col < 0 || row >= static_cast<int>(height) || col >= static_cast<int>(width)) {
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
    const double x, const double y, const double max_displacement_m, const double clearance_m,
    const int max_occupancy, const std::optional<std::pair<double, double>> start,
    const double minimum_travel_m) const {
  if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(max_displacement_m) ||
      !std::isfinite(clearance_m) || !std::isfinite(origin_x) || !std::isfinite(origin_y) ||
      !std::isfinite(origin_yaw) || !std::isfinite(resolution) || resolution <= 0.0 ||
      width == 0U || height == 0U || max_displacement_m < 0.0 || clearance_m < 0 ||
      max_occupancy < 0 || max_occupancy > 99 || width > 100000U || height > 100000U ||
      data.size() != static_cast<std::size_t>(width) * height ||
      !std::isfinite(minimum_travel_m) || minimum_travel_m < 0 ||
      (start && (!std::isfinite(start->first) || !std::isfinite(start->second))) ||
      (!start && minimum_travel_m > 0)) {
    return std::nullopt;
  }
  const auto progresses = [&](double candidate_x, double candidate_y) {
    return !start || std::hypot(candidate_x-start->first, candidate_y-start->second) >= minimum_travel_m;
  };
  if (progresses(x, y) && is_free(x, y, clearance_m, max_occupancy)) {
    return std::pair<double, double>{x, y};
  }
  // Bound the search in grid coordinates; scanning the entire SLAM map for a
  // short goal would otherwise be costly on every sensor callback.
  const double dx = x - origin_x;
  const double dy = y - origin_y;
  const double local_x = std::cos(origin_yaw) * dx + std::sin(origin_yaw) * dy;
  const double local_y = -std::sin(origin_yaw) * dx + std::cos(origin_yaw) * dy;
  if (!std::isfinite(local_x) || !std::isfinite(local_y)) {
    return std::nullopt;
  }
  const auto bound = [](double value, std::uint32_t size) {
    return static_cast<int>(std::clamp(value, 0., static_cast<double>(size) - 1));
  };
  const int min_col = bound(std::floor((local_x - max_displacement_m) / resolution) - 1, width);
  const int max_col = bound(std::ceil((local_x + max_displacement_m) / resolution) + 1, width);
  const int min_row = bound(std::floor((local_y - max_displacement_m) / resolution) - 1, height);
  const int max_row = bound(std::ceil((local_y + max_displacement_m) / resolution) + 1, height);
  std::optional<std::pair<double, double>> best;
  double best_distance = std::numeric_limits<double>::infinity();
  for (int row = min_row; row <= max_row; ++row) {
    for (int col = min_col; col <= max_col; ++col) {
      const auto candidate = centre(col, row);
      const double distance = std::hypot(candidate.first - x, candidate.second - y);
      if (distance > max_displacement_m || distance >= best_distance ||
          !progresses(candidate.first, candidate.second)) {
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
