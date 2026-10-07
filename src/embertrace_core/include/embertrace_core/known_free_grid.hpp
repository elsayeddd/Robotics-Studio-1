#ifndef EMBERTRACE_CORE__KNOWN_FREE_GRID_HPP_
#define EMBERTRACE_CORE__KNOWN_FREE_GRID_HPP_

#include <cstdint>
#include <optional>
#include <utility>
#include <vector>

namespace embertrace_core
{

// Small ROS-independent view of an OccupancyGrid. Unknown (-1) is never free.
struct KnownFreeGrid
{
  unsigned int width{0};
  unsigned int height{0};
  double resolution{0.0};
  double origin_x{0.0};
  double origin_y{0.0};
  double origin_yaw{0.0};
  std::vector<std::int8_t> data;

  bool is_free(double x, double y, double clearance_m, int max_occupancy) const;
  std::optional<std::pair<double, double>> nearest_free(
    double x, double y, double max_displacement_m, double clearance_m,
    int max_occupancy) const;

private:
  std::optional<std::pair<int, int>> cell(double x, double y) const;
  std::pair<double, double> centre(int col, int row) const;
};

}  // namespace embertrace_core

#endif  // EMBERTRACE_CORE__KNOWN_FREE_GRID_HPP_
