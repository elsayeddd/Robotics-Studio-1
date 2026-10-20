#include "embertrace_core/thermal_detector.hpp"
#include <algorithm>
#include <cmath>
#include <cstring>
#include <queue>
namespace embertrace_core
{
std::optional<ThermalResult> thermal_hotspot(const std::vector<std::uint8_t> & bytes,
  std::uint32_t width, std::uint32_t height, std::uint32_t step,
  const std::string & encoding, bool big, double resolution, double threshold,
  std::size_t min_cluster, double min_fraction)
{
  const bool integer = encoding == "mono16" || encoding == "16UC1";
  const bool floating = encoding == "32FC1";
  const std::size_t pixel_bytes = integer ? 2 : 4;
  if ((!integer && !floating) || !width || !height || !step || !min_cluster ||
    !std::isfinite(resolution) || resolution <= 0 || !std::isfinite(threshold) ||
    !std::isfinite(min_fraction) || min_fraction < 0 || min_fraction > 1 ||
    width > step / pixel_bytes || height > bytes.size() / step)
  {return std::nullopt;}
  const std::size_t count = static_cast<std::size_t>(width) * height;
  std::vector<double> temperatures(count);
  double maximum = -80;
  for (std::size_t y = 0; y < height; ++y) {
    for (std::size_t x = 0; x < width; ++x) {
      const auto i = y * step + x * pixel_bytes;
      double celsius;
      if (integer) {
        const std::uint16_t n = big ? (bytes[i] << 8) | bytes[i+1] : bytes[i] | (bytes[i+1] << 8);
        celsius = n * resolution - 273.15;
      } else {
        std::uint32_t bits = 0;
        for (std::size_t b = 0; b < 4; ++b) {bits |= std::uint32_t(bytes[i+b]) << (8 * (big ? 3-b : b));}
        float value; std::memcpy(&value, &bits, sizeof(value)); celsius = value;
      }
      // Zero Kelvin and NaN are invalid measurements, not a cold scene.
      if (!std::isfinite(celsius) || celsius < -80 || celsius > 1000) {return std::nullopt;}
      temperatures[y*width+x] = celsius; maximum = std::max(maximum, celsius);
    }
  }
  std::vector<bool> visited(count, false);
  std::size_t largest = 0; double cluster_peak = -80;
  for (std::size_t i = 0; i < count; ++i) {
    if (visited[i] || temperatures[i] < threshold) {continue;}
    std::queue<std::size_t> queue; queue.push(i); visited[i] = true;
    std::size_t n = 0; double peak = -80;
    while (!queue.empty()) {
      const auto p = queue.front(); queue.pop(); ++n; peak = std::max(peak, temperatures[p]);
      const auto x = p % width, y = p / width;
      const auto add = [&](std::size_t v) {
          if (!visited[v] && temperatures[v] >= threshold) {visited[v] = true; queue.push(v);}};
      if (x) {add(p-1);} if (x+1 < width) {add(p+1);}
      if (y) {add(p-width);} if (y+1 < height) {add(p+width);}
    }
    if (n > largest) {largest = n; cluster_peak = peak;}
  }
  const double fraction = static_cast<double>(largest) / count;
  const bool hotspot = largest >= min_cluster && fraction >= min_fraction;
  return ThermalResult{hotspot ? cluster_peak : maximum, fraction, largest, hotspot};
}
}  // namespace embertrace_core
