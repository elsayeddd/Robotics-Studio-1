#pragma once
#include <cstdint>
#include <cstddef>
#include <optional>
#include <string>
#include <vector>
namespace embertrace_core
{
struct ThermalResult {double peak_c{}, hot_fraction{}; std::size_t hot_pixels{}; bool hotspot{};};
/** Radiometric image decoding, not RGB brightness. Integer pixels are Kelvin/resolution.
 * 32FC1 input is explicitly Celsius. Padded rows and either byte order are supported.
 */
std::optional<ThermalResult> thermal_hotspot(const std::vector<std::uint8_t> & bytes,
  std::uint32_t width, std::uint32_t height, std::uint32_t step,
  const std::string & encoding, bool big_endian, double kelvin_per_unit = .01,
  double threshold_c = 50, std::size_t min_cluster = 4, double min_fraction = .0005);
}  // namespace embertrace_core
