#pragma once
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>
namespace embertrace_core
{
// Returns no value for a malformed or unsupported image, rather than a false negative.
std::optional<double> flame_colour_score(const std::vector<std::uint8_t> & bytes,
  std::uint32_t width, std::uint32_t height, std::uint32_t step,
  const std::string & encoding, std::size_t stride = 8, double target_fraction = 0.025,
  std::size_t minimum_cluster = 4);
}
