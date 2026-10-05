#include "embertrace_core/visual_classifier.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
namespace embertrace_core
{
std::optional<double> flame_colour_score(const std::vector<std::uint8_t> & bytes,
  std::uint32_t width, std::uint32_t height, std::uint32_t step,
  const std::string & encoding, std::size_t stride, double target, std::size_t min_cluster)
{
  const bool rgb = encoding == "rgb8" || encoding == "rgba8";
  const bool bgr = encoding == "bgr8" || encoding == "bgra8";
  if ((!rgb && !bgr) || width == 0 || height == 0 || stride == 0 ||
    !std::isfinite(target) || target <= 0 || min_cluster == 0) {return std::nullopt;}
  const std::size_t channels = (encoding == "rgba8" || encoding == "bgra8") ? 4 : 3;
  if (step < static_cast<std::size_t>(width) * channels ||
    static_cast<std::size_t>(height) > bytes.size() / step) {return std::nullopt;}
  const std::size_t cols = (width + stride - 1) / stride;
  const std::size_t rows = (height + stride - 1) / stride;
  std::vector<bool> mask(cols * rows, false);
  for (std::size_t y = 0; y < rows; ++y) {
    for (std::size_t x = 0; x < cols; ++x) {
      const std::size_t i = y * stride * step + x * stride * channels;
      const int r = bytes[i + (rgb ? 0 : 2)], g = bytes[i + 1], b = bytes[i + (rgb ? 2 : 0)];
      mask[y * cols + x] = r > 160 && g > 35 && g < 220 && b < 115 && r > g + 30 && g > b;
    }
  }
  std::size_t largest = 0;
  for (std::size_t i = 0; i < mask.size(); ++i) {
    if (!mask[i]) {continue;}
    std::queue<std::size_t> pending;
    pending.push(i); mask[i] = false;
    std::size_t count = 0;
    while (!pending.empty()) {
      const auto n = pending.front(); pending.pop(); ++count;
      const std::size_t x = n % cols, y = n / cols;
      const auto add = [&](std::size_t v) {if (mask[v]) {mask[v] = false; pending.push(v);}};
      if (x > 0) {add(n - 1);} if (x + 1 < cols) {add(n + 1);}
      if (y > 0) {add(n - cols);} if (y + 1 < rows) {add(n + cols);}
    }
    largest = std::max(largest, count);
  }
  return largest < min_cluster ? 0.0 :
    std::clamp(static_cast<double>(largest) / (cols * rows * target), 0.0, 1.0);
}
}
