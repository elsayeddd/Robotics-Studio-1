#pragma once
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <stdexcept>

namespace embertrace_core {
/** Timing/persistence for an image stream; shared by RGB and IR adapters/tests. */
class ImageEvidence {
 public:
  ImageEvidence(int frames, double timeout) : frames_(frames), timeout_(timeout) {
    if (frames < 1 || !std::isfinite(timeout) || timeout <= 0) {
      throw std::invalid_argument("Image persistence/timeout must be positive");
    }
  }
  void tick(double now, double wall) {
    if (!std::isfinite(now) || !std::isfinite(wall) || now < 0 || wall < 0) {
      seen_ = false;
      count_ = 0;
      return;
    }
    if (last_clock_ >= 0 && now < last_clock_) {
      last_stamp_ = -1;
      count_ = 0;
      seen_ = false;
    }
    last_clock_ = now;
    if (seen_ && (now - last_stamp_ > timeout_ || wall - receipt_ > timeout_)) {
      seen_ = false;
      count_ = 0;
    }
  }
  bool observe(double stamp, bool layout_valid, bool positive, double now, double wall) {
    tick(now, wall);
    if (!std::isfinite(now) || !std::isfinite(wall) || now < 0 || wall < 0 || !layout_valid ||
        !std::isfinite(stamp) || stamp < 0 || stamp <= last_stamp_ || now - stamp > timeout_ ||
        stamp - now > .1) {
      seen_ = false;
      count_ = 0;
      return false;
    }
    last_stamp_ = stamp;
    receipt_ = wall;
    seen_ = true;
    if (!positive) {
      count_ = 0;
    } else if (count_ < frames_) {
      ++count_;
    }
    return true;
  }
  bool positive() const {
    return seen_ && count_ >= frames_;
  }
  bool available() const {
    return seen_;
  }

 private:
  int frames_, count_{0};
  double timeout_, last_stamp_{-1}, last_clock_{-1}, receipt_{-1};
  bool seen_{false};
};
}  // namespace embertrace_core
