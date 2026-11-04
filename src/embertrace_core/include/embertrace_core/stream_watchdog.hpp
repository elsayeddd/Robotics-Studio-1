#pragma once
#include <cmath>
#include <stdexcept>
namespace embertrace_core {
/** Acquisition ordering plus monotonic receipt age for one input stream. */
class StreamWatchdog {
 public:
  explicit StreamWatchdog(double timeout = 1) : timeout_(timeout) {
    if (!std::isfinite(timeout) || timeout <= 0) {
      throw std::invalid_argument("Invalid stream timeout");
    }
  }
  bool observe(double stamp, double now, double wall) {
    tick(now);
    if (!std::isfinite(now) || now < 0 || !std::isfinite(wall) || wall < 0) {
      seen_ = false;
      return false;
    }
    if (!std::isfinite(stamp) || stamp < 0 || !std::isfinite(wall) || now - stamp > timeout_ ||
        stamp - now > .1 || stamp <= stamp_) {
      return false;
    }
    stamp_ = stamp;
    receipt_ = wall;
    seen_ = true;
    return true;
  }
  void tick(double now) {
    if (!std::isfinite(now) || now < 0) {
      seen_ = false;
      return;
    }
    if (last_clock_ >= 0 && now < last_clock_) {
      stamp_ = -1;
      seen_ = false;
    }
    last_clock_ = now;
  }
  void invalidate() {
    seen_ = false;
  }
  bool fresh(double now, double wall) const {
    return seen_ && now >= stamp_ - .1 && now - stamp_ <= timeout_ && wall >= receipt_ &&
           wall - receipt_ <= timeout_;
  }

 private:
  double timeout_, stamp_{-1}, receipt_{-1}, last_clock_{-1};
  bool seen_{false};
};
}  // namespace embertrace_core
