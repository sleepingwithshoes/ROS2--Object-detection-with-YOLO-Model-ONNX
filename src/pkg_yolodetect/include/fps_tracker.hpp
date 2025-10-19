#pragma once

#include <cstdint>
#include <vector>
#include <tuple>
#include <utility>
#include <deque>

class FPSTracker
{
public:
  FPSTracker(size_t queue_size = 30);

  // Update with new duration in milliseconds and return current FPS
  double update(int64_t duration_ms);

  // Get average FPS
  double getAverage() const;

  // Get current FPS
  double getCurrent() const;

private:
  std::deque<double> fps_queue_;
  size_t max_queue_size_;
  double fps_avg_;
  double fps_current_;
};
