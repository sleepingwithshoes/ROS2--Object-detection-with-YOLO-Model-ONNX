#include "fps_tracker.hpp"
#include <cmath>
#include <limits>
#include <algorithm>
#include <numeric>

// FPSTracker implementation
FPSTracker::FPSTracker(size_t queue_size)
    : max_queue_size_(queue_size), fps_avg_(0.0), fps_current_(0.0) {}

double FPSTracker::update(int64_t duration_ms)
{
  // Calculate current FPS
  fps_current_ = (duration_ms > 0) ? 1000.0 / duration_ms : 0.0;

  // Update queue
  fps_queue_.push_back(fps_current_);
  if (fps_queue_.size() > max_queue_size_)
  {
    fps_queue_.pop_front();
  }

  // Calculate average
  if (!fps_queue_.empty())
  {
    fps_avg_ = std::accumulate(fps_queue_.begin(), fps_queue_.end(), 0.0) / fps_queue_.size();
  }

  return fps_current_;
}

double FPSTracker::getAverage() const
{
  return fps_avg_;
}

double FPSTracker::getCurrent() const
{
  return fps_current_;
}
