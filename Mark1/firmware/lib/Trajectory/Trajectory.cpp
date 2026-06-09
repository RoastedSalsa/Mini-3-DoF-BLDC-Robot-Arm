#include "Trajectory.h"

Trajectory::Trajectory(const float* px, const float* py, const float* pz,
                       uint8_t count, float segment_time_s,
                       TrajectoryEasing easing)
  : px_(px), py_(py), pz_(pz), count_(count),
    segment_time_s_(segment_time_s), easing_(easing), i_(0), t0_(0) {}

void Trajectory::begin(unsigned long now_ms) {
  i_  = 0;
  t0_ = now_ms;
}

// Map normalized segment time [0,1] through the selected easing curve.
static float ease(float a, TrajectoryEasing mode) {
  if (a < 0.0f) a = 0.0f;
  else if (a > 1.0f) a = 1.0f;
  if (mode == TrajectoryEasing::SMOOTH) {
    return a * a * (3.0f - 2.0f * a);   // cubic smoothstep
  }
  return a;                             // LINEAR
}

void Trajectory::update(unsigned long now_ms, float& x, float& y, float& z) {
  if (count_ == 0) return;

  float dt = (now_ms - t0_) / 1000.0f;
  float a  = (segment_time_s_ > 0.0f) ? dt / segment_time_s_ : 1.0f;

  // Segment complete -> advance to the next waypoint pair.
  if (a >= 1.0f) {
    i_  = (i_ + 1) % count_;
    t0_ = now_ms;
    a   = 0.0f;
  }

  uint8_t i0 = i_;
  uint8_t i1 = (i_ + 1) % count_;
  float   s  = ease(a, easing_);

  x = px_[i0] + s * (px_[i1] - px_[i0]);
  y = py_[i0] + s * (py_[i1] - py_[i0]);
  z = pz_[i0] + s * (pz_[i1] - pz_[i0]);
}
