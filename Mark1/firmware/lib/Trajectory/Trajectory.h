#pragma once
#include <Arduino.h>

// Continuous, multi-waypoint Cartesian trajectory generator for the 3-DoF arm.
//
// Cycles endlessly through a list of (x, y, z) waypoints, interpolating position
// over a fixed time per segment. It is time-based (uses millis()), so the output
// is independent of the control-loop rate.

// Easing applied to the normalized segment time [0, 1] before interpolation.
//   LINEAR — constant velocity within a segment.
//   SMOOTH — cubic smoothstep: zero velocity at each waypoint.
enum class TrajectoryEasing { LINEAR, SMOOTH };

class Trajectory {
public:
  Trajectory(const float* px, const float* py, const float* pz,
             uint8_t count, float segment_time_s,
             TrajectoryEasing easing = TrajectoryEasing::LINEAR);

  // (Re)start from the first segment at time `now_ms`.
  void begin(unsigned long now_ms);

  // Advance to time `now_ms` and write the current Cartesian setpoint into
  // x, y, z. Call every loop.
  void update(unsigned long now_ms, float& x, float& y, float& z);

  // Index of the segment currently being traversed [0 .. count-1].
  uint8_t segment() const { return i_; }

private:
  const float* px_;
  const float* py_;
  const float* pz_;
  uint8_t          count_;
  float            segment_time_s_;
  TrajectoryEasing easing_;
  uint8_t          i_;
  unsigned long    t0_;
};
