#pragma once
#include <Arduino.h>

// Lightweight periodic serial telemetry for the 3-DoF arm.
//
// CSV columns:
//   t_ms,x,y,z,cmd1,meas1,err1,cmd2,meas2,err2,cmd3,meas3,err3
class Telemetry {
public:
  explicit Telemetry(Stream& out, unsigned long period_ms = 200);

  void begin();                                   // print the CSV header once
  void setEnabled(bool e) { enabled_ = e; }       // mute without removing calls
  void setPeriod(unsigned long ms) { period_ms_ = ms; }

  // Emit one sample if the period has elapsed. cmd* = commanded joint angles
  // (motor frame, rad), meas* = measured sensor angles (rad), and the active
  // Cartesian target (x, y, z).
  void update(unsigned long now_ms,
              float x, float y, float z,
              float cmd1, float cmd2, float cmd3,
              float meas1, float meas2, float meas3);

private:
  Stream&       out_;
  unsigned long period_ms_;
  unsigned long last_ms_ = 0;
  bool          enabled_ = true;
};
