#pragma once
#include <Arduino.h>

// Generic, scalable serial telemetry for the arm.
//
// Register named float signals ONCE (each backed by a live variable), then call
// update(now) every loop. When the period elapses it emits one newline-delimited
// JSON object containing every registered signal plus a "t" timestamp [ms]:
//
//   {"t":12873,"cmd1":1.2000,"meas1":1.1834,"err1":0.0166, ...}
//
// Design goals (see docs/plotjuggler.md):
//   * Scalable  — adding a signal is a single add() call. No format strings, no
//                 fixed column list, no changes to the host bridge or PlotJuggler
//                 (new JSON keys are discovered automatically downstream).
//   * Decoupled — the firmware is transport- AND tool-agnostic: it only prints
//                 JSON to a Stream&. A host-side ROS2 bridge (Mark1/ros2) turns
//                 each key into a topic; PlotJuggler subscribes to those. Nothing
//                 here knows ROS2 or PlotJuggler exists, so any of those layers
//                 can be swapped without touching control code.
//
// Zero heap use: channels live in a fixed-size table, names/values are stored by
// pointer (not copied), so update() just walks the table and prints.
class Telemetry {
public:
  explicit Telemetry(Stream& out, unsigned long period_ms = 20);

  // Register a signal backed by a live variable. `name` must outlive the
  // Telemetry object (a string literal is ideal — it is stored, not copied).
  // `value` is read (never written) at emit time. Returns false if the channel
  // table is full (raise MAX_CHANNELS if you hit this).
  bool add(const char* name, const float* value);

  void setEnabled(bool e)            { enabled_ = e; }      // mute without unwiring
  void setPeriod(unsigned long ms)   { period_ms_ = ms; }   // emit interval
  void setPrecision(uint8_t d)       { decimals_ = d; }     // float decimals

  // Emit one JSON line if the period has elapsed. Call every loop with millis().
  void update(unsigned long now_ms);

  size_t channelCount() const { return count_; }

private:
  static constexpr size_t MAX_CHANNELS = 32;
  struct Channel { const char* name; const float* value; };

  Stream&       out_;
  Channel       ch_[MAX_CHANNELS];
  size_t        count_ = 0;
  unsigned long period_ms_;
  unsigned long last_ms_ = 0;
  uint8_t       decimals_ = 4;
  bool          enabled_  = true;
};
