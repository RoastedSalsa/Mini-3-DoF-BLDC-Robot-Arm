#include "Telemetry.h"

Telemetry::Telemetry(Stream& out, unsigned long period_ms)
  : out_(out), period_ms_(period_ms) {}

void Telemetry::begin() {
  out_.println(F("t_ms,x,y,z,cmd1,meas1,err1,cmd2,meas2,err2,cmd3,meas3,err3"));
}

void Telemetry::update(unsigned long now_ms,
                       float x, float y, float z,
                       float cmd1, float cmd2, float cmd3,
                       float meas1, float meas2, float meas3) {
  if (!enabled_) return;
  if (now_ms - last_ms_ < period_ms_) return;
  last_ms_ = now_ms;

  out_.print(now_ms);
  out_.print(','); out_.print(x, 4);
  out_.print(','); out_.print(y, 4);
  out_.print(','); out_.print(z, 4);
  out_.print(','); out_.print(cmd1, 3);
  out_.print(','); out_.print(meas1, 3);
  out_.print(','); out_.print(cmd1 - meas1, 3);
  out_.print(','); out_.print(cmd2, 3);
  out_.print(','); out_.print(meas2, 3);
  out_.print(','); out_.print(cmd2 - meas2, 3);
  out_.print(','); out_.print(cmd3, 3);
  out_.print(','); out_.print(meas3, 3);
  out_.print(','); out_.println(cmd3 - meas3, 3);
}
