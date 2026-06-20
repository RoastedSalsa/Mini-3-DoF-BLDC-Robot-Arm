#include "Telemetry.h"

Telemetry::Telemetry(Stream& out, unsigned long period_ms)
  : out_(out), period_ms_(period_ms) {}

bool Telemetry::add(const char* name, const float* value) {
  if (count_ >= MAX_CHANNELS) return false;
  ch_[count_++] = {name, value};
  return true;
}

void Telemetry::update(unsigned long now_ms) {
  if (!enabled_) return;
  if (now_ms - last_ms_ < period_ms_) return;
  last_ms_ = now_ms;

  // One self-describing JSON object per line, e.g.
  //   {"t":12873,"cmd1":1.2000,"meas1":1.1834}
  out_.print(F("{\"t\":"));
  out_.print(now_ms);
  for (size_t i = 0; i < count_; ++i) {
    out_.print(F(",\""));
    out_.print(ch_[i].name);
    out_.print(F("\":"));
    out_.print(*ch_[i].value, decimals_);
  }
  out_.println('}');
}
