# Telemetry & PlotJuggler

Live plotting for the arm, built in three decoupled layers so any one can be
swapped without touching the others:

```
STM32 firmware              host (ROS2)                  PlotJuggler
────────────────            ────────────────────         ──────────────────
lib/Telemetry      ── USB ─► mini_ranka_bridge   ── ROS2 ─► ROS2 Topic
named float          serial   (rclpy: JSON key →    topics   Subscriber
channels → JSON      JSON     /mini_ranka/<key>              (+ ros2 bag
lines                lines    Float64, auto-made)            for logging)
```

The only thing the layers share is a **contract**, below. The firmware never
references ROS2 or PlotJuggler; the bridge never hard-codes the arm's signals.

## The contract (firmware ↔ host)

The firmware emits **one newline-delimited JSON object per sample** over the
USB serial port:

```json
{"t":12873,"cmd1":1.2000,"meas1":1.1834,"err1":0.0166,"cmd2":-0.40, ...}
```

- `t` is `millis()` (device uptime, ms).
- Every other key is a numeric signal. Keys are arbitrary and self-describing.
- Non-JSON lines (boot banners, `Arm ready.`) are ignored by the bridge.

Baud is **921600** (`TELEMETRY_BAUD` in `firmware/include/config.h`); the host
side must match.

## Commands back to the firmware (host → device)

The bridge is **bidirectional**. It subscribes to `<ns>/cmd` (`std_msgs/String`)
and writes each line to the same serial port, and echoes non-JSON serial lines
(boot banners, SimpleFOC Commander replies) on `<ns>/log`. That is what lets the
`pid_tuner` firmware be tuned live over ROS2 — same JSON telemetry out, Commander
gain commands in, no second process fighting for the port:

```bash
ros2 run mini_ranka_bridge tune            # interactive command console
ros2 topic echo /mini_ranka/log            # Commander replies / banners
```

See [tuning-guide.md](tuning-guide.md) for the tuning procedure.

## Adding / removing a plotted signal — one line

Everything flows from the firmware registration. In `firmware/src/full.cpp`:

```cpp
static void registerTelemetry() {
  telemetry.add("meas1", &meas1);   // name on the wire  ->  live variable
  telemetry.add("gz",    &grav_z);  // <- add a new signal here, done.
}
```

`add()` stores a pointer to a live variable; `telemetry.update(now)` in the loop
emits all of them. A new key shows up as a new `/mini_ranka/<key>` topic in ROS2
and a new curve in PlotJuggler **with no host or PlotJuggler changes**. (The
table holds up to `MAX_CHANNELS = 32`; bump it in `lib/Telemetry/Telemetry.h` if
needed.)

Rate and precision are configurable: `TELEMETRY_PERIOD_MS` / `TELEMETRY_BAUD` in
`config.h`, or at runtime `telemetry.setPeriod(ms)` / `setPrecision(decimals)` /
`setEnabled(false)`.

## Quick start

1. **Flash** the firmware:
   ```bash
   cd Mark1/firmware && pio run -e full -t upload
   ```
2. **Build & run** the bridge:
   ```bash
   cd Mark1/ros2 && colcon build && source install/setup.bash
   ros2 launch mini_ranka_bridge bridge.launch.py port:=/dev/ttyACM0
   ```
   (`ls -l /dev/serial/by-id` to find the port.)
3. **PlotJuggler**: launch it (`ros2 run plotjuggler plotjuggler`, or the
   AppImage), then **Streaming → ROS2 Topic Subscriber → Start**, select the
   `/mini_ranka/*` topics, and drag signals onto plots.
4. **Log** a run for later: `ros2 bag record -a` (replay into PlotJuggler, or
   export CSV from PlotJuggler). This replaces the old fixed-schema CSV stream.

## Notes / troubleshooting

- **Timestamps.** PlotJuggler uses message arrival time by default, which is
  fine for live work. For device-accurate time, plot against `/mini_ranka/t`
  (set it as the X axis), or record a bag and use bag time.
- **No data?** Check the port/baud match, that the bridge logged `opened …` and
  `new signal -> …`, and `ros2 topic echo /mini_ranka/meas1`.
- **FOC loop jitter.** Serial writes are throttled to `TELEMETRY_PERIOD_MS` and
  the `full` env enlarges the TX buffer (`SERIAL_TX_BUFFER_SIZE=256`) so a line
  drains without stalling the loop. If you push the rate much higher, watch
  control quality.
- **Future:** swapping the host bridge for micro-ROS on the MCU (or an MQTT/UDP
  bridge) requires no firmware telemetry changes — only the same JSON contract,
  or a new transport behind it.
