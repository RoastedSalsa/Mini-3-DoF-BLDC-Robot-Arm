# PID Tuning & Monitoring Guide

How to tune the per-joint cascaded PID controllers on the Mini 3-DoF arm and how
to read the telemetry while doing it.

All gains live in [`Mark1/firmware/include/config.h`](../Mark1/firmware/include/config.h)
(the `M*_VEL_*` / `M*_ANGLE_*` blocks). The starting values were carried over
from the position-control bring-up (`diagnostics/position.cpp`): each joint runs
a **cascaded controller** — an inner velocity PID feeding an outer angle PID.

---

## 1. The control loop

```
target angle ─▶ [ angle PID (P,I,D) ] ─▶ velocity setpoint
                                            │  (capped at velocity_limit)
             measured velocity ─▶ [ velocity PID (P,I) ] ─▶ voltage ─▶ FOC
```

| Symbol (config.h) | SimpleFOC field | Role |
|---|---|---|
| `M*_VEL_P`, `M*_VEL_I` | `PID_velocity.P/.I` | Inner velocity loop |
| `M*_OUTPUT_RAMP` | `PID_velocity.output_ramp` | Slew-rate limit on the voltage |
| `M*_ANGLE_P/I/D` | `P_angle.P/.I/.D` | Outer position loop |
| `M*_VEL_LIMIT` | `velocity_limit` | Max commanded joint speed [rad/s] |
| `M*_LPF_TF` | `LPF_velocity.Tf` | Velocity-estimate low-pass filter |

Joint software travel limits (`J*_MIN`/`J*_MAX`) are applied to the final motor
target in both `full.cpp` and the tuner, so a bad command can't drive a joint
into a hard-stop.

---

## 2. Live tuning with the `pid_tuner` tool

`src/calibration/pid_tuner.cpp` brings up **one** joint and exposes every gain
over serial via the SimpleFOC Commander — adjust gains while the motor runs, no
reflashing. It streams the **same JSON telemetry contract as `full.cpp`**
(`target`, `meas`, `err`, `vel`), so you watch the step response live in
PlotJuggler through the ROS2 bridge and send gain commands back over ROS2.

```bash
# 1. Pick the joint: edit `#define TUNE_JOINT 1|2|3` in src/calibration/pid_tuner.cpp
# 2. Flash it (streams JSON at 921600 = TELEMETRY_BAUD)
cd Mark1/firmware && pio run -e pid_tuner -t upload

# 3. Start the bridge (it owns the serial port) + an interactive command console
cd Mark1/ros2 && colcon build && source install/setup.bash
ros2 launch mini_ranka_bridge bridge.launch.py     # terminal A
ros2 run   mini_ranka_bridge tune                  # terminal B  (type commands here)

# 4. Plot: PlotJuggler -> Streaming -> ROS2 Topic Subscriber ->
#    /mini_ranka/target, /mini_ranka/meas, /mini_ranka/err, /mini_ranka/vel
```

> **One process owns the port.** The bridge holds `/dev/ttyACM0`, so don't also
> open `pio device monitor` on it — that contention corrupts both streams. Send
> commands through the `tune` console (or `ros2 topic pub --once /mini_ranka/cmd
> std_msgs/msg/String "{data: 'MAP12'}"`); read Commander replies with
> `ros2 topic echo /mini_ranka/log`.

### Serial commands

| Command | Action |
|---|---|
| `MMG0` | Print **all** current gains and limits |
| `MVP0.3` / `MVI0.1` / `MVD0` | Velocity PID P / I / D |
| `MAP12` / `MAI1.0` / `MAD0.05` | Angle PID P / I / D |
| `MLV100` / `MLU12` | Velocity limit / voltage limit |
| `G<rad>` | Set the target angle (clamped to the joint limits) |
| `T` | Toggle the **continuous step** (square wave) setpoint |
| `L<lo> <hi>` | Set the two step levels [rad] |
| `P<sec>` | Set the step half-period [s] |
| `H` | Help |

> The `M…` commands are the standard SimpleFOC Commander *motor* interface — see
> the [SimpleFOC Commander docs](https://docs.simplefoc.com/commander_interface)
> for the full letter map.

### Suggested procedure

1. **Velocity loop first.** Disable the step (`T` off), command a slow target.
   Raise `MVP` until the joint responds briskly without buzzing, then add `MVI`
   to remove steady-state error. Keep `MVD` at 0 for gimbal motors.
2. **Then the angle loop.** Turn on the continuous step (`T`, with `L`/`P` set to
   a safe swing and period). Raise `MAP` for a fast rise; add a little `MAD` to
   damp overshoot; add `MAI` only if there's residual steady-state error.
3. **Watch PlotJuggler** (`/mini_ranka/target` vs `/mini_ranka/meas`, plus
   `/err` and `/vel`) for overshoot, oscillation and settling time as you turn
   each knob.
4. **Copy the final values back into `config.h`** so `full.cpp` uses them.

---

## 3. Monitoring

### `pid_tuner` stream

While tuning, the tool emits one JSON line per sample (every
`TELEMETRY_PERIOD_MS`, default 20 ms) over serial — the same `lib/Telemetry`
contract as `full.cpp`, so the existing ROS2 bridge plots it with no changes:

```
{"t":12873,"target":3.40,"meas":3.39,"err":0.01,"vel":0.02}
```

- `target` is the commanded setpoint (post software-limit), `meas` the encoder
  angle, `err = target - meas` the tracking error, `vel` the shaft velocity.
- Plot in PlotJuggler via the bridge (`/mini_ranka/*`); log with
  `ros2 bag record -a`. Setup: [docs/plotjuggler.md](plotjuggler.md).

### `full.cpp` telemetry → PlotJuggler

The main firmware uses [`lib/Telemetry`](../Mark1/firmware/lib/Telemetry) to emit
one JSON line every `TELEMETRY_PERIOD_MS` (default 20 ms) over serial:

```
{"t":12873,"x":0.20,"y":0.0,"z":0.25,"cmd1":1.20,"meas1":1.18,"err1":0.02, ...}
```

where `x,y,z` is the active Cartesian target, `cmd*` are the commanded joint
angles (motor frame) and `meas*` the measured encoder angles. `err* = cmd-meas`
is the live tracking error per joint — the quantity to minimize.

Plot it live in **PlotJuggler** via the ROS2 bridge, and capture runs with
`ros2 bag record`. Full setup: [docs/plotjuggler.md](plotjuggler.md).

---

## 4. Notes

- **Calibration vs. gains.** `M*_ZERO_ELEC_ANGLE` are *mounting-specific*
  electrical offsets, not tuning knobs — re-measure them (polefinder + the
  `initFOC` report) after any re-assembly. `diagnostics/position.cpp` was last
  run with different alternates; keep whichever set matches the current build.
- **Velocity limit.** `M*_VEL_LIMIT` is high (100 rad/s) from the position-control
  bring-up. The trajectory is slow and the joint limits bound travel, but lower
  it if a large manual step command snaps too aggressively.
