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

`src/tools/pid_tuner.cpp` brings up **one** joint and exposes every gain over
serial via the SimpleFOC Commander — adjust gains while the motor runs, no
reflashing.

```bash
# 1. Pick the joint: edit `#define TUNE_JOINT 1|2|3` in src/tools/pid_tuner.cpp
# 2. Flash and open the serial monitor
pio run -e pid_tuner -t upload
pio device monitor -b 115200
```

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
3. **Watch the CSV** (below) for overshoot, oscillation and settling time as you
   turn each knob.
4. **Copy the final values back into `config.h`** so `full.cpp` uses them.

---

## 3. Monitoring

### `pid_tuner` stream

While tuning, the tool streams a CSV line at ~20 Hz, ideal for the Arduino
**Serial Plotter** or a logging script:

```
t_ms,target,measured,error
```

### `full.cpp` telemetry

The main firmware uses [`lib/Telemetry`](../Mark1/firmware/lib/Telemetry) to emit
one CSV sample every `TELEMETRY_PERIOD_MS` (default 200 ms):

```
t_ms,x,y,z,cmd1,meas1,err1,cmd2,meas2,err2,cmd3,meas3,err3
```

where `x,y,z` is the active Cartesian target, `cmd*` are the commanded joint
angles (motor frame) and `meas*` the measured encoder angles. `err* = cmd-meas`
is the live tracking error per joint — the quantity to minimize.

To capture a run for offline analysis:

```bash
pio device monitor -b 115200 | tee run.csv
```

---

## 4. Notes

- **Calibration vs. gains.** `M*_ZERO_ELEC_ANGLE` are *mounting-specific*
  electrical offsets, not tuning knobs — re-measure them (polefinder + the
  `initFOC` report) after any re-assembly. `diagnostics/position.cpp` was last
  run with different alternates; keep whichever set matches the current build.
- **Velocity limit.** `M*_VEL_LIMIT` is high (100 rad/s) from the position-control
  bring-up. The trajectory is slow and the joint limits bound travel, but lower
  it if a large manual step command snaps too aggressively.
