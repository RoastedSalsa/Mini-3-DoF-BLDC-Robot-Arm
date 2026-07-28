# Digital twin (RViz)

The arm's model in RViz, driven either by the real hardware or by a host-side
mock. Built on the same telemetry contract as [plotjuggler.md](plotjuggler.md) —
the firmware gained three telemetry channels and nothing else.

```
STM32 firmware            host (ROS2)                        RViz
────────────────          ──────────────────────────         ──────────────
lib/Telemetry    ── USB ─► telemetry_bridge                  RobotModel
q1/q2/q3 in the    serial   JSON key -> /mini_ranka/<key>    (+ Tool trace)
IK joint frame     JSON     Float64                             ▲
                              │                                 │ TF
                              ▼                                 │
                           joint_state_bridge ──► /joint_states ─┴─ robot_state_publisher
                           (Float64 -> JointState)                  + mini_ranka_description

                           mock_arm ──────────► /joint_states
                           (no hardware: trajectory + IK on the host)
```

## Running it

```bash
cd Mark1/ros2
pixi run -e lyrical build
pixi shell -e lyrical          # or: source install/setup.bash
```

**No hardware** — replays the `config.h` trajectory through the host-side IK:

```bash
ros2 launch mini_ranka_bridge twin.launch.py            # source:=mock is the default
ros2 launch mini_ranka_bridge twin.launch.py smooth:=true   # smoothstep easing
```

**Real arm** — opens the serial port and mirrors the measured joint angles:

```bash
ros2 launch mini_ranka_bridge twin.launch.py source:=live
ros2 launch mini_ranka_bridge twin.launch.py source:=live port:=/dev/ttyACM1
```

`source:=live` still runs `telemetry_bridge`, so PlotJuggler and `ros2 bag
record` keep working exactly as before while RViz is up.

**Just the model**, with sliders instead of data:

```bash
ros2 launch mini_ranka_description display.launch.py         # jsp:=gui
ros2 launch mini_ranka_description display.launch.py jsp:=idle
```

## The model

`mini_ranka_description/urdf/mini_ranka.urdf` is the Onshape CAD export
(`full_assembly`) put onto the firmware's kinematic convention. The CAD geometry
is already right — axis-to-axis distances come out at exactly `L1`/`L2`/`L3` —
but the export disagrees with `ArmKinematics.cpp` about names, signs and zero,
so four joints were reworked:

| Export | Model | Change |
|---|---|---|
| `gm3506_revolute` | `joint1` | axis negated, `pi` yaw added to the origin |
| `bearing_revolute` | `joint2` | axis negated |
| `gbm2804_revolute` | `joint3` | limits only — signs already agreed |
| `gm3506_revolute_1` | `joint2_belt` | `mimic` of joint2 at the 3:1 belt ratio |

The `pi` yaw is the important one: the export's zero pose puts the arm along
**-X**, and `fk()` puts it along **+X**. Absorbing the half turn into joint1's
origin keeps `base_link` Z-up and X-forward.

`joint2_belt` is a mimic rather than a fourth DOF because the GM3506 pulley and
the arm are one mechanism. Nothing publishes it — `robot_state_publisher`
derives it, and the pulley visibly spins 3x the arm, like the real one.

Meshes are not copied: the URDF points at `package://full_assembly/meshes/`, so
a re-export only has to land in one place. See the header comment in the URDF
for what to re-apply after re-exporting.

### Keeping it honest

`mini_ranka_description/test/test_kinematics.py` compares the URDF against a
port of the firmware's `fk()` — tool position across the travel range, world
joint axes, link lengths, limits, and the belt ratio. A flipped axis still looks
plausible in RViz while the twin quietly mirrors the arm, which is exactly the
failure this catches.

```bash
colcon test --packages-select mini_ranka_description
colcon test-result --all --verbose
```

## The joint-state contract

`robot_state_publisher` needs one `sensor_msgs/JointState` naming
`joint1`/`joint2`/`joint3`, in the **IK joint frame**. `telemetry_bridge`
publishes one `Float64` per JSON key, which is right for plotting and wrong for
this — `joint_state_bridge` is the adapter, and the only piece that speaks both.

It takes input two ways and picks automatically:

- **`q1`/`q2`/`q3`** (preferred) — IK-frame joint angles, computed in the
  firmware and streamed like any other channel. The firmware knows its own live
  home offsets, including ones captured by `O` since boot.
- **`meas1..3`** (fallback) — raw encoder angles, converted host-side with
  `JOINT_DIR`/`JOINT_GEAR` and the `home_offsets` parameter. Works with firmware
  that predates the `q*` channels, but that parameter is a *copy* of `config.h`:
  re-home the arm and this path is silently wrong until you update it.

It starts in fallback and upgrades on the first `q*` message, so either firmware
works with no flag. If telemetry stops, it warns once and holds the last pose
rather than publishing stale data forever.

## Notes / troubleshooting

- **Nothing in RViz, no errors.** Check something is publishing `/joint_states`
  — `twin.launch.py` passes `jsp:=none` to the display launch precisely so two
  publishers can't fight over it. `ros2 topic hz /joint_states`.
- **Arm vanishes.** Usually NaN joint states from an out-of-reach IK target. The
  mock guards against this (holds the last pose and warns); the firmware's
  `ik()` does not, and will hand back NaN if asked for an unreachable point.
- **Arm mirrored or bent the wrong way.** Run the kinematics test. If it passes,
  the model is fine and the home offsets are the suspect.
- **Tool trace missing.** `/mini_ranka/tool_path` is only published by the two
  joint-state sources — with the slider GUI there is no source, so it stays
  empty. `path_length:=0` disables it.
- **`joint3` clamping warnings in mock.** Real, not a bug: see below.

## Known issue — the default trajectory grazes the J3 limit

Running the mock over the `config.h` waypoints logs:

```
IK solution (-0.236, 1.31, -2.001) exceeds the config.h travel limits; clamping
```

The closing segment of the square — `(0.2, -0.1, 0.15)` back to `(0.2, 0.1,
0.15)` — passes through `y = 0`, where the tool is closest to the shoulder and
the elbow folds hardest. Sweeping the full cycle, `joint3` bottoms out at
**-2.0331 rad (-116.5 deg)** at exactly `(0.2, 0.0, 0.15)`, which is **33 mrad
past `J3_MIN = -2.0`**.

The firmware clamps the same way (limits are applied to the final motor target),
so on hardware this shows up as a small flat spot and a tracking-error bump in
the middle of that segment rather than anything dramatic. Clearing it is a
one-line choice — loosen `J3_MIN` to about -2.05, or lift those two waypoints
from `z = 0.15` to `z = 0.16` — but worth making deliberately rather than
meeting it as a mystery on the bench.
