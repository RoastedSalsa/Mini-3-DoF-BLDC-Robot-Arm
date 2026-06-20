# Mini_Ranka ROS2 workspace

Host-side ROS2 packages for the arm. Currently one package:

- **`mini_ranka_bridge`** — bidirectional serial bridge: republishes the
  firmware's JSON telemetry as `std_msgs/Float64` topics for PlotJuggler, and
  forwards `std_msgs/String` commands from `/<ns>/cmd` to the firmware's SimpleFOC
  Commander (non-JSON replies are echoed on `/<ns>/log`). Ships a `tune` console
  for interactive live PID tuning.

This is a normal colcon workspace (`src/` here, `build/ install/ log/` are
git-ignored). It is decoupled from the firmware by a plain
[JSON-over-serial contract](../../docs/plotjuggler.md) — the firmware doesn't
know ROS2 exists, and the bridge doesn't know the arm's schema.

## Build

```bash
cd Mark1/ros2
colcon build
source install/setup.bash
```

## Run

```bash
# defaults: port=/dev/ttyACM0  baud=921600  namespace=mini_ranka
ros2 launch mini_ranka_bridge bridge.launch.py

# override anything:
ros2 launch mini_ranka_bridge bridge.launch.py port:=/dev/ttyACM1
```

Find the port with `ls -l /dev/serial/by-id` (a `by-id` path survives replugging
better than `ttyACM0`). The `baud` must match `TELEMETRY_BAUD` in
`Mark1/firmware/include/config.h`.

## See it

```bash
ros2 topic list                       # /mini_ranka/cmd1, /meas1, /err1, ...
ros2 topic echo /mini_ranka/meas1     # sanity-check one signal
```

Then open PlotJuggler -> **Streaming -> ROS2 Topic Subscriber**, pick the
`/mini_ranka/*` topics, and drag them onto plots. Log with
`ros2 bag record -a`. Full walkthrough: [docs/plotjuggler.md](../../docs/plotjuggler.md).

## Tune (live PID)

The bridge is bidirectional, so you can drive the `pid_tuner` firmware's
SimpleFOC Commander over ROS2 while watching the response in PlotJuggler:

```bash
# interactive console: type commands (MAP12, MVP0.3, L-0.15 0.15, T, MMG0 ...)
ros2 run mini_ranka_bridge tune

# ...or one-shot:
ros2 topic pub --once /mini_ranka/cmd std_msgs/msg/String "{data: 'MAP12'}"

# read Commander replies / banners:
ros2 topic echo /mini_ranka/log
```

Don't also open `pio device monitor` on the same port — the bridge owns it.
Procedure: [docs/tuning-guide.md](../../docs/tuning-guide.md).
