# Mini 3-DoF BLDC Robot Arm

A compact **3-degree-of-freedom robot arm** driven by affordable **BLDC gimbal motors** with field-oriented control, magnetic absolute encoders, and a belt-driven second joint. Built as a **testbed for control and trajectory-generation methods** — a small, low-cost platform for prototyping algorithms before scaling them up to a future 6-DoF arm.

<p align="center">
  <img src="docs/demo.gif" alt="Mini robot arm in motion" width="500"/>
</p>

---

## Why this project

Building a full 6-DoF arm directly is expensive and slow to iterate on. A 3-DoF arm captures most of the interesting control problems — coupled dynamics, trajectory planning, joint-space vs task-space control — at a fraction of the cost and assembly time. Lessons learned here transfer directly to the larger arm later.

The motor choice is also deliberate. Most hobby arms use servos or steppers, both of which fight against smooth motion:

- **Servos** ship with integrated gearboxes — that means **backlash**, limited torque control, and noticeable jitter at low speeds.
- **Steppers** move in discrete steps; even with microstepping, vibrations remain and torque drops sharply at speed.
- **BLDC motors with FOC** give continuous, smooth torque control, no backlash from gearing, and high responsiveness — at the cost of needing a more sophisticated closed-loop controller.

The result is closer to industrial-quality motion on a hobbyist budget.

---

## Hardware

| Subsystem | Component | Notes |
|---|---|---|
| Joint 1 (base) | iPower **GM3506** BLDC | Larger motor for base rotation load |
| Joint 2 (shoulder) | iPower **GM3506** BLDC | Belt-driven for reduced inertia at the joint |
| Joint 3 (elbow) | **GBM2408** BLDC | Smaller motor for the lighter outer link |
| Drivers | 3× **SimpleFOC Mini** | Compact FOC-capable BLDC drivers |
| Encoders | 3× **AS5600** magnetic absolute encoders | 12-bit, I²C, on-axis with diametric magnets |
| Microcontroller | **STM32 Nucleo G474RE** | Plenty of complementary PWM channels for 3-motor FOC on a single board |
| Transmission | **Belt drive** on J2 | Keeps the J2 actuator close to the base, reducing moving inertia |
| Frame | Custom **3D-printed PLA** parts | Designed in Onshape |
| Power | 12 V bench supply |
| Toolchain | **PlatformIO** (C++) | |

---

## Control

- **Library:** [SimpleFOC](https://github.com/simplefoc/Arduino-FOC) for field-oriented control of each joint.
- **Per-joint loop:** cascaded position + velocity PID, closed on the AS5600 encoder via I²C.
- **Single-MCU architecture:** the STM32 G474RE has enough PWM channels to drive all three motors directly, avoiding the dual-MCU workaround used in some earlier projects.
- **Forward / inverse kinematics:** analytical solution is done on paper, in MATLAB and also implemented into the C++ robot arm code.
- **Trajectory generation:** basic square trajectory is used to test the kinematics.
- **Digital twin:** the CAD model in RViz, mirroring the arm's measured joint angles
  over ROS2 — or driven by a host-side mock so it runs with no hardware attached.
  See [`docs/digital-twin.md`](docs/digital-twin.md).

MATLAB scripts in `kinematics/` are used for offline kinematic analysis and trajectory prototyping before porting to the embedded C++ code.

---

## Project status

🟡 **Active development — currently disassembled for upgrades.**

A first version of the arm was built and is shown in the demo video. It has since been **disassembled to install a belt adjustment upgrade on J2** and to **recalibrate all three motors** under a new PlatformIO-based firmware setup. The hardware is on the bench; reassembly and re-tuning are the next milestones.

---

## Roadmap

- [x] First-version assembly and basic motion demo
- [x] Belt upgrade on J2 for reduced backlash and smoother transmission
- [x] Migrate firmware to PlatformIO with the STM32 G474RE
- [ ] Recalibrate all three motors under the new setup
- [x] Implement analytical inverse kinematics
- [ ] Add trajectory generation (trapezoidal / S-curve point-to-point)
- [ ] Test joint-space vs task-space control strategies
- [ ] Port lessons learned to a future 6-DoF arm

---

## Repository structure

```
Mini_Ranka/
├── docs/                     # Wiring, tuning guide, progress notes, decisions
├── archive/                  # Original Arduino .ino sketches (pre-PlatformIO)
└── Mark1/
    ├── firmware/             # PlatformIO project (STM32 G474RE, SimpleFOC)
    │   ├── include/
    │   │   └── config.h      # All pins, gains, calibration, limits, waypoints
    │   ├── lib/
    │   │   ├── ArmKinematics/ # Analytical inverse kinematics
    │   │   ├── Trajectory/    # Continuous Cartesian waypoint generator
    │   │   └── Telemetry/     # Periodic CSV monitoring over serial
    │   ├── src/
    │   │   ├── main.cpp       # Main firmware: 3-joint FOC + IK + trajectory
    │   │   ├── tools/
    │   │   │   └── pid_tuner.cpp   # Live PID tuning + step-response over serial
    │   │   └── diagnostics/   # Bring-up sketches (blink, as5600, openloop, …)
    │   └── platformio.ini     # One build env per src target
    ├── ros2/                  # Host-side ROS2 workspace (pixi + RoboStack)
    │   └── src/
    │       ├── full_assembly/          # Raw Onshape CAD export (meshes + URDF)
    │       ├── mini_ranka_description/ # The robot model the twin drives
    │       └── mini_ranka_bridge/      # Telemetry, joint states, mock arm
    ├── analysis/             # MATLAB dynamics + Simulink/Simscape models
    ├── Simulations/          # Exported simulation archives
    └── hardware/             # CAD
```

Each `src/` target has its own PlatformIO environment, e.g.:

```bash
pio run -e main -t upload          # main firmware
pio run -e pid_tuner -t upload     # live PID tuning tool
pio run -e position -t upload      # single/multi-joint position diagnostic
```

---

## Author

**Augustas Gerardas Pugžlys**
BSc Mechanical Engineering (Cum Laude), TU Eindhoven
[GitHub](https://github.com/RoastedSalsa) · [LinkedIn](https://www.linkedin.com/in/augustas-gerardas-pugzlys/) <!-- TODO: add LinkedIn URL -->

---

## License

Released under the MIT License. See [`LICENSE`](LICENSE) for details.
