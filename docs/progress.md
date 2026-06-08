# Mini_Ranka — Mark 1 Progress & Next Steps

**Project:** 3-DOF Robotic Arm (BLDC + SimpleFOC + NUCLEO-G474RE)
**Last Updated:** March 24, 2026

---

## What Has Been Done

### 1. Hardware Bring-Up (Firmware — `Mark1/Code/`)

A step-by-step validation sequence has been completed for the hardware stack:

**Blink Test (`Blink_474`)**
Basic GPIO verification on the NUCLEO-G474RE. Confirms the board is flashed correctly and PA5 toggles as expected.

**Encoder Readout (`AS5600_474`)**
I2C communication with an AS5600 magnetic encoder is working. Raw angle is read at 12-bit resolution and converted to degrees over a custom `TwoWire` bus on PB9/PB8.

**Open-Loop Motor Spin (`OpenLoop-474`)**
The SimpleFOC library is integrated and a GM3506 BLDC motor spins in open-loop velocity mode. Driver pinout (PA8, PA9, PA10, EN=PA6) is confirmed working.

**Pole-Pair Finder (`Pole-Finder_474`)**
A utility sketch that sweeps a configurable number of electrical revolutions so you can count mechanical turns and compute pole pairs. This confirmed the motor parameters used in closed-loop code.

**Single-Motor Closed-Loop Position Control (`Position_Control_474`)**
Motor 2 runs full closed-loop FOC with:
- AS5600 sensor on I2C bus (PB9/PB8)
- Driver on PA0, PA1, PC0, PB0
- Tuned PID: `P_velocity=0.3`, `I=0.1`, `P_angle=20`
- Hardcoded zero electric angle: `3.529690` rad
- Commander interface for serial angle commands

**Full 3-Motor System with IK (`Full_Start_474`)**
The most advanced firmware. All three axes are operational:
- Three separate I2C buses (PC7/PC6, PB9/PB8, PC9/PC8) each hosting an AS5600
- Three BLDC motors with individual SimpleFOC drivers
- Closed-loop angle control on all three joints
- Hardcoded zero electric angles and sensor directions per motor
- **Inverse Kinematics** implemented analytically for a 3-DOF arm:
  - Link lengths: `l1 = 48.8 mm`, `l2 = 200 mm`, `l3 = 200 mm`
  - IK solves `(x, y, z)` → `(θ1, θ2, θ3)` in real time
  - Motor angle offsets (zeroing) applied at startup based on sensor quadrant
- **4-Point Linear Trajectory** running continuously between Cartesian waypoints with configurable segment time (2 s per segment)
- Serial input to override the target `(x, y, z)` on the fly

---

### 2. Simulation & Dynamics (`Mark1/Simulations/`, `Mark1/Calculations/`)

Three Simulink/Simscape models have been built (all include exported STEP geometry):

| Simulation | Description |
|---|---|
| `Matlab_Sim_BasicIK` | Simulink model validating the inverse kinematics without gravity |
| `Matlab_Sim_Gravity` | First gravity-aware simulation |
| `Matlab_SIm2_Gravity` | Refined gravity simulation (updated version) |

A MATLAB Live Script (`Dynamics.mlx`) contains the symbolic/numeric dynamics calculations for the arm.

---

## What Still Needs to Be Done

### High Priority

**1. Joint Zeroing / Homing Routine**
The current zeroing logic uses hardcoded magic numbers (e.g. `zeroed1 = 6.487` or `0.172` depending on sensor quadrant). This is fragile. A proper homing routine should:
- Command the arm to a known mechanical hard-stop or reference pose
- Record and store encoder offsets
- Make the process repeatable without reflashing

**2. Gravity Compensation in Firmware**
The Simulink gravity simulations exist but the compensation torque has not been fed back into the FOC loop. The next step is to use the arm's known dynamics (`Dynamics.mlx`) to compute a feedforward gravity torque and add it to each motor's voltage command based on joint angles.

**3. Motor 2 Gear Ratio Verification**
The code multiplies motor 2's angle command by 3 (`th2 = zeroed2 + theta2 * 3`), implying a 3:1 gearbox. This factor should be formally confirmed with measurements and documented as a calibration constant rather than a magic number.

**4. Trajectory Smoothing**
The current 4-point trajectory uses linear interpolation, which produces velocity discontinuities at waypoints (jerky motion). Replacing this with:
- Cubic or quintic polynomial splines, or
- Minimum-jerk trajectory profiles
will make motion significantly smoother and protect the mechanical system.

### Medium Priority

**5. PID Tuning Validation**
The PID gains are set empirically. A systematic tuning pass (e.g. step response tests, Ziegler-Nichols, or MATLAB's Control System Toolbox) would improve tracking accuracy and stability across the workspace.

**6. Workspace Limits & Safety**
There are no joint angle limits enforced in firmware. The IK will produce unreachable or self-collision configurations if given bad `(x, y, z)` targets. Adding workspace bounds checks (both in Cartesian and joint space) and graceful error handling is important before serious testing.

**7. Validate Firmware Against Simulation**
Run the same trajectories in both the Simulink model and on the real hardware, compare joint angle logs, and iterate on the model or hardware until they match. This closes the sim-to-real gap.

**8. Serial / Communication Interface**
The current serial input is a simple `parseFloat()` loop. A more robust protocol (e.g. structured packets, checksum, acknowledgment) would make the arm easier to command from a PC or higher-level controller.

### Lower Priority / Future Work

**9. End-Effector / Gripper**
No gripper or tool is implemented yet. Depending on the application, this could be a servo-driven gripper, electromagnet, or custom attachment.

**10. Mark 2 Design Iteration**
Based on lessons from Mark 1 (zeroing issues, gear ratio, trajectory performance), plan mechanical and electrical improvements for a Mark 2 version.

**11. ROS / Higher-Level Control**
For more complex tasks, integrating with ROS 2 (via micro-ROS on the NUCLEO or a serial bridge) would enable path planning, perception, and task programming.

---

## Summary Table

| Area | Status |
|---|---|
| Board bring-up (blink, GPIO) | ✅ Done |
| Encoder readout (AS5600, I2C) | ✅ Done |
| Open-loop motor control | ✅ Done |
| Pole-pair identification | ✅ Done |
| Single-motor closed-loop FOC | ✅ Done |
| 3-motor closed-loop FOC | ✅ Done |
| Analytical inverse kinematics | ✅ Done |
| Basic linear trajectory | ✅ Done |
| MATLAB dynamics model | ✅ Done |
| Simulink simulations (IK + gravity) | ✅ Done |
| Proper homing / zeroing routine | ⬜ Not done |
| Gravity compensation in firmware | ⬜ Not done |
| Smooth trajectory (splines) | ⬜ Not done |
| Joint limits / workspace safety | ⬜ Not done |
| Sim-to-real validation | ⬜ Not done |
| Gripper / end-effector | ⬜ Not done |
| Robust serial protocol | ⬜ Not done |
