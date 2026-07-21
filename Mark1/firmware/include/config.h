#pragma once
//
// config.h — centralized hardware/control constants for the Mini 3-DoF BLDC arm.
//
// NOTE: sensor_direction (Direction::CW for all three joints in full.cpp) is left out of here to account for AS5600.cpp
//
#include <Arduino.h>   // pin names are here

// ======================= I2C / AS5600 magnetic sensors =======================
// Each sensor sits on its own hardware I2C bus (SDA, SCL).
constexpr auto S1_SDA_PIN = PB7;   // sensor 1 — I2C1 HW
constexpr auto S1_SCL_PIN = PA15;
constexpr auto S2_SDA_PIN = PC9;   // sensor 2 — I2C3 HW
constexpr auto S2_SCL_PIN = PC8;
constexpr auto S3_SDA_PIN = PC7;   // sensor 3 — I2C4 HW
constexpr auto S3_SCL_PIN = PC6;
constexpr uint32_t I2C_CLOCK_HZ = 400000;

// ============================= BLDC driver pins ==============================
// 3-PWM phase pins (A, B, C) + enable, per joint.
constexpr auto M1_PH_A = PA8;   // driver1 — TIM1
constexpr auto M1_PH_B = PA9;
constexpr auto M1_PH_C = PA10;
constexpr auto M1_EN   = PC5;

constexpr auto M2_PH_A = PA0;   // driver2 — TIM2
constexpr auto M2_PH_B = PA1;
constexpr auto M2_PH_C = PB10;
constexpr auto M2_EN   = PB12;

constexpr auto M3_PH_A = PA6;   // driver3 — TIM3
constexpr auto M3_PH_B = PB5;
constexpr auto M3_PH_C = PB0;
constexpr auto M3_EN   = PB13;

// ============================== Motor pole pairs =============================
// NOTE: joint 3 differs from joints 1 & 2.
constexpr int M1_POLE_PAIRS = 11; //G3506 Ipower
constexpr int M2_POLE_PAIRS = 11; //G3506 Ipower  
constexpr int M3_POLE_PAIRS = 7;    //GBM2408 random Aliexpress

// ================================ Power / limits =============================
constexpr float SUPPLY_VOLTAGE = 12.0;   // battery / PSU voltage
constexpr float VOLTAGE_LIMIT  = 12.0;   // FOC voltage limit

// ============================ Per-motor control gains ========================
// Cascaded velocity + angle PID, per joint. Can be tunned live using the `pid_tuner` tool

constexpr float M1_VEL_P       = 0.3;
constexpr float M1_VEL_I       = 0.1;
constexpr float M1_ANGLE_P     = 10;
constexpr float M1_ANGLE_I     = 1.0;
constexpr float M1_ANGLE_D     = 0.05;
constexpr float M1_VEL_LIMIT   = 100;
constexpr float M1_LPF_TF      = 0.01;
constexpr float M1_OUTPUT_RAMP = 1000;

constexpr float M2_VEL_P       = 0.3;
constexpr float M2_VEL_I       = 0.1;
constexpr float M2_ANGLE_P     = 10;
constexpr float M2_ANGLE_I     = 1.0;
constexpr float M2_ANGLE_D     = 0.05;
constexpr float M2_VEL_LIMIT   = 100;
constexpr float M2_LPF_TF      = 0.01;
constexpr float M2_OUTPUT_RAMP = 1000;

constexpr float M3_VEL_P       = 0.4;
constexpr float M3_VEL_I       = 0.05;
constexpr float M3_ANGLE_P     = 50;
constexpr float M3_ANGLE_I     = 0;
constexpr float M3_ANGLE_D     = 0.05;
constexpr float M3_VEL_LIMIT   = 20;
constexpr float M3_LPF_TF      = 0.01;
constexpr float M3_OUTPUT_RAMP = 1000;

// ===================== Per-motor calibration (electrical) ====================
// REDO each time motors/sensors are reassembled.
constexpr float M1_ZERO_ELEC_ANGLE = 1.2732;
constexpr float M2_ZERO_ELEC_ANGLE = 0.61210;
constexpr float M3_ZERO_ELEC_ANGLE = 2.2273;

// ============================== Arm kinematics ===============================
// Link lengths [m].
constexpr float L1 = 0.0843;
constexpr float L2 = 0.2;
constexpr float L3 = 0.2;

// ========================= Per-joint frame mapping ===========================
// Joint -> sensor/motor frame relationship. Fixed hardware facts:
//   DIR  = encoder count direction vs the IK joint frame (+1 same, -1 opposite).
//          J1 reads opposite the IK frame (atan2(y,x) is CCW; encoder counts CW).
//   GEAR = motor-side reduction (J2 belt 3:1; J1/J3 direct).
constexpr float JOINT_DIR[3]  = {-1.0f, 1.0f, 1.0f};
constexpr float JOINT_GEAR[3] = { 1.0f, 3.0f, 1.0f};


// ====================== Per-joint software travel limits =====================
// Applied to the FINAL motor target (sensor frame, radians) before move(). 
// Keeps IK / serial commands from driving a joint into a mechanical hard-stop.
constexpr float J1_MIN = -2.0,  J1_MAX = 2.0;
constexpr float J2_MIN = -2.0, J2_MAX = 2.0;
constexpr float J3_MIN = -2.0,  J3_MAX = 2.0;

// ============================== Homing =======================================
// Default sensor-frame offsets [rad], applied to the motor target. Used at
// startup so the arm runs WITHOUT homing. home() ('O') captures fresh ones live
// and prints them — paste the printed values back here to update the defaults.
constexpr float HOME_OFFSET_1 = 3.7675;   // 0 = kinematic zero already matches sensor
constexpr float HOME_OFFSET_2 = -6.7613;
constexpr float HOME_OFFSET_3 = 3.4452;

// Cartesian home pose [m] — the physical reference home() captures against.
constexpr float HOME_X = 0.050;
constexpr float HOME_Y = 0.050;
constexpr float HOME_Z = 0.03;

// ====================== Dynamics feedforward (optional) ======================
// Model-based feedforward applied as a q-axis VOLTAGE on top of the angle-control
// loop (voltage-mode torque control). See lib/Dynamics for the derivation of
//     tau = M(q) qdd + C(q, qd) qd + G(q)
//
// DYN_FF_DEFAULT_ON is the master enable at boot; live-toggle with 'F'.
constexpr bool  DYN_FF_DEFAULT_ON = false;
constexpr float GRAV_G = 9.81f;                 // gravitational accel [m/s^2]

// --- Which terms of the model to evaluate ------------------------------------
// Boot-time defaults; each is independently toggled at runtime with 'D' (see
// main.cpp), so these are just the starting point. Bring the model up one term
// at a time: gravity alone is well-conditioned and safe, while inertia and the
// velocity-product terms depend on differentiated setpoints and on the inertia
// parameters below being roughly right.
constexpr bool DYN_ENABLE_GRAVITY     = true;
constexpr bool DYN_ENABLE_INERTIA     = false;
constexpr bool DYN_ENABLE_CENTRIFUGAL = false;
constexpr bool DYN_ENABLE_CORIOLIS    = false;

// --- Link dynamic parameters (from CAD) --------------------------------------
// Link 2 = upper arm (length L2); link 3 = forearm + any payload (length L3).
// Derived by rigid-body lumping of the Onshape export
// (ros2/src/full_assembly/urdf/full_assembly.urdf); see description/mini_ranka.urdf.
//   LINK2 = link2 + gbm2804_stator        LINK3 = gbm2804_rotor + link3
// NOTE: LINK3_COM is ~0.053 m, not the mid-link 0.1 m you would guess — the
// GBM2804 elbow rotor sits right at the joint and pulls the COM inboard.
constexpr float LINK2_MASS = 0.099447f;  // mass of link 2 [kg]
constexpr float LINK3_MASS = 0.080946f;  // mass of link 3 (+ payload) [kg]
constexpr float LINK2_COM  = 0.102579f;  // COM dist. from joint-2 axis along L2 [m]
constexpr float LINK3_COM  = 0.052760f;  // COM dist. from joint-3 axis along L3 [m]

// Rotational inertia about each link's OWN centre of mass [kg·m^2]. Only used by
// the DYN_INERTIA term. Link 1 is the base yaw stage, about the vertical axis;
// links 2 and 3 are about their pitch axes. Values are the CAD inertia tensors
// (izz for the yaw stage, iyy for the pitch links) rather than the m*l^2/12
// slender-rod estimate, which underestimates link 2 by 1.45x.
// LINK1_INERTIA is the shoulder yoke (yaws but does not pitch), parallel-axis
// shifted onto the yaw axis: 1.5278e-4 + 0.16421*0.025482^2.
constexpr float LINK1_INERTIA = 2.593981e-4f;
constexpr float LINK2_INERTIA = 4.818199e-4f;
constexpr float LINK3_INERTIA = 2.791125e-4f;

// --- Setpoint differentiation -------------------------------------------------
// qd and qdd are obtained by differentiating the IK joint setpoints, so both are
// low-pass filtered (first-order, time constant [s]) to keep the step-shaped
// output of a LINEAR-eased trajectory from producing acceleration spikes. Raise
// these if the feedforward looks noisy; lower them if it lags the motion.
constexpr float DYN_VEL_LPF_TF   = 0.02f;
constexpr float DYN_ACCEL_LPF_TF = 0.05f;

// --- Torque -> voltage mapping (Vq = tau * R / Kt) ---------------------------
// Per-joint motor torque constant Kt [N·m/A] and phase resistance R [ohm].
// Set Kt <= 0 to disable feedforward on a joint (torque_to_voltage returns 0).
constexpr float M1_TORQUE_CONSTANT  = 0.0f;
constexpr float M2_TORQUE_CONSTANT  = 0.0f;
constexpr float M3_TORQUE_CONSTANT  = 0.0f;
constexpr float M1_PHASE_RESISTANCE = 0.0f;
constexpr float M2_PHASE_RESISTANCE = 0.0f;
constexpr float M3_PHASE_RESISTANCE = 0.0f;

// Overall feedforward scale (0..1) for ramp-in / detuning while testing, and a
// hard clamp [V] on the injected feedforward per motor (defense against a bad
// model). The final voltage.q is still clamped to VOLTAGE_LIMIT afterwards.
constexpr float DYN_FF_GAIN          = 1.0f;
constexpr float DYN_FF_VOLTAGE_LIMIT = 3.0f;

// =============================== Trajectory ==================================
// Continuous Cartesian waypoint loop (see lib/Trajectory). The arm cycles
// through these waypoints, interpolating position over a fixed time per segment.
constexpr float   TRAJ_PX[]            = {0.2, 0.2,  0.2,  0.2};
constexpr float   TRAJ_PY[]            = {0.1, 0.1, -0.1, -0.1};
constexpr float   TRAJ_PZ[]            = {0.15, 0.25, 0.25, 0.15};
constexpr uint8_t TRAJ_COUNT           = 4;
constexpr float   TRAJ_SEGMENT_TIME_S  = 2.0;   // seconds per segment

// =============================== Telemetry ===================================
// Periodic serial telemetry for PlotJuggler (see lib/Telemetry + docs/plotjuggler.md).
// One JSON line per interval is streamed over USB serial and bridged to ROS2.
//
// Baud is high so a full ~150-byte JSON line drains in ~1.6 ms and never stalls
// the FOC loop. Keep the host ROS2 bridge (Mark1/ros2) at the SAME baud.
constexpr unsigned long TELEMETRY_BAUD       = 921600;
constexpr unsigned long TELEMETRY_PERIOD_MS  = 20;     // 50 Hz
