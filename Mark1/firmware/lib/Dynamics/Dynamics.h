#pragma once
#include <Arduino.h>

// Dynamics — rigid-body inverse-dynamics feedforward for the Mini 3-DoF arm.
// ----------------------------------------------------------------------------
// Computes the joint torques predicted by the manipulator equation
//
//     tau = M(q) qdd  +  C(q, qd) qd  +  G(q)
//           \______/     \__________/    \___/
//           inertia      centrifugal +   gravity
//                        Coriolis
//
// then converts them to q-axis feedforward VOLTAGES so they can be injected into
// SimpleFOC voltage-mode torque control.
//
// How it plugs into the control loop (see main.cpp):
//   The joints stay in MotionControlType::angle (position control). Right after
//   motor.move(), we add a feedforward voltage on top of the position loop's
//   output:  motor.voltage.q += Vff (then re-clamp to VOLTAGE_LIMIT). Because the
//   torque controller is TorqueControlType::voltage, voltage.q maps ~linearly to
//   torque, so the model does the work WITHOUT the position loop having to build
//   up an error first.
//
// TERM SELECTION
//   Every term above can be enabled independently, so you can bring the model up
//   one piece at a time (gravity first, then inertia, ...) and see each one's
//   effect in isolation. Defaults come from config.h (DYN_ENABLE_*); the live
//   mask is toggled at runtime with the 'D' Commander command, which also works
//   over the ROS2 bridge:
//       ros2 topic pub --once /mini_ranka/cmd std_msgs/String "data: 'DI'"
//
// Joint-frame convention (identical to ArmKinematics / ik()):
//   q1 = base yaw   — rotation about the vertical Z axis.
//   q2 = shoulder pitch, measured from the horizontal plane (FK: z += L2*sin q2).
//   q3 = elbow pitch, RELATIVE to link 2 (FK: z += L3*sin(q2+q3)).
//   All angles/rates are IK JOINT-frame, NOT motor-frame. main.cpp maps the
//   resulting per-joint voltage back to the motor frame (JOINT_DIR / JOINT_GEAR).
//
// Model: each link is a point mass at distance LINK*_COM along the link, plus a
// rotational inertia LINK*_INERTIA about its own COM pitch axis. Link 1 (the base
// yaw stage) contributes LINK1_INERTIA about the vertical axis. Physical
// parameters all live in config.h. See Dynamics.cpp for the derivation.

// ---------------------------------------------------------------------------
// Which terms of the manipulator equation to evaluate. Combine with '|'.
// ---------------------------------------------------------------------------
enum DynamicsTerm : uint8_t {
  DYN_NONE        = 0,
  DYN_GRAVITY     = 1 << 0,   // G(q)                    — holds the arm up
  DYN_INERTIA     = 1 << 1,   // M(q) qdd                — needs qdd
  DYN_CENTRIFUGAL = 1 << 2,   // qd_i^2 terms of C qd     — needs qd
  DYN_CORIOLIS    = 1 << 3,   // qd_i*qd_j terms of C qd  — needs qd
  DYN_ALL         = DYN_GRAVITY | DYN_INERTIA | DYN_CENTRIFUGAL | DYN_CORIOLIS,
};

// Boot-time mask, assembled from the DYN_ENABLE_* flags in config.h.
uint8_t dynamics_default_terms();

// Per-joint feedforward torques in the IK joint frame. [N·m]
struct JointTorques {
  float tau1;   // joint 1 (base yaw)  — zero unless the arm is actually moving
  float tau2;   // joint 2 (shoulder)
  float tau3;   // joint 3 (elbow)
};

// Evaluate the selected terms of tau = M(q)qdd + C(q,qd)qd + G(q).
//
//   q    — joint positions     [rad]
//   qd   — joint velocities    [rad/s]   (ignored unless CENTRIFUGAL/CORIOLIS)
//   qdd  — joint accelerations [rad/s^2] (ignored unless INERTIA)
//   terms — bitwise OR of DynamicsTerm; DYN_NONE returns all zeros.
//
// Disabled terms cost nothing: their trig/products are skipped entirely.
JointTorques joint_torques(const float q[3], const float qd[3], const float qdd[3],
                           uint8_t terms);

// Gravity-only convenience wrapper — equivalent to joint_torques(q, 0, 0, DYN_GRAVITY).
JointTorques gravity_torques(float q1, float q2, float q3);

// Convert a joint torque [N·m] to the q-axis feedforward voltage [V] for
// voltage-mode torque control of the given joint (index 0..2).
//   Vq = tau * R / Kt   (stall / low-speed approximation, back-EMF ignored)
// Uses per-joint Kt (M*_TORQUE_CONSTANT) and R (M*_PHASE_RESISTANCE) from config.
// Gear reduction and sensor direction are applied by the caller (main.cpp).
float torque_to_voltage(uint8_t joint, float tau);

// Human-readable name of a single term bit, for Commander replies. "?" if the
// argument is not exactly one known bit.
const char* dynamics_term_name(uint8_t term_bit);
