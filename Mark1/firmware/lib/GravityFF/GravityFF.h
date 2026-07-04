#pragma once
#include <Arduino.h>

// GravityFF — gravity feedforward for the Mini 3-DoF BLDC arm.
// ----------------------------------------------------------------------------
// Computes per-joint gravity-compensation *torques* from the current joint
// configuration, then converts them to q-axis feedforward *voltages* so they
// can be injected into SimpleFOC voltage-mode torque control.
//
// How it plugs into the control loop (see main.cpp):
//   The joints stay in MotionControlType::angle (position control). Right after
//   motor.move(), we add a feedforward voltage on top of the position loop's
//   output:  motor.voltage.q += Vff (then re-clamp to VOLTAGE_LIMIT). Because
//   the torque controller is TorqueControlType::voltage, voltage.q maps ~linearly
//   to torque, so this compensates gravity WITHOUT the position loop having to
//   build up an error first. The feature is live-toggled at runtime ('F').
//
// Joint-frame convention (identical to ArmKinematics / ik()):
//   q1 = base yaw   — rotation about the vertical Z axis. Gravity does no work
//                     on this joint, so tau1 == 0 always.
//   q2 = shoulder pitch, measured from the horizontal plane (FK: z += L2*sin q2).
//   q3 = elbow pitch, RELATIVE to link 2 (FK: z += L3*sin(q2+q3)).
//   All angles are IK JOINT-frame radians, NOT motor-frame. main.cpp maps the
//   resulting per-joint voltage back to the motor frame (JOINT_DIR / JOINT_GEAR).
//
// Physics parameters (masses, COM distances, Kt, R, ...) live in config.h.

// Gravity-compensation torque at each joint, in the IK joint frame. [N·m]
struct GravityTorques {
  float tau1;   // joint 1 (base yaw)  — expected ~0
  float tau2;   // joint 2 (shoulder)
  float tau3;   // joint 3 (elbow)
};

// Compute the gravity-compensation joint torques for configuration (q1,q2,q3).
//
// >>> PHYSICS GOES HERE — implemented in GravityFF.cpp. <<<
// A reference 2-link derivation is written out in the .cpp as comments; the body
// currently returns zeros so the feature is a safe no-op until you fill it in.
GravityTorques gravity_torques(float q1, float q2, float q3);

// Convert a joint torque [N·m] to the q-axis feedforward voltage [V] for
// voltage-mode torque control of the given joint (index 0..2).
//   Vq = tau * R / Kt   (stall / low-speed approximation, back-EMF ignored)
// Uses per-joint Kt (M*_TORQUE_CONSTANT) and R (M*_PHASE_RESISTANCE) from config.
// Gear reduction and sensor direction are applied by the caller (main.cpp).
float torque_to_voltage(uint8_t joint, float tau);
