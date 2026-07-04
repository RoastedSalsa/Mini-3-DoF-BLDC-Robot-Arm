#include "GravityFF.h"
#include "config.h"   // GRAV_G, LINK*_MASS, LINK*_COM, M*_TORQUE_CONSTANT, M*_PHASE_RESISTANCE

// ============================================================================
//  Gravity-compensation joint torques
// ============================================================================
//
//  >>>>>>>>>>>>>>>>>>>>>>>>  ADD THE PHYSICS HERE  <<<<<<<<<<<<<<<<<<<<<<<<<<<
//
//  This 3-DoF arm is, for gravity purposes, a 2-link planar manipulator in a
//  vertical plane (link 2 + link 3) mounted on a vertical yaw axis (joint 1).
//  Joint 1 rotates about vertical, so gravity applies NO torque there -> tau1 = 0.
//
//  Reference form (RR planar arm in a vertical plane, angles from horizontal):
//    Let m2, m3      = masses of links 2 and 3            (LINK2_MASS, LINK3_MASS)
//        lc2, lc3    = COM distance from each joint axis  (LINK2_COM,  LINK3_COM)
//        L2          = length of link 2                   (from config.h)
//        g           = GRAV_G
//
//      tau2 = ( m2*lc2 + m3*L2 ) * g * cos(q2)  +  m3*lc3 * g * cos(q2 + q3)
//      tau3 =                                       m3*lc3 * g * cos(q2 + q3)
//
//  Sign convention: tau is the torque the JOINT must apply to hold the arm up,
//  in the +q direction of the IK frame. main.cpp reflects it through JOINT_DIR /
//  JOINT_GEAR to reach the motor frame. Verify the sign on hardware at low
//  feedforward gain before trusting it.
//
//  Add a payload by folding its mass into m3 / lc3, or extend the model with a
//  4th term. Keep the body pure (no I/O, no globals beyond config constants) so
//  it stays cheap enough to call every control loop.
// ----------------------------------------------------------------------------
GravityTorques gravity_torques(float q1, float q2, float q3) {
  (void)q1;   // base yaw contributes no gravity torque

  GravityTorques t;
  t.tau1 = 0.0f;

  // --- USER PHYSICS: replace the zeros below -------------------------------
  t.tau2 = 0.0f;
  t.tau3 = 0.0f;
  // -------------------------------------------------------------------------

  return t;
}

// ============================================================================
//  Joint torque [N·m] -> q-axis feedforward voltage [V]
// ============================================================================
// Voltage-mode torque control: with no current sensing, SimpleFOC drives
// voltage.q directly. At low speed the q-current is Iq ≈ Vq / R and torque is
// tau ≈ Kt * Iq, so   Vq = tau * R / Kt.   Per-joint Kt and R (motors differ).
float torque_to_voltage(uint8_t joint, float tau) {
  float kt, r;
  switch (joint) {
    case 0: kt = M1_TORQUE_CONSTANT; r = M1_PHASE_RESISTANCE; break;
    case 1: kt = M2_TORQUE_CONSTANT; r = M2_PHASE_RESISTANCE; break;
    case 2: kt = M3_TORQUE_CONSTANT; r = M3_PHASE_RESISTANCE; break;
    default: return 0.0f;
  }
  if (kt <= 0.0f) return 0.0f;   // unconfigured joint -> no feedforward
  return tau * r / kt;
}
