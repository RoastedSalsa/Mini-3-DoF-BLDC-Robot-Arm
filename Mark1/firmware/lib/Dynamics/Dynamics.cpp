#include "Dynamics.h"
#include "config.h"   // GRAV_G, LINK*_MASS, LINK*_COM, LINK*_INERTIA,
                      // L2, M*_TORQUE_CONSTANT, M*_PHASE_RESISTANCE, DYN_ENABLE_*

// ============================================================================
//  Derivation
// ============================================================================
// Point-mass links: m2 at lc2 along link 2 (from the joint-2 axis), m3 at lc3
// along link 3 (from the joint-3 axis), plus rotational inertias I1 (base yaw,
// about vertical), I2 and I3 (pitch, about each link's own COM).
//
// In the vertical plane, with r = radial distance from the yaw axis:
//   link-2 COM:  r2 = lc2 c2                z2 = lc2 s2
//   link-3 COM:  r3 = L2 c2 + lc3 c23       z3 = L2 s2 + lc3 s23
// Yawing by q1 sweeps each COM on a circle of radius r_i, contributing r_i^2 q1d^2.
//
// Kinetic energy separates into a yaw part and a planar part:
//   2T = M11 q1d^2  +  a q2d^2 + b (q2d+q3d)^2 + 2 c cos(q3) q2d (q2d+q3d)
// with the shorthands
//   a = m2 lc2^2 + m3 L2^2 + I2      b = m3 lc3^2 + I3      c = m3 L2 lc3
//
// so the mass matrix is
//   M11 = I1 + m2 lc2^2 c2^2 + m3 (L2 c2 + lc3 c23)^2
//   M22 = a + b + 2 c cos q3      M23 = M32 = b + c cos q3      M33 = b
//   M12 = M13 = 0   (yaw KE has no q1d*q2d or q1d*q3d cross term)
//
// Potential energy  U = g [ m2 lc2 s2 + m3 (L2 s2 + lc3 s23) ]  gives
//   G1 = 0
//   G2 = g [ m2 lc2 c2 + m3 (L2 c2 + lc3 c23) ]
//   G3 = g m3 lc3 c23
//
// Velocity-product terms from the Christoffel symbols
// C_ijk = 1/2 (dM_ij/dq_k + dM_ik/dq_j - dM_jk/dq_i), with h = -c sin q3 and
// dM11_2 = dM11/dq2, dM11_3 = dM11/dq3:
//
//   tau1 =  dM11_2 q1d q2d  +  dM11_3 q1d q3d            <- all CORIOLIS
//   tau2 = -1/2 dM11_2 q1d^2  +  h q3d^2                 <- CENTRIFUGAL
//          +  2 h q2d q3d                                <- CORIOLIS
//   tau3 = -1/2 dM11_3 q1d^2  -  h q2d^2                 <- CENTRIFUGAL
//
// The split between "centrifugal" and "Coriolis" is the textbook one: squared
// rates (qd_i^2) are centrifugal, mixed products (qd_i qd_j, i != j) are Coriolis.
// Note tau1 has NO centrifugal contribution and no gravity — the yaw joint only
// ever sees inertia and Coriolis.
//
// Ignored on purpose: joint friction, link-2/3 yaw inertia beyond the parallel-
// axis point-mass term, and motor rotor inertia reflected through the gearing.
// Those are the first things to add if the model under-predicts.
// ============================================================================

namespace {

// Model shorthands (see derivation above). All constexpr, so the compiler folds
// them into the trig expressions below.
constexpr float A_COEF = LINK2_MASS * LINK2_COM * LINK2_COM
                       + LINK3_MASS * L2 * L2
                       + LINK2_INERTIA;
constexpr float B_COEF = LINK3_MASS * LINK3_COM * LINK3_COM + LINK3_INERTIA;
constexpr float C_COEF = LINK3_MASS * L2 * LINK3_COM;

}  // namespace

uint8_t dynamics_default_terms() {
  uint8_t m = DYN_NONE;
  if (DYN_ENABLE_GRAVITY)     m |= DYN_GRAVITY;
  if (DYN_ENABLE_INERTIA)     m |= DYN_INERTIA;
  if (DYN_ENABLE_CENTRIFUGAL) m |= DYN_CENTRIFUGAL;
  if (DYN_ENABLE_CORIOLIS)    m |= DYN_CORIOLIS;
  return m;
}

const char* dynamics_term_name(uint8_t term_bit) {
  switch (term_bit) {
    case DYN_GRAVITY:     return "gravity";
    case DYN_INERTIA:     return "inertia";
    case DYN_CENTRIFUGAL: return "centrifugal";
    case DYN_CORIOLIS:    return "coriolis";
    default:              return "?";
  }
}

// ============================================================================
//  tau = M(q) qdd + C(q, qd) qd + G(q), evaluating only the selected terms
// ============================================================================
JointTorques joint_torques(const float q[3], const float qd[3], const float qdd[3],
                           uint8_t terms) {
  JointTorques t{0.0f, 0.0f, 0.0f};
  if (terms == DYN_NONE) return t;

  // q1 (base yaw) never enters M, C or G — the arm's mass distribution about the
  // yaw axis is the same at every yaw angle. Only its RATE matters.
  const float c2  = cosf(q[1]),         s2  = sinf(q[1]);
  const float c23 = cosf(q[1] + q[2]),  s23 = sinf(q[1] + q[2]);

  // ---- gravity ------------------------------------------------------------
  if (terms & DYN_GRAVITY) {
    t.tau3 += GRAV_G * LINK3_MASS * LINK3_COM * c23;
    t.tau2 += GRAV_G * (LINK2_MASS * LINK2_COM * c2
                        + LINK3_MASS * (L2 * c2 + LINK3_COM * c23));
    // t.tau1 += 0 — gravity does no work on the yaw joint.
  }

  // Everything below needs the velocity-dependent geometry.
  const bool need_M  = (terms & DYN_INERTIA) != 0;
  const bool need_vp = (terms & (DYN_CENTRIFUGAL | DYN_CORIOLIS)) != 0;
  if (!need_M && !need_vp) return t;

  const float c3 = cosf(q[2]);
  const float P  = L2 * c2 + LINK3_COM * c23;   // radial dist. of the link-3 COM
  const float Q  = L2 * s2 + LINK3_COM * s23;   // height  of the link-3 COM

  // ---- inertia:  M(q) qdd -------------------------------------------------
  if (need_M) {
    const float M11 = LINK1_INERTIA
                    + LINK2_MASS * LINK2_COM * LINK2_COM * c2 * c2
                    + LINK3_MASS * P * P;
    const float M23 = B_COEF + C_COEF * c3;
    const float M22 = A_COEF + B_COEF + 2.0f * C_COEF * c3;
    const float M33 = B_COEF;

    t.tau1 += M11 * qdd[0];
    t.tau2 += M22 * qdd[1] + M23 * qdd[2];
    t.tau3 += M23 * qdd[1] + M33 * qdd[2];
  }

  // ---- velocity-product terms:  C(q, qd) qd -------------------------------
  if (need_vp) {
    const float s3 = sinf(q[2]);
    const float h  = -C_COEF * s3;

    // Partials of M11 — the only mass-matrix entry that varies with the pose in
    // a way the yaw joint feels.
    const float dM11_2 = -2.0f * (LINK2_MASS * LINK2_COM * LINK2_COM * c2 * s2
                                  + LINK3_MASS * P * Q);
    const float dM11_3 = -2.0f * LINK3_MASS * LINK3_COM * s23 * P;

    if (terms & DYN_CENTRIFUGAL) {
      const float w1sq = qd[0] * qd[0];
      t.tau2 += -0.5f * dM11_2 * w1sq + h * qd[2] * qd[2];
      t.tau3 += -0.5f * dM11_3 * w1sq - h * qd[1] * qd[1];
      // tau1 has no centrifugal term.
    }
    if (terms & DYN_CORIOLIS) {
      t.tau1 += dM11_2 * qd[0] * qd[1] + dM11_3 * qd[0] * qd[2];
      t.tau2 += 2.0f * h * qd[1] * qd[2];
      // tau3 has no Coriolis term.
    }
  }

  return t;
}

JointTorques gravity_torques(float q1, float q2, float q3) {
  const float q[3]    = {q1, q2, q3};
  const float zero[3] = {0.0f, 0.0f, 0.0f};
  return joint_torques(q, zero, zero, DYN_GRAVITY);
}

// ============================================================================
//  Joint torque [N·m] -> q-axis feedforward voltage [V]
// ============================================================================
// Voltage-mode torque control: with no current sensing, SimpleFOC drives
// voltage.q directly. At low speed the q-current is Iq ~= Vq / R and torque is
// tau ~= Kt * Iq, so   Vq = tau * R / Kt.   Per-joint Kt and R (motors differ).
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
