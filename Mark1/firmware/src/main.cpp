// main.cpp — main firmware for the Mini 3-DoF BLDC arm.
//
// Closed-loop FOC angle control on all three joints, driven by an inverse-
// kinematics + trajectory pipeline:
//
//   Trajectory (lib/Trajectory) --> (x,y,z)
//        --> ik() (lib/ArmKinematics) --> joint angles
//        --> J2 gear ratio + software limits
//        --> motor.move()
//        --> Telemetry (lib/Telemetry) streams cmd-vs-measured over serial
//
//
// All hardware pins, control gains, calibration, limits and trajectory waypoints
// live in include/config.h. Tune the PID gains live with the `pid_tuner` tool
// (src/calibration/pid_tuner.cpp); see docs/tuning-guide.md.
//
// Operator input goes through the SimpleFOC Commander, exactly like pid_tuner,
// so it is driven over the ROS2 bridge (publish to /mini_ranka/cmd, replies come
// back on /mini_ranka/log) as well as a plain serial monitor. See the commands
// below and docs/plotjuggler.md.
//
// Commands
//   1… 2… 3…     full SimpleFOC motor command for joint 1/2/3 — live gain access:
//                   1MG0       print joint-1 gains/limits
//                   2VP0.3     joint-2 velocity PID P   2VI0.1  vel I
//                   3AP12      joint-3 angle P          3AI1.0  angle I  3AD0.05 D
//   T<x> <y> <z>  set a manual Cartesian target [m] (pauses the trajectory)
//   G             resume the continuous trajectory
//   F             master on/off for the dynamics feedforward
//   D             print / select which dynamics terms are active (DG DI DC DK DA DN)
//   H             print this help
//
// Telemetry (JSON, lib/Telemetry):  {"t",x,y,z,cmd1,meas1,err1, …}

#include <Arduino.h>
#include <Wire.h>
#include <SimpleFOC.h>
#include "config.h"
#include "ArmKinematics.h"
#include "Trajectory.h"
#include "Telemetry.h"
#include "Dynamics.h"

// ============================ Hardware objects ===============================
// One AS5600 magnetic encoder per joint, each on its own hardware I2C bus.
TwoWire myWire1(S1_SDA_PIN, S1_SCL_PIN);
TwoWire myWire2(S2_SDA_PIN, S2_SCL_PIN);
TwoWire myWire3(S3_SDA_PIN, S3_SCL_PIN);
MagneticSensorI2C sensor1 = MagneticSensorI2C(AS5600_I2C);
MagneticSensorI2C sensor2 = MagneticSensorI2C(AS5600_I2C);
MagneticSensorI2C sensor3 = MagneticSensorI2C(AS5600_I2C);

BLDCMotor motor1 = BLDCMotor(M1_POLE_PAIRS);
BLDCMotor motor2 = BLDCMotor(M2_POLE_PAIRS);
BLDCMotor motor3 = BLDCMotor(M3_POLE_PAIRS);
BLDCDriver3PWM driver1 = BLDCDriver3PWM(M1_PH_A, M1_PH_B, M1_PH_C, M1_EN);  // TIM1
BLDCDriver3PWM driver2 = BLDCDriver3PWM(M2_PH_A, M2_PH_B, M2_PH_C, M2_EN);  // TIM2
BLDCDriver3PWM driver3 = BLDCDriver3PWM(M3_PH_A, M3_PH_B, M3_PH_C, M3_EN);  // TIM3

// ============================ Software modules ===============================
Trajectory trajectory(TRAJ_PX, TRAJ_PY, TRAJ_PZ, TRAJ_COUNT, TRAJ_SEGMENT_TIME_S);
Telemetry  telemetry(Serial, TELEMETRY_PERIOD_MS);
Commander  command = Commander(Serial);   // operator input -> ROS2 bridge /cmd

// ============================== Cascade gains ================================
// Per-joint velocity + angle PID set
struct JointGains {
  float vel_p, vel_i, out_ramp;
  float ang_p, ang_i, ang_d;
  float vel_limit, lpf_tf, zero_elec;
};
constexpr JointGains GAINS1{M1_VEL_P, M1_VEL_I, M1_OUTPUT_RAMP,
                            M1_ANGLE_P, M1_ANGLE_I, M1_ANGLE_D,
                            M1_VEL_LIMIT, M1_LPF_TF, M1_ZERO_ELEC_ANGLE};
constexpr JointGains GAINS2{M2_VEL_P, M2_VEL_I, M2_OUTPUT_RAMP,
                            M2_ANGLE_P, M2_ANGLE_I, M2_ANGLE_D,
                            M2_VEL_LIMIT, M2_LPF_TF, M2_ZERO_ELEC_ANGLE};
constexpr JointGains GAINS3{M3_VEL_P, M3_VEL_I, M3_OUTPUT_RAMP,
                            M3_ANGLE_P, M3_ANGLE_I, M3_ANGLE_D,
                            M3_VEL_LIMIT, M3_LPF_TF, M3_ZERO_ELEC_ANGLE};

// =============================== Run-time state ==============================
float x = 0.2f, y = 0.0f, z = 0.2488f;   // active Cartesian target [m]
float th1 = 0, th2 = 0, th3 = 0; // IK joint angles [rad]
float theta1 = 0, theta2 = 0, theta3 = 0;          // motor-frame targets (commanded) [rad]
float meas1 = 0, meas2 = 0, meas3 = 0;    // measured sensor angles [rad]
float err1 = 0, err2 = 0, err3 = 0;       // command - measured [rad]
float q1=0, q2=0,q3=0;
float home_offset[3] = {HOME_OFFSET_1, HOME_OFFSET_2, HOME_OFFSET_3};
bool  homed = false;            // status only — false = running on config defaults

float meas_x = 0, meas_y = 0, meas_z = 0;   // measured end-effector pos (FK) [m]
bool  cart_meas_enabled = false;            // toggle Cartesian measurement

bool traj_enabled = true;    // trajectory running, or holding a manual target?

bool    dyn_ff_enabled = DYN_FF_DEFAULT_ON;   // dynamics feedforward master on/off ('F')
uint8_t dyn_terms      = 0;                   // which model terms are active ('D'), set in setup()
float   ff1 = 0, ff2 = 0, ff3 = 0;            // injected feedforward voltage [V] (telemetry)

// Joint-space setpoints and their derivatives, IK joint frame. qd/qdd are obtained
// by differentiating the position setpoint (the trajectory only produces position),
// each passed through a first-order low-pass so a LINEAR-eased segment boundary
// does not turn into an acceleration spike. Desired rates are used rather than
// measured ones so the feedforward stays open-loop and cannot inject sensor noise
// back into the drive.
float q_des[3]   = {0, 0, 0};
float qd_des[3]  = {0, 0, 0};
float qdd_des[3] = {0, 0, 0};
unsigned long dyn_last_us = 0;   // 0 = no previous sample yet

// ============================= Setup helpers =================================

// Bring one joint fully online: I2C bus, encoder, driver, cascaded PID, FOC
// alignment and angle-control mode. Centralizing this guarantees all three
// joints are configured identically — only the gains differ.
static void configureJoint(const char* name,
                           TwoWire& bus, MagneticSensorI2C& sensor,
                           BLDCDriver3PWM& driver, BLDCMotor& motor,
                           const JointGains& g) {
  bus.begin();
  bus.setClock(I2C_CLOCK_HZ);
  sensor.init(&bus);
  delay(50);

  driver.voltage_power_supply = SUPPLY_VOLTAGE;
  driver.init();

  motor.linkSensor(&sensor);
  motor.linkDriver(&driver);
  motor.voltage_limit            = VOLTAGE_LIMIT;
  motor.PID_velocity.P           = g.vel_p;
  motor.PID_velocity.I           = g.vel_i;
  motor.PID_velocity.output_ramp = g.out_ramp;
  motor.P_angle.P                = g.ang_p;
  motor.P_angle.I                = g.ang_i;
  motor.P_angle.D                = g.ang_d;
  motor.velocity_limit           = g.vel_limit;
  motor.LPF_velocity.Tf          = g.lpf_tf;
  motor.zero_electric_angle      = g.zero_elec;
  motor.sensor_direction         = Direction::CW;
  motor.init();
  motor.initFOC();
  motor.controller               = MotionControlType::angle; //Amgle for now, will switch to torque when I add the current sensors.

  Serial.print(name);
  Serial.println(F(" initialized"));
}

void printHelp() {
  Serial.println(F("--- mini_ranka ---"));
  Serial.println(F("1.. 2.. 3.. : SimpleFOC motor cmd per joint (1MG0, 1VP, 1AP, ...)"));
  Serial.println(F("T<x> <y> <z>: manual Cartesian target [m] (pauses trajectory)"));
  Serial.println(F("G   : resume trajectory"));
  Serial.println(F("F   : toggle dynamics feedforward (master)"));
  Serial.println(F("D   : dynamics terms - DG grav, DI inertia, DC centrif, DK coriolis, DA all, DN none"));
  Serial.println(F("H   : help"));
  Serial.println(F("JSON: {t,x,y,z,cmdN,measN,errN} -> ROS2 bridge -> PlotJuggler"));
}

// --- Commander callbacks ---------------------------------------------------
// Each joint exposes the full SimpleFOC motor command set for live gain access.
void doMotor1(char* cmd) { command.motor(&motor1, cmd); }
void doMotor2(char* cmd) { command.motor(&motor2, cmd); }
void doMotor3(char* cmd) { command.motor(&motor3, cmd); }
void doHelp(char* /*cmd*/) { printHelp(); }

void doResume(char* /*cmd*/) {
  traj_enabled = true;
  Serial.println(F("Trajectory resumed"));
}

// Manual Cartesian target: "T<x> <y> <z>" [m], pauses the trajectory.
void doTarget(char* cmd) {
  // Parse three floats by hand. newlib-nano's sscanf(%f) is a silent no-op
  // unless the build links -u _scanf_float, so use strtod (the atof path).
  char* end = nullptr;
  float nx = strtod(cmd, &end);
  if (end == cmd) { Serial.println(F("usage: T<x> <y> <z>")); return; }
  float ny = strtod(end, &end);
  float nz = strtod(end, &end);
  x = nx; y = ny; z = nz;
  traj_enabled = false;
  Serial.print(F("Manual target (x,y,z): "));
  Serial.print(x, 3); Serial.print(F(", "));
  Serial.print(y, 3); Serial.print(F(", "));
  Serial.println(z, 3);
}

void doCartToggle(char* /*cmd*/) {
  cart_meas_enabled = !cart_meas_enabled;
  if (!cart_meas_enabled) { meas_x = meas_y = meas_z = 0; }   // park at 0 when off
  Serial.print(F("cartesian measurement: "));
  Serial.println(cart_meas_enabled ? F("on") : F("off"));
}

// Home: arm is physically at the Cartesian home pose -> capture sensor offsets.
void doHome(char* /*cmd*/) {
  float hq1, hq2, hq3;
  if (!ik(HOME_X, HOME_Y, HOME_Z, hq1, hq2, hq3)) {     // needs the reachability guard
    Serial.println(F("home unreachable")); return;
  }
  home_offset[0] = sensor1.getAngle() + hq1;
  home_offset[1] = sensor2.getAngle() - hq2 * JOINT_GEAR[1];   // J2 motor-side
  home_offset[2] = sensor3.getAngle() - hq3;
  homed = true;
  Serial.print(F("homed offsets: "));
  Serial.print(home_offset[0],4); Serial.print(' ');
  Serial.print(home_offset[1],4); Serial.print(' ');
  Serial.println(home_offset[2],4);
  Serial.print(F("home_angle"));
  Serial.print(hq1);
  Serial.print(sensor1.getAngle());
}
// Dynamics feedforward master switch. Off by default; capped by DYN_FF_VOLTAGE_LIMIT.
void doDynFF(char* /*cmd*/) {
  dyn_ff_enabled = !dyn_ff_enabled;
  Serial.print(F("dynamics feedforward: "));
  Serial.println(dyn_ff_enabled ? F("on") : F("off"));
}

// Report which model terms are currently active.
static void printDynTerms() {
  Serial.print(F("dynamics terms:"));
  const uint8_t bits[4] = {DYN_GRAVITY, DYN_INERTIA, DYN_CENTRIFUGAL, DYN_CORIOLIS};
  for (uint8_t i = 0; i < 4; ++i) {
    Serial.print(' ');
    Serial.print(dynamics_term_name(bits[i]));
    Serial.print((dyn_terms & bits[i]) ? F("=on") : F("=off"));
  }
  Serial.println();
}

// Select which terms of tau = M(q)qdd + C(q,qd)qd + G(q) are evaluated. Each
// letter toggles one term, so you can bring the model up a piece at a time and
// watch ff1..ff3 in PlotJuggler to see what each one contributes.
//
//   D    print the current selection      DG  gravity        DI  inertia
//   DA   all terms on                     DC  centrifugal    DK  Coriolis
//   DN   all terms off
//
// Works identically over the ROS2 bridge, which forwards String messages
// verbatim to the Commander:
//   ros2 topic pub --once /mini_ranka/cmd std_msgs/String "data: 'DI'"
void doDynTerms(char* cmd) {
  // Commander hands us everything after the 'D'; skip leading blanks.
  while (*cmd == ' ') ++cmd;
  switch (toupper(*cmd)) {
    case '\0': break;                              // bare 'D' -> just report
    case 'A': dyn_terms = DYN_ALL;  break;
    case 'N': dyn_terms = DYN_NONE; break;
    case 'G': dyn_terms ^= DYN_GRAVITY;     break;
    case 'I': dyn_terms ^= DYN_INERTIA;     break;
    case 'C': dyn_terms ^= DYN_CENTRIFUGAL; break;
    case 'K': dyn_terms ^= DYN_CORIOLIS;    break;
    default:
      Serial.println(F("usage: D | DG | DI | DC | DK | DA | DN"));
      return;
  }
  printDynTerms();
}

// Differentiate the joint setpoint to get qd and qdd for the velocity- and
// acceleration-dependent model terms. Called once per loop with the current IK
// joint angles; dt comes from micros() because the loop rate is not fixed.
//
// Both derivatives are low-pass filtered (DYN_VEL_LPF_TF / DYN_ACCEL_LPF_TF).
// The first sample after boot only seeds the state — differentiating against an
// undefined previous value would produce one enormous bogus acceleration.
static void updateSetpointDerivatives(float t1, float t2, float t3, unsigned long now_us) {
  const float target[3] = {t1, t2, t3};

  if (dyn_last_us == 0) {                       // first call: seed, no derivative
    for (uint8_t i = 0; i < 3; ++i) q_des[i] = target[i];
    dyn_last_us = now_us;
    return;
  }

  const float dt = (now_us - dyn_last_us) * 1e-6f;   // unsigned wrap is well-defined
  dyn_last_us = now_us;
  if (dt <= 0.0f || dt > 0.1f) {
    // Loop stalled (or micros() wrapped oddly). The arm may have moved a long way
    // since the last sample, so the stored rates are meaningless — drop them to
    // zero rather than feed a stale acceleration into the drive, and resync.
    for (uint8_t i = 0; i < 3; ++i) { q_des[i] = target[i]; qd_des[i] = 0; qdd_des[i] = 0; }
    return;
  }

  // First-order LPF blend factors: alpha = dt / (Tf + dt).
  const float a_vel = dt / (DYN_VEL_LPF_TF   + dt);
  const float a_acc = dt / (DYN_ACCEL_LPF_TF + dt);

  for (uint8_t i = 0; i < 3; ++i) {
    const float vel_raw = (target[i] - q_des[i]) / dt;
    const float vel_new = qd_des[i] + a_vel * (vel_raw - qd_des[i]);
    const float acc_raw = (vel_new - qd_des[i]) / dt;
    qdd_des[i] = qdd_des[i] + a_acc * (acc_raw - qdd_des[i]);
    qd_des[i]  = vel_new;
    q_des[i]   = target[i];
  }
}

// Inject the model feedforward voltage on each motor's q-axis, ON TOP OF the
// angle-loop output already set by motor.move(). Called every loop right after
// move(), using the IK JOINT-frame setpoint and its derivatives.
//
// Per joint: joint torque -> motor torque (÷ gear) -> voltage (Kt/R) -> motor
// frame (× JOINT_DIR) -> scaled (DYN_FF_GAIN) -> clamped (DYN_FF_VOLTAGE_LIMIT).
// The final voltage.q is then re-clamped to VOLTAGE_LIMIT so we never exceed it.
static void applyDynamicsFF() {
  JointTorques t = joint_torques(q_des, qd_des, qdd_des, dyn_terms);
  const float tau[3] = { t.tau1, t.tau2, t.tau3 };
  BLDCMotor* motors[3] = { &motor1, &motor2, &motor3 };
  float* ff[3] = { &ff1, &ff2, &ff3 };

  for (uint8_t i = 0; i < 3; ++i) {
    float v = JOINT_DIR[i] * torque_to_voltage(i, tau[i] / JOINT_GEAR[i]) * DYN_FF_GAIN;
    v = constrain(v, -DYN_FF_VOLTAGE_LIMIT, DYN_FF_VOLTAGE_LIMIT);
    *ff[i] = v;
    float vq = constrain(motors[i]->voltage.q + v, -VOLTAGE_LIMIT, VOLTAGE_LIMIT);
    motors[i]->voltage.q = vq;
  }
}

// ================================= Arduino ===================================

// Register every signal we want on the wire. Each is backed by a live variable,
// so adding a plot is a single add() line here — the host ROS2 bridge and
// PlotJuggler pick up new keys automatically (see docs/plotjuggler.md).
static void registerTelemetry() {
  telemetry.add("x", &x);          // Cartesian target [m]
  telemetry.add("y", &y);
  telemetry.add("z", &z);
  telemetry.add("cmd1", &th1);     // commanded motor-frame angle [rad]
  telemetry.add("meas1", &meas1);  // measured sensor angle [rad]
  telemetry.add("err1", &err1);    // command - measured [rad]
  telemetry.add("cmd2", &th2);
  telemetry.add("meas2", &meas2);
  telemetry.add("err2", &err2);
  telemetry.add("cmd3", &th3);
  telemetry.add("meas3", &meas3);
  telemetry.add("err3", &err3);
  telemetry.add("q1", &q1);        // measured angle in the IK JOINT frame [rad]
  telemetry.add("q2", &q2);        // (meas* are sensor frame: geared + offset)
  telemetry.add("q3", &q3);        // -> drives the ROS2 twin, see Mark1/ros2
  telemetry.add("mx", &meas_x);   // measured end-effector X (FK of measured angles) [m]
  telemetry.add("my", &meas_y);
  telemetry.add("mz", &meas_z);
  telemetry.add("ff1", &ff1);     // injected gravity feedforward voltage [V]
  telemetry.add("ff2", &ff2);
  telemetry.add("ff3", &ff3);
}

void setup() {
  Serial.begin(TELEMETRY_BAUD);
  while (!Serial && millis() < 2000);
  Serial.setTimeout(10);   // bound serial reads so they never stall the FOC loop

  configureJoint("Motor 1", myWire1, sensor1, driver1, motor1, GAINS1);
  configureJoint("Motor 2", myWire2, sensor2, driver2, motor2, GAINS2);
  configureJoint("Motor 3", myWire3, sensor3, driver3, motor3, GAINS3);

  command.add('1', doMotor1, "joint 1 motor");
  command.add('2', doMotor2, "joint 2 motor");
  command.add('3', doMotor3, "joint 3 motor");
  command.add('T', doTarget, "target x y z [m]");
  command.add('G', doResume, "resume trajectory");
  command.add('H', doHelp, "Help");
  command.add('O', doHome, "at origin, add home offsets");
  command.add('C', doCartToggle,"toggle cartesian measurement");
  command.add('F', doDynFF,     "toggle dynamics feedforward");
  command.add('D', doDynTerms,  "select dynamics terms");

  dyn_terms = dynamics_default_terms();   // DYN_ENABLE_* from config.h

  registerTelemetry();
  trajectory.begin(millis());
  printHelp();
  printDynTerms();
  Serial.println(F("Arm ready."));
}

void loop() {
  // 1. Update sensors and run the FOC loops.
  sensor1.update();
  sensor2.update();
  sensor3.update();
  motor1.loopFOC();
  motor2.loopFOC();
  motor3.loopFOC();

  // 2. Operator input (Commander), then advance the trajectory (unless a manual hold).
  command.run();
  unsigned long now = millis();
  if (traj_enabled) trajectory.update(now, x, y, z);


  // 3. Cartesian target -> joint angles -> motor frame

  ik(x, y, z, theta1, theta2, theta3);

  theta1 = constrain(theta1, J1_MIN, J1_MAX);
  theta2 = constrain(theta2, J2_MIN, J2_MAX);
  theta3 = constrain(theta3, J3_MIN, J3_MAX);

  th1 = JOINT_DIR[0] * JOINT_GEAR[0] * theta1 + home_offset[0];
  th2 = JOINT_DIR[1] * JOINT_GEAR[1] * theta2 + home_offset[1];
  th3 = JOINT_DIR[2] * JOINT_GEAR[2] * theta3 + home_offset[2];  


  // Track the joint setpoint and its derivatives every loop, whether or not the
  // feedforward is active — so toggling 'F' on mid-motion starts from a settled
  // qd/qdd rather than a startup transient.
  updateSetpointDerivatives(theta1, theta2, theta3, micros());

  motor1.move(th1);
  motor2.move(th2);
  motor3.move(th3);

  // Add the model feedforward voltage on top of the angle-loop output, so the
  // next loopFOC() drives the combined voltage.q. Still a no-op until the
  // M*_TORQUE_CONSTANT / M*_PHASE_RESISTANCE values in config.h are filled in.
  if (dyn_ff_enabled) applyDynamicsFF();
  else                ff1 = ff2 = ff3 = 0;   // don't leave stale values on the wire

  // 4. Snapshot measured angles + tracking error, then stream telemetry
  //    (JSON lines, throttled internally; see lib/Telemetry).
  meas1 = sensor1.getAngle(); err1 = th1 - meas1;
  meas2 = sensor2.getAngle(); err2 = th2 - meas2;
  meas3 = sensor3.getAngle(); err3 = th3 - meas3;

  // Sensor frame -> IK joint frame. Unconditional: q1..q3 are what the ROS2
  // digital twin drives the model with (joint_state_bridge), and gating them on
  // the 'C' toggle would leave RViz frozen until someone remembered to press it.
  // These are also the only place the LIVE home offsets appear on the wire, so
  // the host never has to keep its own copy of them in sync with config.h.
  q1 = JOINT_DIR[0] * (meas1 - home_offset[0]) / JOINT_GEAR[0];
  q2 = JOINT_DIR[1] * (meas2 - home_offset[1]) / JOINT_GEAR[1];   // J2 motor-side -> joint
  q3 = JOINT_DIR[2] * (meas3 - home_offset[2]) / JOINT_GEAR[2];

  // The Cartesian FK stays behind the toggle — it costs three trig calls per
  // loop and is only wanted when you are watching the tool position.
  if (cart_meas_enabled) {
    fk(meas_x, meas_y, meas_z, q1, q2, q3);                // measured end-effector pos
  }
  telemetry.update(now);
}
