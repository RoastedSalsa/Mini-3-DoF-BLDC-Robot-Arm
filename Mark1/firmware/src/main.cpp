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
// NOTE: there is currently no joint homing — the IK joint angles are commanded
// directly in the motor frame (no startup offset). A new homing scheme is TBD.
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
float theta1 = 0, theta2 = 0, theta3 = 0; // IK joint angles [rad]
float th1 = 0, th2 = 0, th3 = 0;          // motor-frame targets (commanded) [rad]
float meas1 = 0, meas2 = 0, meas3 = 0;    // measured sensor angles [rad]
float err1 = 0, err2 = 0, err3 = 0;       // command - measured [rad]

bool traj_enabled = true;    // trajectory running, or holding a manual target?

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
  command.add('H', doHelp,   "help");

  registerTelemetry();
  trajectory.begin(millis());
  printHelp();
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

  // 3. Cartesian target -> joint angles -> motor frame (J2 gear ratio only;
  //    no homing offset for now).
  ik(x, y, z, theta1, theta2, theta3);
  th1 = theta1;
  th2 = theta2 * J2_GEAR_RATIO;
  th3 = theta3;

  // 4. Software travel limits before commanding (mechanical hard-stop guard).
  th1 = constrain(th1, J1_MIN, J1_MAX);
  th2 = constrain(th2, J2_MIN, J2_MAX);
  th3 = constrain(th3, J3_MIN, J3_MAX);

  motor1.move(th1);
  motor2.move(th2);
  motor3.move(th3);

  // 5. Snapshot measured angles + tracking error, then stream telemetry
  //    (JSON lines, throttled internally; see lib/Telemetry).
  meas1 = sensor1.getAngle(); err1 = th1 - meas1;
  meas2 = sensor2.getAngle(); err2 = th2 - meas2;
  meas3 = sensor3.getAngle(); err3 = th3 - meas3;
  telemetry.update(now);
}
