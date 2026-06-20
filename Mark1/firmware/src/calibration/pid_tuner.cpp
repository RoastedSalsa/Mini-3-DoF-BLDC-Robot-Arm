// tools/pid_tuner.cpp — live ("continuous") PID tuning for one joint.
//
// Brings a single selected joint online using the shared config.h pin map,
// calibration and starting gains, then exposes the full SimpleFOC Commander
// interface so EVERY cascade gain can be adjusted over serial WHILE the motor
// runs — no recompiling, no reflashing. A built-in square-wave step generator
// lets you watch the step response continuously as you turn the knobs, and a
// JSON telemetry stream (lib/Telemetry) feeds the ROS2 bridge -> PlotJuggler.
//
// Build:  pio run -e pid_tuner -t upload   (set TUNE_JOINT below first)
// Serial: 921600 baud (TELEMETRY_BAUD), newline line-endings.
//
// The commands below drive the SimpleFOC Commander over the serial port. With
// the ROS2 bridge running (it owns the port), deliver them through it: publish
// to /mini_ranka/cmd or use the `tune` console (ros2 run mini_ranka_bridge tune).
// Commander text replies come back on /mini_ranka/log. See docs/plotjuggler.md.
//
// Commands
//   M…            full SimpleFOC motor command — live gain access, e.g.
//                   MMG0       print all gains/limits
//                   MVP0.3     velocity PID P     MVI0.1  vel I    MVD0  vel D
//                   MAP12      angle P            MAI1.0  angle I  MAD0.05 angle D
//                   MLV100     velocity limit     MLU12   voltage limit
//   G<rad>        set the target angle (constrained to the joint's limits)
//   T             toggle the continuous step (square-wave) setpoint
//   L<lo> <hi>    set the two step levels [rad]
//   P<sec>        set the step half-period [s]
//   H             print this help
//
// Telemetry (JSON, lib/Telemetry):  {"t":..,"target":..,"meas":..,"err":..,"vel":..}
//
// See docs/tuning-guide.md for the recommended tuning procedure.

#include <Arduino.h>
#include <Wire.h>
#include <SimpleFOC.h>
#include "config.h"
#include "Telemetry.h"

// =============================================================================
//  Select the joint to tune: 1, 2, or 3
// =============================================================================
#define TUNE_JOINT 3

#if   TUNE_JOINT == 1
  #define J_SDA   S1_SDA_PIN
  #define J_SCL   S1_SCL_PIN
  #define J_PH_A  M1_PH_A
  #define J_PH_B  M1_PH_B
  #define J_PH_C  M1_PH_C
  #define J_EN    M1_EN
  #define J_POLES M1_POLE_PAIRS
  constexpr float J_ZERO  = M1_ZERO_ELEC_ANGLE;
  constexpr float J_VEL_P = M1_VEL_P, J_VEL_I = M1_VEL_I, J_OUT_RAMP = M1_OUTPUT_RAMP;
  constexpr float J_ANG_P = M1_ANGLE_P, J_ANG_I = M1_ANGLE_I, J_ANG_D = M1_ANGLE_D;
  constexpr float J_VEL_LIMIT = M1_VEL_LIMIT, J_LPF = M1_LPF_TF;
  constexpr float J_MIN = J1_MIN, J_MAX = J1_MAX;
#elif TUNE_JOINT == 2
  #define J_SDA   S2_SDA_PIN
  #define J_SCL   S2_SCL_PIN
  #define J_PH_A  M2_PH_A
  #define J_PH_B  M2_PH_B
  #define J_PH_C  M2_PH_C
  #define J_EN    M2_EN
  #define J_POLES M2_POLE_PAIRS
  constexpr float J_ZERO  = M2_ZERO_ELEC_ANGLE;
  constexpr float J_VEL_P = M2_VEL_P, J_VEL_I = M2_VEL_I, J_OUT_RAMP = M2_OUTPUT_RAMP;
  constexpr float J_ANG_P = M2_ANGLE_P, J_ANG_I = M2_ANGLE_I, J_ANG_D = M2_ANGLE_D;
  constexpr float J_VEL_LIMIT = M2_VEL_LIMIT, J_LPF = M2_LPF_TF;
  constexpr float J_MIN = J2_MIN, J_MAX = J2_MAX;
#elif TUNE_JOINT == 3
  #define J_SDA   S3_SDA_PIN
  #define J_SCL   S3_SCL_PIN
  #define J_PH_A  M3_PH_A
  #define J_PH_B  M3_PH_B
  #define J_PH_C  M3_PH_C
  #define J_EN    M3_EN
  #define J_POLES M3_POLE_PAIRS
  constexpr float J_ZERO  = M3_ZERO_ELEC_ANGLE;
  constexpr float J_VEL_P = M3_VEL_P, J_VEL_I = M3_VEL_I, J_OUT_RAMP = M3_OUTPUT_RAMP;
  constexpr float J_ANG_P = M3_ANGLE_P, J_ANG_I = M3_ANGLE_I, J_ANG_D = M3_ANGLE_D;
  constexpr float J_VEL_LIMIT = M3_VEL_LIMIT, J_LPF = M3_LPF_TF;
  constexpr float J_MIN = J3_MIN, J_MAX = J3_MAX;
#else
  #error "TUNE_JOINT must be 1, 2, or 3"
#endif

// =============================================================================
//  Hardware
// =============================================================================
TwoWire jWire(J_SDA, J_SCL);
MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);
BLDCMotor    motor  = BLDCMotor(J_POLES);
BLDCDriver3PWM driver = BLDCDriver3PWM(J_PH_A, J_PH_B, J_PH_C, J_EN);
Commander command = Commander(Serial);
Telemetry telemetry(Serial, TELEMETRY_PERIOD_MS);   // JSON -> ROS2 bridge -> PlotJuggler

// =============================================================================
//  Targeting / step generator
// =============================================================================
float target = 0.5f * (J_MIN + J_MAX);          // commanded angle [rad]

// Live signals streamed as JSON (registered in registerTelemetry()).
float cmd_angle = target;                        // angle actually sent to move() [rad]
float meas      = 0;                             // measured encoder angle [rad]
float err       = 0;                             // cmd_angle - meas [rad]
float vel       = 0;                             // measured shaft velocity [rad/s]

bool          step_on   = false;                // continuous step active?
float         step_lo   = 0, step_hi = 0;       // the two step levels [rad]
float         step_half_period = 1.0f;          // seconds per level
unsigned long step_t0   = 0;
bool          step_high = false;

void printHelp() {
  Serial.println(F("--- pid_tuner ---"));
  Serial.print  (F("Tuning joint ")); Serial.println(TUNE_JOINT);
  Serial.println(F("M.. : SimpleFOC motor cmd (MMG0, MVP, MVI, MAP, MAI, MAD, MLV, MLU)"));
  Serial.println(F("G<r>: set target [rad]"));
  Serial.println(F("T   : toggle continuous step"));
  Serial.println(F("L<lo> <hi>: step levels [rad]"));
  Serial.println(F("P<s>: step half-period [s]"));
  Serial.println(F("H   : help"));
  Serial.println(F("JSON: {t,target,meas,err,vel} -> ROS2 bridge -> PlotJuggler"));
}

// Register the signals streamed as JSON (lib/Telemetry). Each new key appears as
// a /mini_ranka/<key> topic in ROS2 and a curve in PlotJuggler automatically.
static void registerTelemetry() {
  telemetry.add("target", &cmd_angle);   // commanded setpoint [rad]
  telemetry.add("meas",   &meas);        // measured angle [rad]
  telemetry.add("err",    &err);         // tracking error [rad]
  telemetry.add("vel",    &vel);         // shaft velocity [rad/s]
}

// --- Commander callbacks ---------------------------------------------------
void doMotor(char* cmd)  { command.motor(&motor, cmd); }
void doTarget(char* cmd) { command.scalar(&target, cmd); }
void doPeriod(char* cmd) { command.scalar(&step_half_period, cmd); }
void doHelp(char* /*cmd*/) { printHelp(); }

void doStepToggle(char* /*cmd*/) {
  step_on   = !step_on;
  step_t0   = millis();
  step_high = false;
  Serial.print(F("step: ")); Serial.println(step_on ? F("ON") : F("OFF"));
}

void doStepLevels(char* cmd) {
  // Parse two floats by hand. newlib-nano's sscanf(%f) is a silent no-op unless
  // the build links -u _scanf_float, so use strtod (the same path atof uses).
  char* end = nullptr;
  float lo = strtod(cmd, &end);
  if (end == cmd) { Serial.println(F("usage: L<lo> <hi>")); return; }
  float hi = strtod(end, &end);
  step_lo = constrain(lo, J_MIN, J_MAX);
  step_hi = constrain(hi, J_MIN, J_MAX);
  Serial.print(F("step levels: ")); Serial.print(step_lo, 3);
  Serial.print(F(" / ")); Serial.println(step_hi, 3);
}

void setup() {
  Serial.begin(TELEMETRY_BAUD);
  while (!Serial && millis() < 2000);
  Serial.setTimeout(10);   // bound serial reads so they never stall the FOC loop

  // default step around mid-range, ±0.3 rad, clamped to the joint limits
  float mid = 0.5f * (J_MIN + J_MAX);
  step_lo = constrain(mid - 0.3f, J_MIN, J_MAX);
  step_hi = constrain(mid + 0.3f, J_MIN, J_MAX);

  jWire.begin();
  jWire.setClock(I2C_CLOCK_HZ);
  sensor.init(&jWire);
  delay(50);

  driver.voltage_power_supply = SUPPLY_VOLTAGE;
  driver.init();

  motor.linkSensor(&sensor);
  motor.linkDriver(&driver);
  motor.voltage_limit            = VOLTAGE_LIMIT;
  motor.PID_velocity.P           = J_VEL_P;
  motor.PID_velocity.I           = J_VEL_I;
  motor.PID_velocity.output_ramp = J_OUT_RAMP;
  motor.P_angle.P                = J_ANG_P;
  motor.P_angle.I                = J_ANG_I;
  motor.P_angle.D                = J_ANG_D;
  motor.velocity_limit           = J_VEL_LIMIT;
  motor.LPF_velocity.Tf          = J_LPF;
  motor.zero_electric_angle      = J_ZERO;
  motor.sensor_direction         = Direction::CW;
  motor.init();
  motor.initFOC();
  motor.controller = MotionControlType::angle;

  command.add('M', doMotor,      "motor");
  command.add('G', doTarget,     "target [rad]");
  command.add('T', doStepToggle, "toggle step");
  command.add('L', doStepLevels, "step levels lo hi");
  command.add('P', doPeriod,     "step half-period [s]");
  command.add('H', doHelp,       "help");

  registerTelemetry();
  printHelp();
  Serial.println(F("ready"));
}

void loop() {
  sensor.update();
  motor.loopFOC();

  // Continuous step generator overrides the manual target while active.
  if (step_on) {
    if (millis() - step_t0 >= (unsigned long)(step_half_period * 1000.0f)) {
      step_high = !step_high;
      step_t0   = millis();
    }
    target = step_high ? step_hi : step_lo;
  }

  cmd_angle = constrain(target, J_MIN, J_MAX);
  motor.move(cmd_angle);
  command.run();

  // JSON telemetry -> ROS2 bridge -> PlotJuggler (throttled inside lib/Telemetry).
  meas = sensor.getAngle();
  vel  = motor.shaft_velocity;
  err  = cmd_angle - meas;
  telemetry.update(millis());
}
