// full.cpp — main firmware for the Mini 3-DoF BLDC arm.
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
// (src/tools/pid_tuner.cpp); see docs/tuning-guide.md.

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
float th1 = 0, th2 = 0, th3 = 0;          // motor-frame targets [rad]

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

// Operator input over serial: three floats "x y z" set a manual Cartesian target
// (and pause the trajectory); a leading 'g' resumes the continuous trajectory.
static void handleSerial() {
  if (!Serial.available()) return;

  char buf[48];
  size_t n = Serial.readBytesUntil('\n', buf, sizeof(buf) - 1);
  buf[n] = '\0';

  if (buf[0] == 'g' || buf[0] == 'G') {
    traj_enabled = true;
    Serial.println(F("Trajectory resumed"));
    return;
  }

  float nx, ny, nz;
  if (sscanf(buf, "%f %f %f", &nx, &ny, &nz) == 3) {
    x = nx; y = ny; z = nz;
    traj_enabled = false;
    Serial.print(F("Manual target (x,y,z): "));
    Serial.print(x, 3); Serial.print(F(", "));
    Serial.print(y, 3); Serial.print(F(", "));
    Serial.println(z, 3);
  }
}

// ================================= Arduino ===================================

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 2000);
  Serial.setTimeout(10);   // bound serial reads so they never stall the FOC loop

  configureJoint("Motor 1", myWire1, sensor1, driver1, motor1, GAINS1);
  configureJoint("Motor 2", myWire2, sensor2, driver2, motor2, GAINS2);
  configureJoint("Motor 3", myWire3, sensor3, driver3, motor3, GAINS3);

  trajectory.begin(millis());
  telemetry.begin();
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

  // 2. Operator input, then advance the trajectory (unless a manual hold).
  handleSerial();
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

  // 5. Stream telemetry (CSV, throttled internally).
  telemetry.update(now, x, y, z, th1, th2, th3,
                   sensor1.getAngle(), sensor2.getAngle(), sensor3.getAngle());
}
