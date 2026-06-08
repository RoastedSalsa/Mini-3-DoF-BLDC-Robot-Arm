#include <Arduino.h>
#include <Wire.h>
#include <SimpleFOC.h>
#include "config.h"
#include "ArmKinematics.h"

// === I2C / AS5600 ===
TwoWire myWire1(S1_SDA_PIN, S1_SCL_PIN);  // I2C1 HW — sensor 1 (SDA, SCL)
TwoWire myWire2(S2_SDA_PIN, S2_SCL_PIN);  // I2C3 HW — sensor 2 (SDA, SCL)
TwoWire myWire3(S3_SDA_PIN, S3_SCL_PIN);  // I2C4 HW — sensor 3 (SDA, SCL)
MagneticSensorI2C sensor1 = MagneticSensorI2C(AS5600_I2C);
MagneticSensorI2C sensor2 = MagneticSensorI2C(AS5600_I2C);
MagneticSensorI2C sensor3 = MagneticSensorI2C(AS5600_I2C);
// === Motors / Drivers ===
BLDCMotor motor1 = BLDCMotor(M1_POLE_PAIRS);
BLDCMotor motor2 = BLDCMotor(M2_POLE_PAIRS);
BLDCMotor motor3 = BLDCMotor(M3_POLE_PAIRS);
BLDCDriver3PWM driver1 = BLDCDriver3PWM(M1_PH_A, M1_PH_B,  M1_PH_C, M1_EN);   // TIM1
BLDCDriver3PWM driver2 = BLDCDriver3PWM(M2_PH_A, M2_PH_B,  M2_PH_C, M2_EN);  // TIM2
BLDCDriver3PWM driver3 = BLDCDriver3PWM(M3_PH_A, M3_PH_B,  M3_PH_C, M3_EN);  // TIM3

// === params ===
float voltage_power_supply = SUPPLY_VOLTAGE; // battery / PSU voltage
float voltage_limit = VOLTAGE_LIMIT;        // FOC voltage limit

// --- optional: tuning (you can change via Serial if desired) ---
float target_angle = 0;


bool started = false;
float zeroed1 = 0.0;
float zeroed2 = 0.0;
float zeroed3 = 0.0;

float theta1 = 0.0;
float theta2 = 0.0;
float theta3 = 0.0;

float th1 = 0.0;
float th2 = 0.0;
float th3 = 0.0;

// link lengths l1/l2/l3 and the IK temp r now live in config.h (L1/L2/L3) and
// ArmKinematics; see ik().
float x = 0.2;
float y = 0.0;
float z = 0.2488;

void trajectory();

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 2000);

  // init I2C custom bus
  myWire1.begin();
  myWire1.setClock(I2C_CLOCK_HZ); // try 100000 if issues
  myWire2.begin();
  myWire2.setClock(I2C_CLOCK_HZ); // try 100000 if issues
  myWire3.begin();
  myWire3.setClock(I2C_CLOCK_HZ); // try 100000 if issues

  // init sensor (use custom TwoWire)
  sensor1.init(&myWire1);
  delay(50);
  sensor2.init(&myWire2);
  delay(50);
  sensor3.init(&myWire3);
  delay(50);
  Serial.println("Wire done");
  // init driver and motor (same order as before)

  driver1.voltage_power_supply = voltage_power_supply;
  driver1.init();
  motor1.linkSensor(&sensor1);
  motor1.linkDriver(&driver1);
  motor1.voltage_limit = voltage_limit;
  motor1.PID_velocity.P = M1_VEL_P;
  motor1.PID_velocity.I = M1_VEL_I;
  motor1.P_angle.P = M1_ANGLE_P;
  motor1.velocity_limit = M1_VEL_LIMIT;
  motor1.LPF_velocity.Tf = M1_LPF_TF;
  motor1.PID_velocity.output_ramp = M1_OUTPUT_RAMP;
  motor1.init();
  motor1.zero_electric_angle = M1_ZERO_ELEC_ANGLE;
  motor1.sensor_direction = Direction::CW;    // or CCW
  delay(50);
  motor1.initFOC();
  Serial.println("Motor 1 initialized");

  driver2.voltage_power_supply = voltage_power_supply;
  driver2.init();
  motor2.linkSensor(&sensor2);
  motor2.linkDriver(&driver2);
  motor2.voltage_limit = voltage_limit;
  motor2.PID_velocity.P = M2_VEL_P;
  motor2.PID_velocity.I = M2_VEL_I;
  // optionally set angle PID P
  motor2.P_angle.P = M2_ANGLE_P;
  motor2.velocity_limit = M2_VEL_LIMIT;
  motor2.LPF_velocity.Tf = M2_LPF_TF;
  motor2.PID_velocity.output_ramp = M2_OUTPUT_RAMP;
  motor2.zero_electric_angle = M2_ZERO_ELEC_ANGLE;
  motor2.sensor_direction = Direction::CW;    // or CCW
  motor2.init();
  delay(50);
  motor2.initFOC();
  Serial.println("Motor 2 initialized");

  driver3.voltage_power_supply = voltage_power_supply;
  driver3.init();
  motor3.linkSensor(&sensor3);
  motor3.linkDriver(&driver3);
  motor3.voltage_limit = voltage_limit;
  motor3.PID_velocity.P = M3_VEL_P;
  motor3.PID_velocity.I = M3_VEL_I;
  motor3.P_angle.P = M3_ANGLE_P;
  motor3.velocity_limit = M3_VEL_LIMIT;
  motor3.LPF_velocity.Tf = M3_LPF_TF;
  motor3.PID_velocity.output_ramp = M3_OUTPUT_RAMP;
  motor3.init();
  motor3.sensor_direction = Direction::CW;    // or CCW
  motor3.zero_electric_angle = M3_ZERO_ELEC_ANGLE;
  // link sensor to motor
  motor3.initFOC(); // runs alignment using linked sensor
  // initialize FOC (this runs the alignment routine)
  Serial.println("Motor 3 initialized");

  //
  // select angle
  motor1.controller = MotionControlType::angle;
  delay(50);
  motor2.controller = MotionControlType::angle;
  motor3.controller = MotionControlType::angle;
}

void loop() {
  // update sensor (important)
  // 125, 347, 16.5
  sensor1.update();
  sensor2.update();
  sensor3.update();

  motor1.loopFOC();
  motor2.loopFOC();
  motor3.loopFOC();
  //Startup


  // --- Read desired position from Serial: "x y z"
  if (Serial.available()) {
      float nx, ny, nz;
      if (Serial.parseFloat() || Serial.peek() == '-') {
          nx = Serial.parseFloat();
          ny = Serial.parseFloat();
          nz = Serial.parseFloat();

          x = nx;
          y = ny;
          z = nz;

          Serial.print("New target (x,y,z): ");
          Serial.print(x); Serial.print(", ");
          Serial.print(y); Serial.print(", ");
          Serial.println(z);
      }
      while (Serial.available()) Serial.read(); // clear buffer
  }

  trajectory();

  if(!started)
    {
      Serial.println(sensor1.getAngle());
      if(sensor1.getAngle()>3.14 & sensor1.getAngle()<6.28)
        {
          zeroed1=HOMING_OFFSET_J1_UPPER;
        }
      else if(sensor1.getAngle()<=3.14 & sensor1.getAngle()>-3.14)
        {
          zeroed1=HOMING_OFFSET_J1_LOWER;
        }
      Serial.println(sensor1.getAngle());
      //second motor zeroing
      if(sensor2.getAngle()<1)
        {
          zeroed2=HOMING_OFFSET_J2_NEG;
        }
      else if(sensor2.getAngle()>1)
        {
          zeroed2=HOMING_OFFSET_J2_POS;
        }
        Serial.println(sensor1.getAngle());
      zeroed3=HOMING_OFFSET_J3;
      started=true;
      Serial.println(zeroed2);
    }
  // command the motor in angle-control mode

  ik(x, y, z, theta1, theta2, theta3);


  th1 = zeroed1 +theta1;
  th2 = zeroed2 +theta2*J2_GEAR_RATIO;
  th3 = zeroed3 +theta3;

  //Setting desired angles using inverse kinematics
  motor1.move(th1);
  motor2.move(th2);
  motor3.move(th3); // target in radians
  // debug print periodically
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 200) {
    lastPrint = millis();
    float angle_rad1 = sensor1.getAngle();
    float angle_rad2 = sensor2.getAngle();
    float angle_rad3 = sensor3.getAngle();
    Serial.print("angle_deg1: "); Serial.print(angle_rad1, 3);
    Serial.print("angle_deg2: "); Serial.print(angle_rad2, 3);
    Serial.print("angle_deg3: "); Serial.println(angle_rad3, 3);
    Serial.print("des_deg1: "); Serial.print(th1, 3);
    Serial.print("des_deg2: "); Serial.print(th2, 3);
    Serial.print("des_deg3: "); Serial.println(th3, 3);
  }

  //command.run();
}

// 4-point continuous trajectory
void trajectory() {
    // ---- EDIT YOUR 4 CARTESIAN WAYPOINTS HERE ----
    static float Px[4] = {0.2, 0.2, 0.2, 0.2};
    static float Py[4] = {0.1, 0.1, -0.1, -0.1};
    static float Pz[4] = {0.15, 0.25, 0.25, 0.15};

    // time for each segment (sec)
    const float segmentTime = 2.0;

    // interpolation tracking
    static int i = 0;                    // current segment index [0..3]
    static unsigned long t0 = millis();  // start of segment

    // --- compute interpolation alpha (0 -> 1) ---
    float dt = (millis() - t0) / 1000.0f;
    float a = dt / segmentTime;

    if (a >= 1.0f) {
        // move to next segment
        i = (i + 1) % 4;
        t0 = millis();
        a = 0.0f;
    }

    // indices for segment endpoints
    int i0 = i;
    int i1 = (i + 1) % 4;

    // --- linear interpolation of x,y,z ---
    x = Px[i0] + a * (Px[i1] - Px[i0]);
    y = Py[i0] + a * (Py[i1] - Py[i0]);
    z = Pz[i0] + a * (Pz[i1] - Pz[i0]);
}
