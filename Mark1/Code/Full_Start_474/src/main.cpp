#include <Arduino.h>
#include <Wire.h>
#include <SimpleFOC.h>

// === I2C / AS5600 ===
TwoWire myWire1(PC7, PC6);               // SDA, SCL (your custom bus)
TwoWire myWire2(PB9, PB8);
TwoWire myWire3(PC9, PC8);
MagneticSensorI2C sensor1 = MagneticSensorI2C(AS5600_I2C);
MagneticSensorI2C sensor2 = MagneticSensorI2C(AS5600_I2C);
MagneticSensorI2C sensor3 = MagneticSensorI2C(AS5600_I2C);
// === Motor / Driver (keep your original) ===
BLDCMotor motor3 = BLDCMotor(7);         // 11 pole pairs for GM3506
BLDCMotor motor2 = BLDCMotor(11);
BLDCMotor motor1 = BLDCMotor(11);
BLDCDriver3PWM driver1 = BLDCDriver3PWM(PB10, PB4, PB5, PA10);
BLDCDriver3PWM driver2 = BLDCDriver3PWM(PA0, PA1, PC0, PB0);
BLDCDriver3PWM driver3 = BLDCDriver3PWM(PC3, PC2, PC12, PC10);

// === params ===
float voltage_power_supply = 12.0; // battery / PSU voltage
float voltage_limit = 12.0;        // FOC voltage limit

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

float l1 = 0.0488;
float l2 = 0.2;
float l3 = 0.2;

float x = 0.2;
float y = 0.0;
float z = 0.2488;
float r = 0.0;

void trajectory();

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 2000);

  // init I2C custom bus
  myWire1.begin();
  myWire1.setClock(400000); // try 100000 if issues
  myWire2.begin();
  myWire2.setClock(400000); // try 100000 if issues
  myWire3.begin();
  myWire3.setClock(400000); // try 100000 if issues

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
  motor1.PID_velocity.P = 0.5;
  motor1.PID_velocity.I = 0.1;
  motor1.P_angle.P = 20;
  motor1.velocity_limit = 10;
  motor1.LPF_velocity.Tf = 0.01;
  motor1.PID_velocity.output_ramp = 1000;
  motor1.init();
  motor1.zero_electric_angle = 6.002470;
  motor1.sensor_direction = Direction::CW;    // or CCW
  delay(50);
  motor1.initFOC();
  Serial.println("Motor 1 initialized");

  driver2.voltage_power_supply = voltage_power_supply;
  driver2.init();
  motor2.linkSensor(&sensor2);
  motor2.linkDriver(&driver2);
  motor2.voltage_limit = voltage_limit;
  motor2.PID_velocity.P = 0.3;
  motor2.PID_velocity.I = 0.1;
  // optionally set angle PID P
  motor2.P_angle.P = 20;
  motor2.velocity_limit = 10;
  motor2.LPF_velocity.Tf = 0.01;
  motor2.PID_velocity.output_ramp = 1000;
  motor2.zero_electric_angle = 3.529690;
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
  motor3.PID_velocity.P = 0.5;
  motor3.PID_velocity.I = 0.1;
  motor3.P_angle.P = 20;
  motor3.velocity_limit = 10;
  motor3.LPF_velocity.Tf = 0.01;
  motor3.PID_velocity.output_ramp = 1000;
  motor3.init();
  motor3.sensor_direction = Direction::CW;    // or CCW
  motor3.zero_electric_angle = 2.21200;
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
          zeroed1=6.487;
        }
      else if(sensor1.getAngle()<=3.14 & sensor1.getAngle()>-3.14)
        {
          zeroed1=0.172;
        }
      Serial.println(sensor1.getAngle());
      //second motor zeroing
      if(sensor2.getAngle()<1)
        {
          zeroed2=-6.7;
        }
      else if(sensor2.getAngle()>1)
        {
          zeroed2=4.34;
        }
        Serial.println(sensor1.getAngle());
      zeroed3=3.5;
      started=true;
      Serial.println(zeroed2);
    }
  // command the motor in angle-control mode

  r = sqrt(pow(x,2) + pow(y,2));
  theta1 =  atan2(x,y) - PI/2;
  theta3 =  atan2(-sqrt(1-pow(((pow(r,2)+pow((-z+l1),2) -pow(l2,2) - pow(l3,2))/ (2 * l2 * l3)),2) ), (pow(r,2) + pow((-z+l1),2) -pow(l2,2) -pow(l3,2))/(2 * l2 * l3));
  theta2 =  -( atan2(-z + l1, r ) + atan2(l3*sin(theta3), l2 + l3*cos(theta3)));


  th1 = zeroed1 +theta1;
  th2 = zeroed2 +theta2*3;
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
