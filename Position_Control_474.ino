#include <Wire.h>
#include <SimpleFOC.h>

// === I2C / AS5600 ===
TwoWire myWire2(PB9, PB8);                // SDA, SCL (your custom bus)
MagneticSensorI2C sensor2 = MagneticSensorI2C(AS5600_I2C);

// === Motor / Driver (keep your original) ===
BLDCMotor motor2 = BLDCMotor(11);         // 11 pole pairs for GM3506
//BLDCDriver3PWM driver = BLDCDriver3PWM(PA8, PA9, PA10, PA6); // EN=PA6
BLDCDriver3PWM driver2 = BLDCDriver3PWM(PA0, PA1, PC0, PB0);

// === params ===
float voltage_power_supply = 12.0; // battery / PSU voltage
float voltage_limit = 12.0;        // FOC voltage limit

// --- optional: tuning (you can change via Serial if desired) ---
float target_angle = 0;
Commander command = Commander(Serial);
void onTarget(char* cmd){ command.scalar(&target_angle, cmd); }

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 2000);

  // init I2C custom bus
  myWire2.begin();
  myWire2.setClock(400000); // try 100000 if issues

  // init sensor (use custom TwoWire)
  sensor2.init(&myWire2);
  delay(50);

  // init driver and motor (same order as before)
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

  // link sensor to motor
 

  // initialize FOC (this runs the alignment routine)
  Serial.println("Calling motor.initFOC() ...");
  motor2.initFOC(); // runs alignment using linked sensor
  Serial.println("initFOC done.");

  // select angle controller
  motor2.controller = MotionControlType::angle;
  Serial.print("zero_electric_angle = ");
  Serial.println(motor2.zero_electric_angle, 6);
  Serial.print("sensor_direction = ");
  Serial.println(motor2.sensor_direction);
  command.add('T', onTarget, "target angle");


}

void loop() {
  // update sensor (important)
  sensor2.update();
  motor2.loopFOC();
  // command the motor in angle-control mode
  motor2.move(200.0/180.0*3.141); // target in radians

  // debug print periodically
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 200) {
    lastPrint = millis();
    float angle_rad = sensor2.getAngle();
    Serial.print("angle_deg: "); Serial.print(angle_rad * 180.0 / M_PI, 2);
    Serial.print("\tTdeg: "); Serial.print(target_angle * 180.0 / M_PI, 2);
    Serial.print("\tvel: "); Serial.print(sensor2.getVelocity(), 3);
    Serial.print("\tVlim: "); Serial.println(motor2.voltage_limit, 2);
  }

  command.run();
}
