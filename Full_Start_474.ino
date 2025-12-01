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
Commander command = Commander(Serial);
void onTarget(char* cmd){ command.scalar(&target_angle, cmd); }
bool started = false;
float zeroed1 = 0;
float zeroed2 = 0;
float zeroed3 = 0;

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
  //Serial.println(motor3.sensor_direction);
  //Serial.println(motor3.zero_electric_angle,6);

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
  
  if(!started)
    {
      Serial.println(sensor1.getAngle());
      if(sensor1.getAngle()>3.14 & sensor1.getAngle()<6.28)
        {
          zeroed1=350.0/180.0*3.141;
        }
      else if(sensor1.getAngle()<=3.14 & sensor1.getAngle()>-3.14)
        {
          zeroed1=-10.0/180.0*3.141;
        }
      Serial.println(sensor1.getAngle());
      //second motor zeroing
      if(sensor2.getAngle()<1)
        {
          zeroed2=-1.94;
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
  motor1.move(zeroed1);
  motor2.move(zeroed2);
  motor3.move(zeroed3); // target in radians
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

    
    //Serial.print("\tvel: "); Serial.print(sensor.getVelocity(), 3);
   // Serial.print("\tVlim: "); Serial.println(motor.voltage_limit, 2);
  }

  //command.run();
}
