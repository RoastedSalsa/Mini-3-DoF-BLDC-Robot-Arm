#include <Arduino.h>
#include <Wire.h>
#include <SimpleFOC.h>

// === Motor selection — comment out to disable ===
#define ENABLE_MOTOR1
// #define ENABLE_MOTOR2
// #define ENABLE_MOTOR3

// === I2C buses ===
TwoWire myWire1(PB7,  PA15);  // I2C1 HW — sensor 1 (SDA, SCL)
TwoWire myWire2(PC9,  PC8);   // I2C3 HW — sensor 2 (SDA, SCL)
TwoWire myWire3(PC7,  PC6);   // I2C4 HW — sensor 3 (SDA, SCL)

// === Sensors ===
MagneticSensorI2C sensor1 = MagneticSensorI2C(AS5600_I2C);
MagneticSensorI2C sensor2 = MagneticSensorI2C(AS5600_I2C);
MagneticSensorI2C sensor3 = MagneticSensorI2C(AS5600_I2C);

// === Motors ===
BLDCMotor motor1 = BLDCMotor(11);
BLDCMotor motor2 = BLDCMotor(11);
BLDCMotor motor3 = BLDCMotor(7);

// === Drivers ===
BLDCDriver3PWM driver1 = BLDCDriver3PWM(PA8, PA9,  PA10, PC5);   // TIM1
BLDCDriver3PWM driver2 = BLDCDriver3PWM(PA0, PA1,  PB10, PB12);  // TIM2
BLDCDriver3PWM driver3 = BLDCDriver3PWM(PA6, PB5,  PB0,  PB13);  // TIM3

// === Targets ===
float target1 = 0, target2 = 0, target3 = 0;

Commander command = Commander(Serial);
#ifdef ENABLE_MOTOR1
void onTarget1(char* cmd) { command.scalar(&target1, cmd); }
#endif
#ifdef ENABLE_MOTOR2
void onTarget2(char* cmd) { command.scalar(&target2, cmd); }
#endif
#ifdef ENABLE_MOTOR3
void onTarget3(char* cmd) { command.scalar(&target3, cmd); }
#endif

void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 2000);

#ifdef ENABLE_MOTOR1
    myWire1.begin();
    myWire1.setClock(400000);
    sensor1.init(&myWire1);
    delay(50);

    driver1.voltage_power_supply = 12.0f;
    driver1.init();

    motor1.linkSensor(&sensor1);
    motor1.linkDriver(&driver1);
    motor1.voltage_limit             = 12.0f;
    motor1.PID_velocity.P            = 0.3f;
    motor1.PID_velocity.I            = 0.1f;
    motor1.PID_velocity.output_ramp  = 1000.0f;
    motor1.P_angle.P                 = 10.0f;
    motor1.P_angle.I                 = 1.0f;
    motor1.P_angle.D                 = 0.05f;
    motor1.velocity_limit            = 100.0f;
    motor1.LPF_velocity.Tf           = 0.01f;
    //motor1.zero_electric_angle     = 6.002470f;
    //motor1.sensor_direction        = Direction::CW;
    motor1.init();
    motor1.initFOC();
    motor1.controller = MotionControlType::angle;
    command.add('1', onTarget1, "target motor1");
#endif

#ifdef ENABLE_MOTOR2
    myWire2.begin();
    myWire2.setClock(400000);
    sensor2.init(&myWire2);
    delay(50);

    driver2.voltage_power_supply = 12.0f;
    driver2.init();

    motor2.linkSensor(&sensor2);
    motor2.linkDriver(&driver2);
    motor2.voltage_limit             = 12.0f;
    motor2.PID_velocity.P            = 0.3f;
    motor2.PID_velocity.I            = 0.1f;
    motor2.PID_velocity.output_ramp  = 1000.0f;
    motor2.P_angle.P                 = 10.0f;
    motor2.P_angle.I                 = 1.0f;
    motor2.P_angle.D                 = 0.05f;
    motor2.velocity_limit            = 100.0f;
    motor2.LPF_velocity.Tf           = 0.01f;
    //motor2.zero_electric_angle     = 3.529690f;
    //motor2.sensor_direction        = Direction::CW;
    motor2.init();
    motor2.initFOC();
    motor2.controller = MotionControlType::angle;
    command.add('2', onTarget2, "target motor2");
#endif

#ifdef ENABLE_MOTOR3
    myWire3.begin();
    myWire3.setClock(400000);
    sensor3.init(&myWire3);
    delay(50);

    driver3.voltage_power_supply = 12.0f;
    driver3.init();

    motor3.linkSensor(&sensor3);
    motor3.linkDriver(&driver3);
    motor3.voltage_limit             = 12.0f;
    motor3.PID_velocity.P            = 0.5f;
    motor3.PID_velocity.I            = 0.1f;
    motor3.PID_velocity.output_ramp  = 1000.0f;
    motor3.P_angle.P                 = 10.0f;
    motor3.P_angle.I                 = 1.0f;
    motor3.P_angle.D                 = 0.05f;
    motor3.velocity_limit            = 100.0f;
    motor3.LPF_velocity.Tf           = 0.01f;
    //motor3.zero_electric_angle     = 2.212000f;
    //motor3.sensor_direction        = Direction::CW;
    motor3.init();
    motor3.initFOC();
    motor3.controller = MotionControlType::angle;
    command.add('3', onTarget3, "target motor3");
#endif

    Serial.println("Motors ready.");
}

void loop() {
#ifdef ENABLE_MOTOR1
    target1 = constrain(target1, -0.960f, 2.182f);  // soft stops: -55° to 125°
    sensor1.update(); motor1.loopFOC(); motor1.move(target1);
#endif

#ifdef ENABLE_MOTOR2
    sensor2.update(); motor2.loopFOC(); motor2.move(target2);
#endif

#ifdef ENABLE_MOTOR3
    sensor3.update(); motor3.loopFOC(); motor3.move(target3);
#endif

    static unsigned long lastPrint = 0;
    if (millis() - lastPrint > 200) {
        lastPrint = millis();
#ifdef ENABLE_MOTOR1
        Serial.print("m1: "); Serial.print(sensor1.getAngle(), 3);
#endif
#ifdef ENABLE_MOTOR2
        Serial.print("\tm2: "); Serial.print(sensor2.getAngle(), 3);
#endif
#ifdef ENABLE_MOTOR3
        Serial.print("\tm3: "); Serial.print(sensor3.getAngle(), 3);
#endif
        Serial.println();
    }

    command.run();
}
