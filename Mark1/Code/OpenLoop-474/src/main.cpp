#include <Arduino.h>
#include <SimpleFOC.h>

// === Select motor: 1, 2, or 3 ===
#define MOTOR_SELECT 3

// === Motors ===
BLDCMotor motor1 = BLDCMotor(11);
BLDCMotor motor2 = BLDCMotor(11);
BLDCMotor motor3 = BLDCMotor(7);

// === Drivers ===
BLDCDriver3PWM driver1 = BLDCDriver3PWM(PA8, PA9,  PA10, PC5);   // TIM1
BLDCDriver3PWM driver2 = BLDCDriver3PWM(PA0, PA1,  PB10, PB12);  // TIM2
BLDCDriver3PWM driver3 = BLDCDriver3PWM(PA6, PB5,  PB0,  PB13);  // PA6=TIM3_CH1, PB5=TIM3_CH2, PB6=TIM4_CH1 (PB0 unusable: only TIM1_CH2N / TIM3_CH3-via-ALT, both fail in SimpleFOC)

#if   MOTOR_SELECT == 1
  #define MOTOR  motor1
  #define DRIVER driver1
#elif MOTOR_SELECT == 2
  #define MOTOR  motor2
  #define DRIVER driver2
#elif MOTOR_SELECT == 3
  #define MOTOR  motor3
  #define DRIVER driver3
#endif

void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 2000);

    DRIVER.voltage_power_supply = 12.0f;
    DRIVER.init();

    MOTOR.linkDriver(&DRIVER);
    MOTOR.voltage_limit = 6.0f;
    MOTOR.init();

    MOTOR.controller = MotionControlType::velocity_openloop;

    Serial.print("Open loop — motor "); Serial.println(MOTOR_SELECT);
}

void loop() {
    MOTOR.move(1.0f); // rad/s — adjust as needed
}
