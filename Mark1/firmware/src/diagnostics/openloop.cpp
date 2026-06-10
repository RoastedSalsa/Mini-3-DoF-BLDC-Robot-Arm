#include <Arduino.h>
#include <SimpleFOC.h>
#include "config.h"

// === Select motor: 1, 2, or 3 ===
#define MOTOR_SELECT 3

// === Motors ===
BLDCMotor motor1 = BLDCMotor(M1_POLE_PAIRS);
BLDCMotor motor2 = BLDCMotor(M2_POLE_PAIRS);
BLDCMotor motor3 = BLDCMotor(M3_POLE_PAIRS);

// === Drivers ===
BLDCDriver3PWM driver1 = BLDCDriver3PWM(M1_PH_A, M1_PH_B,  M1_PH_C, M1_EN);   // TIM1
BLDCDriver3PWM driver2 = BLDCDriver3PWM(M2_PH_A, M2_PH_B,  M2_PH_C, M2_EN);  // TIM2
BLDCDriver3PWM driver3 = BLDCDriver3PWM(M3_PH_A, M3_PH_B,  M3_PH_C, M3_EN);  // TIM3

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

    DRIVER.voltage_power_supply = SUPPLY_VOLTAGE;
    DRIVER.init();

    MOTOR.linkDriver(&DRIVER);
    MOTOR.voltage_limit = 6.0f;
    MOTOR.init();

    MOTOR.controller = MotionControlType::velocity_openloop;

    Serial.print("Open loop — motor "); Serial.println(MOTOR_SELECT);
}

void loop() {
    MOTOR.loopFOC(); //Loopfoc is needed in openloop from SimpleFOC v2.4.0
    MOTOR.move(1.0f); // rad/s — adjust as needed
}
