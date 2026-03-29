#include <SimpleFOC.h>

// Motor instance
BLDCMotor motor = BLDCMotor(11);    // 11 pole pairs for GM3506
BLDCDriver3PWM driver = BLDCDriver3PWM(PA8, PA9, PA10, PA6); // EN=PA6

void setup() {
  Serial.begin(115200);
  while (!Serial);

  driver.voltage_power_supply = 12;
  driver.init();
  motor.linkDriver(&driver);
  motor.voltage_limit = 6;
  motor.init();
  Serial.println("Open loop test");

  // Start open-loop velocity mode
  motor.controller = MotionControlType::velocity_openloop;
}

void loop() {
  // Spin slowly in open loop
  motor.move(0); // rad/s
}
