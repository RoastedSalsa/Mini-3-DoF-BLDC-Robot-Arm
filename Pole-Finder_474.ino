#include <SimpleFOC.h>

// === Pinout for NUCLEO-G474RE + SimpleFOC Mini ===
#define IN1 PA8
#define IN2 PA9
#define IN3 PA10
#define EN  PA6

// Dummy motor (pole pairs = 1 for open-loop control)
BLDCMotor motor = BLDCMotor(1);
BLDCDriver3PWM driver = BLDCDriver3PWM(IN1, IN2, IN3, EN);

// how many electrical revolutions to run
const int electrical_revs = 50;     
const float voltage_limit = 3.0;    // volts (safe)
const float supply_voltage = 12.0;  // volts
const int delay_per_step = 3;       // ms per step
const int steps_per_rev = 200;      // resolution

void setup() {
  Serial.begin(115200);
  while(!Serial);

  Serial.println("=== POLE PAIR FINDER ===");
  Serial.println("Motor will turn slowly. Count mechanical turns!");

  driver.voltage_power_supply = supply_voltage;
  driver.init();
  driver.enable();
  motor.linkDriver(&driver);
  motor.voltage_limit = voltage_limit;
  motor.controller = MotionControlType::angle_openloop;
  motor.init();

  delay(1000);
  Serial.println("Starting...");
}

void loop() {
  // sweep through several electrical revolutions
  for (int rev = 0; rev < electrical_revs; rev++) {
    for (int i = 0; i < steps_per_rev; i++) {
      float angle = _2PI * (rev + (float)i / steps_per_rev);
      motor.move(angle);
      delay(delay_per_step);
    }
    Serial.print("Electrical rev ");
    Serial.println(rev + 1);
  }

  Serial.println("Done. Measure mechanical rotations.");
  Serial.println("Pole pairs = electrical revs / mechanical revs.");
  while (1);
}
