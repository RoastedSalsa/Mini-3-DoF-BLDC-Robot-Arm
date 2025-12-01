#include <Wire.h>

TwoWire myWire(PB9, PB8);  // use I2C1 on PB9 (SDA), PB8 (SCL)
#define AS5600_ADDR 0x36   // 7-bit I2C address of AS5600

void setup() {
  Serial.begin(115200);
  myWire.begin();
  delay(100);

  Serial.println("AS5600 test start");
}

void loop() {
  uint16_t angle = readAS5600RawAngle();
  float degrees = (angle * 360.0) / 4096.0;  // 12-bit resolution

  Serial.print("Raw angle: ");
  Serial.print(angle);
  Serial.print("\tDegrees: ");
  Serial.println(degrees, 2);

  delay(2);
}

uint16_t readAS5600RawAngle() {
  myWire.beginTransmission(AS5600_ADDR);
  myWire.write(0x0C);  // RAW_ANGLE high byte register
  myWire.endTransmission(false);
  myWire.requestFrom(AS5600_ADDR, (uint8_t)2);

  if (myWire.available() == 2) {
    uint8_t high = myWire.read();
    uint8_t low  = myWire.read();
    return ((high & 0x0F) << 8) | low;
  } else {
    return 0xFFFF; // error value
  }
}
