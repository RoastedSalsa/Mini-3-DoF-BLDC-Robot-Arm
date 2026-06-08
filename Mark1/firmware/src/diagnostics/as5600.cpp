#include <Arduino.h>
#include <Wire.h>
#include "config.h"

#define AS5600_ADDR 0x36

TwoWire myWire1(S1_SDA_PIN, S1_SCL_PIN);  // I2C1 HW — sensor 1 (SDA, SCL)
TwoWire myWire2(S2_SDA_PIN, S2_SCL_PIN);  // I2C3 HW — sensor 2 (SDA, SCL) (Ground and 3.3V cables colours are mixed up >.<)
TwoWire myWire3(S3_SDA_PIN, S3_SCL_PIN);  // I2C4 HW — sensor 3 (SDA, SCL)

uint16_t readAngle(TwoWire &wire) {
    wire.beginTransmission(AS5600_ADDR);
    wire.write(0x0C);  // RAW_ANGLE high byte
    wire.endTransmission(false);
    wire.requestFrom(AS5600_ADDR, (uint8_t)2);

    if (wire.available() == 2) {
        uint8_t high = wire.read();
        uint8_t low  = wire.read();
        return ((high & 0x0F) << 8) | low;
    }
    return 0xFFFF;
}

void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 2000);

    myWire1.begin();
    myWire2.begin();
    myWire3.begin();

    Serial.println("AS5600 3-sensor test");
}

void loop() {
    uint16_t r1 = readAngle(myWire1);
    uint16_t r2 = readAngle(myWire2);
    uint16_t r3 = readAngle(myWire3);

    Serial.print("S1: "); Serial.print(r1 * 360.0f / 4096.0f, 2);
    Serial.print("\tS2: "); Serial.print(r2 * 360.0f / 4096.0f, 2);
    Serial.print("\tS3: "); Serial.println(r3 * 360.0f / 4096.0f, 2);

    delay(10);
}
