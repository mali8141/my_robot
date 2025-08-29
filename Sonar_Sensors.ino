#include <Wire.h>

byte sensorAddresses[] = {0xE6, 0xE0, 0xFA, 0xF8, 0xE2, 0xE8};

void setup() {
  Wire.begin();
  Serial.begin(9600);
}

void loop() {
  for (int i = 0; i < 5; i++) {
    byte addr = sensorAddresses[i] >> 1;  // 7-bit address for Wire

    // Trigger ranging in cm (0x51)
    Wire.beginTransmission(addr);
    Wire.write(0x00);  // Command register
    Wire.write(0x51);  // Ranging command (in cm)
    Wire.endTransmission();

    delay(100);  // Wait for sensor to complete measurement

    // Read result from register 0x02 (2 bytes)
    Wire.beginTransmission(addr);
    Wire.write(0x02);
    Wire.endTransmission();

    Wire.requestFrom(addr, 2);
    if (Wire.available() == 2) {
      int highByte = Wire.read();
      int lowByte = Wire.read();
      int range = (highByte << 8) | lowByte;

      Serial.print("Sensor ");
      Serial.print(i + 1);
      Serial.print(" (0x");
      Serial.print(sensorAddresses[i], HEX);
      Serial.print("): ");
      Serial.print(range);
      Serial.println(" cm");
    }
  }

  Serial.println("--------");
  delay(300);  // Slow down the loop
}
