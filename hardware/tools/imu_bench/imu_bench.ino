// Standalone USB bench test for an MPU-6050 type breakout at 3.3 V.
// Connect SDA to Uno A4, SCL to A5, and keep ESC power disconnected.
#include <Wire.h>

constexpr uint8_t whoAmI = 0x75;
constexpr uint8_t powerManagement = 0x6B;
constexpr uint8_t accelStart = 0x3B;
constexpr uint8_t gyroConfig = 0x1B;
constexpr uint8_t accelConfig = 0x1C;
uint8_t sensorAddress = 0;

bool readRegisters(uint8_t address, uint8_t first, uint8_t *data, uint8_t count) {
  Wire.beginTransmission(address);
  Wire.write(first);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(address, count) != count) return false;
  for (uint8_t i = 0; i < count; ++i) data[i] = Wire.read();
  return true;
}

bool writeRegister(uint8_t address, uint8_t reg, uint8_t value) {
  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

void findSensor() {
  for (uint8_t address = 0x68; address <= 0x69; ++address) {
    uint8_t identity;
    if (!readRegisters(address, whoAmI, &identity, 1)) continue;
    Serial.print(F("Address=0x"));
    Serial.print(address, HEX);
    Serial.print(F(", WHO_AM_I=0x"));
    Serial.println(identity, HEX);
    if (identity != 0x68) continue;
    if (writeRegister(address, powerManagement, 0x01) &&
        writeRegister(address, gyroConfig, 0x00) &&
        writeRegister(address, accelConfig, 0x00)) {
      sensorAddress = address;
      delay(100);
      Serial.println(F("Accelerometer [g], gyroscope [deg/s]"));
      Serial.println(F("time_ms,accel_x,accel_y,accel_z,gyro_x,gyro_y,gyro_z"));
      return;
    }
  }
  Serial.println(F("No MPU-6050 stream. Check power, SDA/SCL, and I2C pull-ups."));
}

int16_t readWord(const uint8_t *bytes) {
  return (int16_t)(((uint16_t)bytes[0] << 8) | bytes[1]);
}

void setup() {
  // Wire normally enables the Uno's internal pull-ups to 5 V.
  // Disable them globally before Wire starts. Use the breakout's
  // pull-ups to its logic supply or an I2C level shifter.
  MCUCR |= _BV(PUD);
  Wire.begin();
  Wire.setClock(100000);
  Wire.setWireTimeout(3000, true);
  Serial.begin(115200);
  Serial.println(F("MPU-6050 bench test"));
}

void loop() {
  if (!sensorAddress) {
    findSensor();
    if (!sensorAddress) {
      delay(1000);
      return;
    }
  }
  uint8_t data[14];
  if (!readRegisters(sensorAddress, accelStart, data, sizeof(data))) {
    Serial.println(F("Motion read failed. Rechecking I2C."));
    sensorAddress = 0;
    delay(300);
    return;
  }
  Serial.print(millis());
  for (uint8_t axis = 0; axis < 3; ++axis) {
    Serial.print(',');
    Serial.print(readWord(data + 2 * axis) / 16384.0f, 4);
  }
  for (uint8_t axis = 0; axis < 3; ++axis) {
    Serial.print(',');
    Serial.print(readWord(data + 8 + 2 * axis) / 131.0f, 3);
  }
  Serial.println();
  delay(100);
}
