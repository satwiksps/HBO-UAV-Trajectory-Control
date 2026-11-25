// Uno + MPU-6050 (A4/A5) + SimonK ESC signal (D6).
#include <Arduino.h>
#include <Wire.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <util/atomic.h>
#include <math.h>

constexpr uint16_t IDLE = 950;
constexpr uint16_t LOW_DRIVE = 1100;
constexpr uint16_t HARD_MAX = 1300;
constexpr uint32_t SENSOR_US = 10000, PID_US = 5000, FRAME_US = 20000;
constexpr uint32_t LINK_MS = 750, RUN_MS = 180000;
constexpr uint32_t LAUNCH_MS = 15000;
constexpr float LAUNCH_STEP = 12.0f * PID_US * 0.000001f;
constexpr float LAUNCH_LEAD = 8.0f;
constexpr float LAUNCH_KP = 0.55f, LAUNCH_KD = 0.25f;
constexpr float APPROACH_BRAKE_GAIN = 0.70f;
constexpr float LAUNCH_CAPTURE_ERROR = 3.0f, LAUNCH_CAPTURE_RATE = 5.0f;
constexpr float LAUNCH_BOOST_MAX = 8.0f;
constexpr float LAUNCH_BOOST_STEP = 1.2f * PID_US * 0.000001f;
constexpr float LAUNCH_BOOST_RELEASE = 10.0f * PID_US * 0.000001f;
constexpr uint32_t DIP_MS = 2000, DIP_GAP_MS = 15000;
constexpr int8_t DIP_US = -4;
enum Mode : uint8_t { IDLE_MODE, PID_MODE, FIXED };
enum Fault : uint8_t { OK, LINK_LOST, IMU_LOST, TILT_INVALID, UPPER_LIMIT, OUT_OF_RANGE, RUN_LIMIT, OVERSPEED,
                       LAUNCH_TIMEOUT };

volatile uint16_t pulseTicks = IDLE * 2;
uint16_t pulse = IDLE, manualPulse = IDLE, hover = 1149, cap = HARD_MAX;
Mode mode = IDLE_MODE;
Fault fault = OK;
bool armed = false, calibrated = false, haveAngle = false, sensorOK = false, gravityOK = false;
bool launching = false;
uint8_t imuAddress = 0;
uint32_t lastPacket = 0, started = 0, lastPulse = 0, topSince = 0;
uint32_t nextSensor = 0, nextPid = 0, nextFrame = 0, lastImu = 0, lastGravity = 0, lastProbe = 0, sequence = 0;
float zeroAxis[3], tangent[3], gyroAxis[3], bias[3];
float lower = 0, upper = 0, angle = 0, accelAngle = 0, rate = 0, target = 0, integral = 0;
float launchReference = 0, launchBoost = 0;
float kp = 0.55f, ki = 0.13f, kd = 0.25f;
float pTerm = 0, iTerm = 0, dTerm = 0;
int8_t appliedDip = 0;
uint32_t dipStart = 0;
bool dipScheduled = false;
int16_t raw[6] = {};
char rx[122], tx[150];
uint8_t rxSize = 0, txPos = 0, txSize = 0;
bool rxOverflow = false;
char ackKey[8];
bool ackOK[8];
uint8_t ackRead = 0, ackWrite = 0;

ISR(TIMER1_COMPA_vect) {
  OCR1B = pulseTicks;
  PORTD |= _BV(PD6);
}
ISR(TIMER1_COMPB_vect) { PORTD &= ~_BV(PD6); }

void setPulse(uint16_t value) {
  pulse = value < LOW_DRIVE ? IDLE : min(value, min(cap, HARD_MAX));
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { pulseTicks = pulse * 2; }
}

void startEsc() {
  digitalWrite(6, LOW);
  pinMode(6, OUTPUT);
  TCCR1A = TCCR1B = 0;
  TCNT1 = 0;
  OCR1A = 39999;
  OCR1B = IDLE * 2;
  TIFR1 = _BV(OCF1A) | _BV(OCF1B);
  TIMSK1 = _BV(OCIE1A) | _BV(OCIE1B);
  TCCR1B = _BV(WGM12) | _BV(CS11);
}

void stopDrive() {
  mode = IDLE_MODE;
  topSince = 0;
  integral = 0;
  launching = false;
  launchBoost = 0;
  dipScheduled = false;
  appliedDip = 0;
  pTerm = iTerm = dTerm = 0;
  setPulse(IDLE);
}

void trip(Fault reason) {
  stopDrive();
  armed = false;
  fault = reason;
}

bool readRegs(uint8_t address, uint8_t reg, uint8_t *out, uint8_t count) {
  Wire.beginTransmission(address);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0 || Wire.requestFrom(address, count) != count) return false;
  for (uint8_t i = 0; i < count; ++i) out[i] = Wire.read();
  return true;
}
bool writeReg(uint8_t address, uint8_t reg, uint8_t val) {
  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}
void probeImu() {
  for (uint8_t a = 0x68; a <= 0x69; ++a) {
    uint8_t id;
    if (readRegs(a, 0x75, &id, 1) && id == 0x68 &&
        writeReg(a, 0x6B, 0x01) && writeReg(a, 0x1A, 0x03) &&
        writeReg(a, 0x19, 0x09) && writeReg(a, 0x1B, 0x00) && writeReg(a, 0x1C, 0x00)) {
      imuAddress = a;
      return;
    }
  }
}
float dot(const float *x, const float *y) {
  return x[0]*y[0] + x[1]*y[1] + x[2]*y[2];
}
float wrapAngle(float value) {
  if (value > 180.0f) return value - 360.0f;
  if (value < -180.0f) return value + 360.0f;
  return value;
}
int16_t wordAt(const uint8_t *data) { return (int16_t)(((uint16_t)data[0] << 8) | data[1]); }

void readImu(uint32_t now) {
  uint8_t bytes[14];
  if (!imuAddress || !readRegs(imuAddress, 0x3B, bytes, sizeof(bytes))) {
    if (sensorOK && now - lastImu > 50000UL) { sensorOK = false; imuAddress = 0; haveAngle = false; }
    if (imuAddress && !sensorOK && now - lastImu > 200000UL) imuAddress = 0;
    return;
  }
  float dt = constrain((now - lastImu) * 0.000001f, 0.005f, 0.035f);
  for (uint8_t i = 0; i < 3; ++i) {
    raw[i] = wordAt(bytes + 2*i);
    raw[i+3] = wordAt(bytes + 8 + 2*i);
  }
  sensorOK = true;
  lastImu = now;
  float acc[3] = {raw[0]/16384.0f, raw[1]/16384.0f, raw[2]/16384.0f};
  float norm2 = dot(acc, acc);
  gravityOK = norm2 >= 0.72f*0.72f && norm2 <= 1.28f*1.28f;
  if (!calibrated) return;
  float px = dot(acc, zeroAxis), py = dot(acc, tangent);
  gravityOK = gravityOK && px*px + py*py > 0.24f;
  float gyro[3] = {(raw[3]-bias[0])/131.0f, (raw[4]-bias[1])/131.0f,
                   (raw[5]-bias[2])/131.0f};
  float r = dot(gyro, gyroAxis);
  rate += 0.22f * (r - rate);
  if (gravityOK) {
    accelAngle = atan2f(py, px) * 57.2957795f;
    lastGravity = now;
    if (!haveAngle) { angle = accelAngle; rate = r; haveAngle = true; }
  }
  if (haveAngle) {
    angle = wrapAngle(angle + r*dt);
    if (gravityOK) angle = wrapAngle(angle + (dt/(0.45f+dt))*wrapAngle(accelAngle-angle));
  }
}

bool healthy(uint32_t now) {
  return sensorOK && haveAngle && now - lastImu <= 50000UL && now - lastGravity <= 350000UL;
}
bool targetOK(float value) { return calibrated && value >= lower+3 && value <= upper-4; }

void control(uint32_t now, uint32_t ms) {
  if (!armed) return;
  if (ms - lastPacket >= LINK_MS) { trip(LINK_LOST); return; }
  if (!sensorOK || now - lastImu > 50000UL) { trip(IMU_LOST); return; }
  if (!healthy(now)) { trip(TILT_INVALID); return; }
  if (mode == IDLE_MODE) return;
  if (ms - started >= RUN_MS) { trip(RUN_LIMIT); return; }
  if (angle > upper+0.5f) { trip(UPPER_LIMIT); return; }
  if (angle < lower-6) { trip(OUT_OF_RANGE); return; }
  if (fabsf(rate) > 160.0f) { trip(OVERSPEED); return; }
  // Cut drive on a fast approach to the ceiling, allowing for ESC and motor lag.
  if (rate > 0 && angle >= upper-4 && angle + 0.14f*rate >= upper+0.5f) {
    trip(UPPER_LIMIT); return;
  }
  if (angle >= upper-1.5f) {
    if (!topSince) topSince = ms;
    if (ms - topSince >= 120) { trip(UPPER_LIMIT); return; }
  } else topSince = 0;

  uint16_t requested = manualPulse;
  if (mode == PID_MODE) {
    appliedDip = dipScheduled && ms-dipStart < DIP_MS ? DIP_US : 0;
    float e = target-angle;
    // Keep the same approach brake through launch, capture and hold.
    const float approachRate = 2.0f + 0.65f*max(e, 0.0f);
    const float approachBrake = -APPROACH_BRAKE_GAIN*max(rate-approachRate, 0.0f);
    if (launching) {
      // Leave the lower stop with a nearby reference and start integrating at capture.
      float goal = constrain(target, angle-LAUNCH_LEAD, angle+LAUNCH_LEAD);
      launchReference += constrain(goal-launchReference, -LAUNCH_STEP, LAUNCH_STEP);
      float launchError = constrain(launchReference-angle, -LAUNCH_LEAD, LAUNCH_LEAD);
      if (fabsf(launchReference-target) < 0.1f && fabsf(e) <= LAUNCH_CAPTURE_ERROR &&
          fabsf(rate) <= LAUNCH_CAPTURE_RATE) {
        // Match the launch output before switching to the selected hold gains.
        if (ki > 0)
          integral = constrain(launchBoost + LAUNCH_KP*launchError - kp*e + (kd-LAUNCH_KD)*rate,
                               -8.0f, 8.0f);
        launching = false;
      } else {
        if (ms-started >= LAUNCH_MS) { trip(LAUNCH_TIMEOUT); return; }
        // Compensate for a stall without winding up on the full starting angle error.
        if (e > LAUNCH_CAPTURE_ERROR && launchError >= LAUNCH_CAPTURE_ERROR &&
            fabsf(rate) <= 4.0f && pulse >= hover)
          launchBoost = min(LAUNCH_BOOST_MAX, launchBoost + LAUNCH_BOOST_STEP);
        else if (rate > LAUNCH_CAPTURE_RATE || launchError <= 0)
          launchBoost = max(0.0f, launchBoost - LAUNCH_BOOST_RELEASE);
        e = launchError;
      }
    }
    if (!launching) {
      float cand = constrain(integral + ki*e*0.005f, -8.0f, 8.0f);
      float command = hover + kp*e + cand - kd*rate + appliedDip + approachBrake;
      bool lagging = pulse >= LOW_DRIVE && command > pulse+1.0f;
      if (((!lagging && command <= cap) || cand < integral) && (command >= LOW_DRIVE || cand > integral))
        integral = cand;
    }
    pTerm = (launching ? LAUNCH_KP : kp)*e;
    iTerm = launching ? 0 : integral;
    dTerm = -(launching ? LAUNCH_KD : kd)*rate;
    float command = hover+pTerm+iTerm+dTerm+appliedDip+approachBrake+(launching ? launchBoost : 0);
    requested = (uint16_t)(constrain(command, (float)LOW_DRIVE, (float)cap)+0.5f);
  }
  if (requested <= pulse) setPulse(requested);
  else if (ms - lastPulse >= 20) {
    lastPulse = ms;
    setPulse(min(requested, pulse < LOW_DRIVE ? LOW_DRIVE : (uint16_t)(pulse+1)));
  }
}

bool parseValues(char *p, float *values, uint8_t count) {
  for (uint8_t i = 0; i < count; ++i) {
    char *end;
    values[i] = strtod(p, &end);
    if (end == p || !isfinite(values[i])) return false;
    p = end;
  }
  while (*p == ' ') ++p;
  return *p == 0;
}

void calibration(const float *v) {
  for (uint8_t i = 0; i < 3; ++i) { zeroAxis[i] = v[i]; tangent[i] = v[i+3]; bias[i] = v[i+6]; }
  for (uint8_t i = 0; i < 3; ++i) {
    uint8_t j = (i+1)%3, k = (i+2)%3;
    gyroAxis[i] = zeroAxis[k]*tangent[j] - zeroAxis[j]*tangent[k];
  }
  lower = v[9];
  upper = v[10];
  calibrated = true;
  target = 0;
  haveAngle = false;
}

// Replies are staged as complete lines to avoid interleaving with telemetry.
void reply(char command, bool success) {
  uint8_t next = (ackWrite+1)&7;
  if (next == ackRead) return;
  ackKey[ackWrite] = command;
  ackOK[ackWrite] = success;
  ackWrite = next;
}
void command() {
  char key = rx[0];
  bool simple = rx[1] == 0;
  if (key == 'H' && simple) { lastPacket = millis(); return; }
  if (key == 'X' && simple) { stopDrive(); armed = false; reply(key, true); return; }
  if (key == 'S' && simple) { stopDrive(); reply(key, true); return; }
  if (key == 'A' && simple && !armed && healthy(micros()) &&
      angle >= lower-4 && angle < upper-4 && millis()-lastPacket < 250) {
    fault = OK; armed = true; reply(key, true); return;
  }
  float v[11];
  if (key == 'K' && !armed && parseValues(rx+1, v, 11) &&
      fabsf(dot(v,v)-1) < 0.04f && fabsf(dot(v+3,v+3)-1) < 0.04f &&
      fabsf(dot(v,v+3)) < 0.03f && fabsf(v[6]) < 5000 && fabsf(v[7]) < 5000 &&
      fabsf(v[8]) < 5000 && v[9] >= -85 && v[9] <= -6 && v[10] >= 8 && v[10] <= 85) {
    calibration(v); reply(key, true); return;
  }
  if (key == 'L' && !armed && parseValues(rx+1, v, 2) &&
      v[0] >= 1120 && v[0] < v[1] && v[1] >= 1129 && v[1] <= HARD_MAX) {
    hover = (uint16_t)v[0]; cap = (uint16_t)v[1]; reply(key, true); return;
  }
  if (key == 'G' && mode == IDLE_MODE && parseValues(rx+1, v, 3) &&
      v[0] >= 0 && v[0] <= 2 && v[1] >= 0 && v[1] <= 1 && v[2] >= 0 && v[2] <= 1) {
    kp = v[0]; ki = v[1]; kd = v[2]; reply(key, true); return;
  }
  if (key == 'T' && parseValues(rx+1, v, 1) && targetOK(v[0])) {
    target = v[0]; reply(key, true); return;
  }
  if (armed && fault == OK && healthy(micros())) {
    if (key == 'J' && mode == PID_MODE && !launching && fabsf(target-angle) < 3.0f &&
        fabsf(rate) < 12.0f && parseValues(rx+1, v, 3) &&
        v[0] >= 0 && v[0] <= 2 && v[1] >= 0 && v[1] <= 1 && v[2] >= 0 && v[2] <= 1) {
      const float e = target-angle;
      const float previous = kp*e + integral - kd*rate;
      kp = v[0]; ki = v[1]; kd = v[2];
      integral = ki > 0 ? constrain(previous-kp*e+kd*rate, -8.0f, 8.0f) : 0;
      reply(key, true); return;
    }
    if (key == 'D' && simple && mode == PID_MODE && !launching &&
        angle < upper-6 && fabsf(target-angle) < 5.0f && fabsf(rate) < 8.0f &&
        (!dipScheduled || millis()-dipStart >= DIP_GAP_MS)) {
      dipStart = millis();
      dipScheduled = true;
      reply(key, true); return;
    }
    if (key == 'R' && simple && targetOK(target) && angle < upper-3) {
      if (mode != PID_MODE) {
        started = lastPulse = millis();
        integral = 0;
        launchReference = angle;
        launchBoost = 0;
        launching = true;
        mode = PID_MODE;
      }
      reply(key, true); return;
    }
    if (key == 'P' && parseValues(rx+1, v, 1) && v[0] >= LOW_DRIVE && v[0] <= cap &&
        angle < upper-2) {
      if (mode != FIXED) {
        started = lastPulse = millis();
        launching = false;
        dipScheduled = false;
        appliedDip = 0;
        pTerm = iTerm = dTerm = 0;
      }
      manualPulse = (uint16_t)v[0]; mode = FIXED; reply(key, true); return;
    }
  }
  reply(key, false);
}

void serviceSerial() {
  if (txPos == txSize && ackRead != ackWrite) {
    txSize = snprintf(tx, sizeof(tx), "ACK,%c,%d\n", ackKey[ackRead], ackOK[ackRead] ? 1 : 0);
    txPos = 0;
    ackRead = (ackRead+1)&7;
  }
  if (txPos < txSize) {
    uint8_t count = min((uint8_t)Serial.availableForWrite(), (uint8_t)(txSize-txPos));
    if (count) { Serial.write((uint8_t*)tx+txPos, count); txPos += count; }
  }
  // High-priority stop and heartbeat use the same short input buffer.
  for (uint8_t i = 0; i < 24 && Serial.available(); ++i) {
    char c = Serial.read();
    if (c == '\n') {
      if (rxSize && !rxOverflow) { rx[rxSize] = 0; command(); }
      rxSize = 0; rxOverflow = false;
    } else if (c != '\r') {
      if (rxSize < sizeof(rx)-1) rx[rxSize++] = c;
      else rxOverflow = true;
    }
  }
}

int scaled(float v) { return (int)constrain(v*100, -32760.0f, 32760.0f); }
void telemetry() {
  if (txPos != txSize || ackRead != ackWrite) return;
  uint8_t flags = sensorOK | (calibrated<<1) | (haveAngle<<2) | (armed<<3) | (gravityOK<<4) | (launching<<5);
  txSize = snprintf(tx, sizeof(tx), "ARM3,%lu,%lu,%u,%u,%u,%u,%u,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%u,%d,%d,%d,%d\n",
     millis(), sequence++, flags, (unsigned)mode, (unsigned)fault, pulse, cap,
     scaled(target), scaled(angle), scaled(accelAngle), scaled(rate),
     raw[0], raw[1], raw[2], raw[3], raw[4], raw[5], imuAddress,
     scaled(pTerm), scaled(iTerm), scaled(dTerm), (int)appliedDip);
  txPos = 0;
}

void setup() {
  MCUSR = 0;
  wdt_disable();
  startEsc();
  MCUCR |= _BV(PUD);  // Prevent internal 5 V I2C pull-ups.
  Wire.begin();
  Wire.setClock(100000);
  Wire.setWireTimeout(2500, true);
  Serial.begin(115200);
  wdt_enable(WDTO_250MS);
}
void loop() {
  wdt_reset();
  uint32_t now = micros(), ms = millis();
  if (!imuAddress && !armed && ms-lastProbe >= 400) { lastProbe = ms; probeImu(); }
  if ((int32_t)(now-nextSensor) >= 0) { nextSensor = now+SENSOR_US; readImu(now); }
  if ((int32_t)(now-nextPid) >= 0) { nextPid = now+PID_US; control(now, ms); }
  serviceSerial();
  if ((int32_t)(now-nextFrame) >= 0) { nextFrame = now+FRAME_US; telemetry(); }
}
