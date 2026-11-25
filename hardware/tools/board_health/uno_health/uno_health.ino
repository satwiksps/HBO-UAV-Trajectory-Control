// Bare Uno only: disconnect all external wiring before issuing TEST.
// Diagnostic firmware, not the propeller-arm controller. D9 stays LOW except
// for a 100 ms, 47% duty PWM self-test requested by TEST.
#include <Arduino.h>
#include <EEPROM.h>
#include <avr/wdt.h>

volatile uint8_t ramProbe[512];
char command[48];
uint8_t commandLength = 0;
bool streaming = false;
uint32_t sequence = 0, ticks = 0, missed = 0, nextTick = 0;
uint32_t maxWorkUs = 0, maxLateUs = 0, lastCommandMs = 0;
uint16_t adcAverage = 0, bandgapAdc = 0, supplyMv = 0;
volatile float calculationSink = 0;
float integralProbe = 0, previousAngle = 0, rateProbe = 0;

uint16_t crc16(const char *text) {
  uint16_t crc = 0xFFFF;
  while (*text) {
    crc ^= (uint16_t)(uint8_t)*text++ << 8;
    for (uint8_t b = 0; b < 8; ++b)
      crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : crc << 1;
  }
  return crc;
}

void frame(const char *text) {
  char checksum[7];
  snprintf(checksum, sizeof(checksum), "*%04X", crc16(text));
  Serial.print(text);
  Serial.println(checksum);
}

uint16_t internalAdc(uint8_t channel) {
  ADMUX = _BV(REFS0) | channel;
  delayMicroseconds(1000);
  ADCSRA |= _BV(ADSC);
  while (ADCSRA & _BV(ADSC)) {}
  (void)ADC;
  uint16_t sum = 0;
  for (uint8_t n = 0; n < 8; ++n) {
    ADCSRA |= _BV(ADSC);
    while (ADCSRA & _BV(ADSC)) {}
    sum += ADC;
  }
  return (sum + 4) / 8;
}

void readSupply() {
  bandgapAdc = internalAdc(0x0E);
  // Nominal 1.1 V bandgap is NOT calibrated: this is only a rough estimate.
  supplyMv = bandgapAdc ? 1125300UL / bandgapAdc : 0;
}

uint16_t ramTest() {
  uint16_t errors = 0;
  const uint8_t patterns[] = {0x00, 0xFF, 0x55, 0xAA};
  for (uint8_t p = 0; p < sizeof(patterns); ++p) {
    for (uint16_t i = 0; i < sizeof(ramProbe); ++i) ramProbe[i] = patterns[p];
    for (uint16_t i = 0; i < sizeof(ramProbe); ++i)
      if (ramProbe[i] != patterns[p]) ++errors;
  }
  for (uint16_t i = 0; i < sizeof(ramProbe); ++i) ramProbe[i] = (uint8_t)(i ^ (i >> 8));
  for (uint16_t i = 0; i < sizeof(ramProbe); ++i)
    if (ramProbe[i] != (uint8_t)(i ^ (i >> 8))) ++errors;
  return errors;
}

void allPinsIdle() {
  analogWrite(9, 0);
  for (uint8_t pin = 2; pin <= A5; ++pin) {
    pinMode(pin, INPUT);
    digitalWrite(pin, LOW);
  }
  pinMode(9, OUTPUT);
  digitalWrite(9, LOW);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
}

void selfTest() {
  streaming = false;
  allPinsIdle();
  frame("TEST_BEGIN");
  char msg[140];
  snprintf(msg, sizeof(msg), "RAM,512,%u", ramTest());
  frame(msg);
  uint32_t pinErrors = 0;
  for (uint8_t pin = 2; pin <= A5; ++pin) {
    digitalWrite(pin, LOW);
    pinMode(pin, OUTPUT);
    delayMicroseconds(20);
    bool good = digitalRead(pin) == LOW;
    digitalWrite(pin, HIGH);
    delayMicroseconds(20);
    good = good && digitalRead(pin) == HIGH;
    if (!good) pinErrors |= 1UL << pin;
    pinMode(pin, INPUT);
    digitalWrite(pin, LOW);
  }
  snprintf(msg, sizeof(msg), "GPIO,18,%lu", (unsigned long)pinErrors);
  frame(msg);
  uint16_t ground = internalAdc(0x0F);
  readSupply();
  snprintf(msg, sizeof(msg), "ADC_INTERNAL,%u,%u,%u", ground, bandgapAdc, supplyMv);
  frame(msg);
  for (uint8_t pin = A0; pin <= A5; ++pin) {
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);
    delayMicroseconds(30);
    analogRead(pin);
    uint16_t low = analogRead(pin);
    digitalWrite(pin, HIGH);
    delayMicroseconds(30);
    analogRead(pin);
    uint16_t high = analogRead(pin);
    snprintf(msg, sizeof(msg), "ADC_PIN,%u,%u,%u", pin - A0, low, high);
    frame(msg);
    pinMode(pin, INPUT);
    digitalWrite(pin, LOW);
  }
  // Sample the actual D9 input pad (PINB), not just the output register.
  TCCR1A = _BV(WGM10);
  TCCR1B = _BV(CS11);
  analogWrite(9, 120);
  uint8_t previous = (PINB & _BV(PB1)) != 0;
  uint32_t start = micros(), lastRise = 0, periodSum = 0, highSum = 0;
  uint16_t periods = 0, highs = 0;
  while ((uint32_t)(micros() - start) < 100000UL) {
    uint8_t current = (PINB & _BV(PB1)) != 0;
    if (current != previous) {
      uint32_t now = micros();
      if (current) {
        if (lastRise) { periodSum += now - lastRise; ++periods; }
        lastRise = now;
      } else if (lastRise) { highSum += now - lastRise; ++highs; }
      previous = current;
    }
  }
  analogWrite(9, 0);
  uint32_t hz = periodSum ? (1000000UL * periods) / periodSum : 0;
  uint16_t permille = (highs && periodSum) ?
    (uint16_t)((1000ULL * highSum * periods) / ((uint64_t)highs * periodSum)) : 0;
  snprintf(msg, sizeof(msg), "PWM_D9,%lu,%u,%u", (unsigned long)hz, permille, periods);
  frame(msg);
  allPinsIdle();
  frame("TEST_END");
}

void eepromTestAndClear() {
  streaming = false;
  allPinsIdle();
  frame("EEPROM_BEGIN");
  uint16_t errors = 0;
  const uint8_t patterns[] = {0x55, 0xAA, 0xFF};
  for (uint8_t p = 0; p < sizeof(patterns); ++p) {
    for (uint16_t i = 0; i < EEPROM.length(); ++i) EEPROM.update(i, patterns[p]);
    for (uint16_t i = 0; i < EEPROM.length(); ++i)
      if (EEPROM.read(i) != patterns[p]) ++errors;
  }
  char msg[60];
  snprintf(msg, sizeof(msg), "EEPROM,%u,%u,255", EEPROM.length(), errors);
  frame(msg);
}

void dispatch() {
  lastCommandMs = millis();
  if (!strcmp(command, "TEST")) selfTest();
  else if (!strcmp(command, "EEPROM")) eepromTestAndClear();
  else if (!strcmp(command, "START")) {
    allPinsIdle();
    pinMode(A0, INPUT_PULLUP);
    readSupply();
    ticks = sequence = missed = maxWorkUs = maxLateUs = 0;
    integralProbe = previousAngle = rateProbe = 0;
    nextTick = micros() + 5000UL;
    streaming = true;
    frame("STARTED");
  } else if (!strcmp(command, "STOP")) {
    streaming = false;
    allPinsIdle();
    frame("STOPPED");
  } else if (!strncmp(command, "ECHO,", 5)) frame(command);
  else frame("ERROR,UNKNOWN_COMMAND");
}

void setup() {
  MCUSR = 0;
  wdt_disable();
  allPinsIdle();
  Serial.begin(115200);
  frame("BOOT,UNO_HEALTH,1");
}

void loop() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      command[commandLength] = 0;
      dispatch();
      commandLength = 0;
    } else if (c != '\r') {
      if (commandLength < sizeof(command) - 1) command[commandLength++] = c;
      else { commandLength = 0; frame("ERROR,COMMAND_TOO_LONG"); }
    }
  }
  if (!streaming) return;
  if ((uint32_t)(millis() - lastCommandMs) > 5000UL) {
    streaming = false;
    allPinsIdle();
    frame("STOPPED,HOST_TIMEOUT");
    return;
  }
  uint32_t now = micros();
  if ((int32_t)(now - nextTick) < 0) return;
  uint32_t late = now - nextTick;
  if (late > maxLateUs) maxLateUs = late;
  if (late >= 5000UL) { missed += late / 5000UL; nextTick = now; }
  nextTick += 5000UL;
  uint32_t workStart = micros();
  uint16_t sum = 0;
  for (uint8_t n = 0; n < 8; ++n) sum += analogRead(A0);
  adcAverage = (sum + 4) / 8;
  float angle = ((float)adcAverage - 512.0f) / 3.79f;
  float error = 20.0f - angle;
  integralProbe = constrain(integralProbe + 3.0f * error * 0.005f, -60.0f, 60.0f);
  rateProbe += 0.15f * ((angle - previousAngle) / 0.005f - rateProbe);
  previousAngle = angle;
  calculationSink = constrain(120.0f + 3.0f * error + integralProbe - rateProbe, 0.0f, 189.0f);
  ++ticks;
  if (ticks % 200UL == 0) {
    readSupply();
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  }
  if (ticks % 4UL == 0) {
    char msg[140];
    snprintf(msg, sizeof(msg), "DATA,%lu,%lu,%lu,%u,%u,%lu,%lu,%lu",
      (unsigned long)sequence++, (unsigned long)millis(), (unsigned long)ticks,
      adcAverage, supplyMv, (unsigned long)missed,
      (unsigned long)maxWorkUs, (unsigned long)maxLateUs);
    frame(msg);
  }
  uint32_t workUs = micros() - workStart;
  if (workUs > maxWorkUs) maxWorkUs = workUs;
}
