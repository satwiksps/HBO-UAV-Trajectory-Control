#include <Arduino.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <util/atomic.h>

// Uno (ATmega328P at 16 MHz). RC pulses at 50 Hz.
// Bench test with propeller removed and motor firmly mounted.
#if !defined(__AVR_ATmega328P__) || F_CPU != 16000000UL
#error This bench sketch requires a 16 MHz Arduino Uno.
#endif

#ifndef ESC_SIGNAL_PIN
#define ESC_SIGNAL_PIN 9
#endif
#if ESC_SIGNAL_PIN != 6 && ESC_SIGNAL_PIN != 9
#error The ESC signal pin must be D6 or D9.
#endif

#ifndef ESC_IDLE_US
#define ESC_IDLE_US 1000
#endif
static_assert(ESC_IDLE_US == 950 || ESC_IDLE_US == 1000,
              "Use either the regular idle or the short arming test pulse");
constexpr uint16_t idleUs = ESC_IDLE_US;
constexpr uint8_t signalPin = ESC_SIGNAL_PIN;
constexpr uint32_t startupMs = 5000;
constexpr uint32_t spinMs = 2800;
constexpr uint32_t stepMs = 560;
constexpr uint16_t maxPulseUs = 1300;
constexpr uint32_t commandTimeoutMs = 750;
constexpr uint32_t liveLimitMs = 180000;
constexpr uint32_t slewIntervalMs = 20;
constexpr uint16_t slewStepUs = 20;
const uint16_t spinSteps[] = {1100, 1150, 1200, 1250, maxPulseUs};
static_assert(sizeof(spinSteps) / sizeof(spinSteps[0]) * stepMs == spinMs,
              "Each throttle step must fit in the fixed run time");

uint32_t bootMs = 0;
uint32_t spinStartMs = 0;
bool spinning = false;
bool live = false;
enum class IdleReason : uint8_t { Normal, LinkTimeout, RunLimit };
IdleReason idleReason = IdleReason::Normal;
uint32_t liveStartMs = 0;
uint32_t lastCommandMs = 0;
uint32_t lastSlewMs = 0;
uint8_t step = 0;
uint16_t commandUs = idleUs;
uint16_t targetUs = idleUs;
char line[16];
uint8_t lineLength = 0;
bool overlong = false;

#if ESC_SIGNAL_PIN == 6
// Timer1 compare interrupts drive D6 (PD6) without changing Timer0.
volatile uint16_t nextPulseTicks = idleUs * 2;

ISR(TIMER1_COMPA_vect) {
  OCR1B = nextPulseTicks;
  PORTD |= _BV(PD6);
}

ISR(TIMER1_COMPB_vect) {
  PORTD &= ~_BV(PD6);
}
#endif

void setPulse(uint16_t pulseUs) {
  commandUs = pulseUs;
#if ESC_SIGNAL_PIN == 6
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    nextPulseTicks = pulseUs * 2;
  }
#else
  OCR1A = pulseUs * 2;
#endif
}

void startPulses() {
  digitalWrite(signalPin, LOW);
  pinMode(signalPin, OUTPUT);
  // Timer1 runs at 2 MHz for a 20 ms frame on either pin.
  TCCR1B = 0;
  TCCR1A = 0;
  TCNT1 = 0;
#if ESC_SIGNAL_PIN == 6
  OCR1A = 39999;
  OCR1B = idleUs * 2;
  TIFR1 = _BV(OCF1A) | _BV(OCF1B);
  TIMSK1 = _BV(OCIE1A) | _BV(OCIE1B);
  TCCR1B = _BV(WGM12) | _BV(CS11);
#else
  ICR1 = 39999;
  OCR1A = idleUs * 2;
  TCCR1A = _BV(COM1A1) | _BV(WGM11);
  TCCR1B = _BV(WGM13) | _BV(WGM12) | _BV(CS11);
#endif
}

void idleOutput(IdleReason reason) {
  spinning = false;
  live = false;
  targetUs = idleUs;
  idleReason = reason;
  setPulse(idleUs);
}

void stopSpin() {
  idleOutput(IdleReason::Normal);
  Serial.print(F("IDLE,"));
  Serial.println(idleUs);
}

void sendState() {
  Serial.print(F("ESC_LIVE_V1,D"));
  Serial.print(signalPin);
  Serial.print(',');
  Serial.print(idleUs);
  Serial.print(',');
  Serial.print(maxPulseUs);
  Serial.print(',');
  Serial.print(commandUs);
  Serial.print(',');
  if (live) Serial.println(F("LIVE"));
  else if (spinning) Serial.println(F("RAMP"));
  else if (idleReason == IdleReason::LinkTimeout) Serial.println(F("TIMEOUT"));
  else if (idleReason == IdleReason::RunLimit) Serial.println(F("LIMIT"));
  else Serial.println(F("IDLE"));
}

void announceThrottle(uint16_t pulseUs) {
  setPulse(pulseUs);
  Serial.print(F("THROTTLE,"));
  Serial.println(pulseUs);
}

void handleLine() {
  if (strcmp(line, "STOP") == 0) {
    stopSpin();
  } else if (strcmp(line, "STATE") == 0) {
    sendState();
  } else if (strcmp(line, "LIVE") == 0 && !spinning && !live &&
             millis() - bootMs >= startupMs) {
    idleOutput(IdleReason::Normal);
    liveStartMs = lastCommandMs = lastSlewMs = millis();
    live = true;
    sendState();
  } else if (strncmp(line, "SET ", 4) == 0 && live) {
    char *end;
    const long value = strtol(line + 4, &end, 10);
    if (end != line + 4 && *end == '\0' && value >= idleUs && value <= maxPulseUs) {
      lastCommandMs = millis();
      targetUs = (uint16_t)value;
    } else {
      idleOutput(IdleReason::Normal);
      Serial.println(F("REJECTED"));
    }
  } else if (strcmp(line, "CHECK") == 0 && !spinning && !live) {
    uint32_t pulse = pulseIn(signalPin, HIGH, 45000);
    Serial.print(F("ESC_BENCH_V2,D"));
    Serial.print(signalPin);
    Serial.print(',');
    Serial.print(commandUs);
    Serial.print(',');
    Serial.println(pulse);
  } else if (strcmp(line, "SPIN") == 0 && !spinning && !live &&
             millis() - bootMs >= startupMs) {
    spinStartMs = millis();
    step = 0;
    spinning = true;
    announceThrottle(spinSteps[step]);
  } else {
    Serial.println(F("REJECTED"));
  }
}

void setup() {
  MCUSR = 0;
  wdt_disable();
  startPulses();
  Serial.begin(115200);
  bootMs = millis();
  wdt_enable(WDTO_250MS);
}

void updateLiveOutput(uint32_t now) {
  if (now - lastCommandMs >= commandTimeoutMs) {
    idleOutput(IdleReason::LinkTimeout);
  } else if (now - liveStartMs >= liveLimitMs) {
    idleOutput(IdleReason::RunLimit);
  } else if (now - lastSlewMs >= slewIntervalMs) {
    lastSlewMs = now;
    // A lower request takes effect on the next frame.
    if (targetUs < commandUs) setPulse(targetUs);
    else if (targetUs > commandUs) setPulse(min(targetUs, commandUs + slewStepUs));
  }
}

void loop() {
  wdt_reset();
  if (live) updateLiveOutput(millis());
  if (spinning) {
    uint32_t elapsed = millis() - spinStartMs;
    if (elapsed >= spinMs) {
      stopSpin();
    } else {
      uint8_t scheduledStep = elapsed / stepMs;
      if (scheduledStep != step) {
        step = scheduledStep;
        announceThrottle(spinSteps[step]);
      }
    }
  }
  // Process one byte per pass so serial input cannot postpone the motor cutoff.
  if (!Serial.available()) return;
  char c = Serial.read();
  if (c == '\n') {
    if (!overlong) {
      line[lineLength] = '\0';
      handleLine();
    }
    overlong = false;
    lineLength = 0;
  } else if (c != '\r') {
    if (lineLength < sizeof(line) - 1) line[lineLength++] = c;
    else overlong = true;
  }
}
