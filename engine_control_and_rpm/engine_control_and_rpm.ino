// ExArk 351cc Drone Engine Bench Controller
// engine_control_and_rpm.ino
//
// - Reads CDI hall pulses on Pin 18 (interrupt) -> RPM
// - Reads 10K pot on A0 -> throttle command
// - Outputs RC servo PWM on Pin 9 to ECU 舵机接收器 input
// - Displays RPM + THR% on I2C 1602 LCD
// - Cuts throttle to 1000us if RPM exceeds MAX_RPM

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Servo.h>

// ---------- Pins ----------
const uint8_t  THROTTLE_OUTPUT_PIN = 9;
const uint8_t  POT_INPUT_PIN       = A0;
const uint8_t  RPM_INPUT_PIN       = 2;    // INT4 on Mega

// ---------- Constants ----------
const uint8_t  LCD_ADDRESS         = 0x27; // try 0x3F if blank
const uint8_t  MAX_THROTTLE        = 30;   // % cap during break-in
const uint16_t MAX_RPM             = 6000;
const uint8_t  PULSES_PER_REV      = 1;
const uint16_t UPDATE_INTERVAL_MS  = 500;

const uint16_t PWM_MIN_US          = 1000; // 0%   throttle
const uint16_t PWM_MAX_US          = 2000; // 100% throttle

// ---------- Globals ----------
LiquidCrystal_I2C lcd(LCD_ADDRESS, 16, 2);
Servo throttleOut;

volatile uint32_t pulseCount = 0;
uint32_t lastUpdateMs = 0;

uint16_t currentRpm     = 0;
uint8_t  currentThrPct  = 0;
uint16_t currentPwmUs   = PWM_MIN_US;
bool     overspeed      = false;

// ---------- ISR ----------
void onRpmPulse() {
  pulseCount++;
}

// ---------- Setup ----------
void setup() {
  Serial.begin(9600);

  pinMode(POT_INPUT_PIN, INPUT);

  throttleOut.attach(THROTTLE_OUTPUT_PIN);
  throttleOut.writeMicroseconds(PWM_MIN_US); // safe start: 0% throttle

  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0); lcd.print("ExArk 351cc");
  lcd.setCursor(0, 1); lcd.print("Init...");

  pinMode(RPM_INPUT_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(RPM_INPUT_PIN), onRpmPulse, RISING);

  delay(800);
  lcd.clear();
  lastUpdateMs = millis();
}

// ---------- Loop ----------
void loop() {
  // --- Throttle command (every loop, snappy) ---
  uint16_t pot = analogRead(POT_INPUT_PIN);                    // 0..1023
  uint8_t  thrPct = map(pot, 0, 1023, 0, MAX_THROTTLE);        // 0..MAX_THROTTLE
  uint16_t pwmUs  = map(thrPct, 0, 100, PWM_MIN_US, PWM_MAX_US);

  // Overspeed override — hard close
  if (overspeed) {
    pwmUs = PWM_MIN_US;
    thrPct = 0;
  }

  throttleOut.writeMicroseconds(pwmUs);
  currentThrPct = thrPct;
  currentPwmUs  = pwmUs;

  // --- 500ms tick: RPM, LCD, Serial ---
  uint32_t now = millis();
  if (now - lastUpdateMs >= UPDATE_INTERVAL_MS) {
    uint32_t elapsedMs = now - lastUpdateMs;
    lastUpdateMs = now;

    noInterrupts();
    uint32_t pulses = pulseCount;
    pulseCount = 0;
    interrupts();

    // RPM = (pulses / sec) * 60 / PULSES_PER_REV
    currentRpm = (uint16_t)((pulses * 60000UL) / (elapsedMs * PULSES_PER_REV));

    // Overspeed latch on / off
    if (currentRpm > MAX_RPM) {
      overspeed = true;
    } else if (overspeed && currentRpm < MAX_RPM) {
      overspeed = false;
    }

    updateLcd();
    logSerial(elapsedMs, pulses);
  }
}

// ---------- LCD ----------
void updateLcd() {
  char line0[17];
  char line1[17];

  if (overspeed) {
    snprintf(line0, sizeof(line0), "OVERSPEED       ");
  } else {
    snprintf(line0, sizeof(line0), "RPM: %-10u", currentRpm);
  }
  snprintf(line1, sizeof(line1), "THR: %-3u%%       ", currentThrPct);

  lcd.setCursor(0, 0); lcd.print(line0);
  lcd.setCursor(0, 1); lcd.print(line1);
}

// ---------- Serial ----------
void logSerial(uint32_t elapsedMs, uint32_t pulses) {
  Serial.print(F("RPM=")); Serial.print(currentRpm);
  Serial.print(F("  THR=")); Serial.print(currentThrPct); Serial.print('%');
  Serial.print(F("  PWM=")); Serial.print(currentPwmUs); Serial.print(F("us"));
  Serial.print(F("  pulses=")); Serial.print(pulses);
  Serial.print(F("  dt=")); Serial.print(elapsedMs); Serial.print(F("ms"));
  if (overspeed) Serial.print(F("  [OVERSPEED]"));
  Serial.println();
}
