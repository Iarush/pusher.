/*
  Pololu #998 motor test with MT6701 encoder speed feedback
  Raspberry Pi Pico 2 + Waveshare Pico-Motor-Driver
  Arduino core: Earle F. Philhower III RP2040/RP2350 core

  Variant of pico_motor_test.ino. Each motor's duty is corrected every tick
  from its measured RPM (MT6701, I2C mode), so measured wheel speed, not just
  elapsed time, sets the commanded duty.

  Sequence (same time-based profile as pico_motor_test.ino):
    - Forward for 5 seconds:
        5% -> 30% over 2.5 seconds
        30% -> 3% over 2.5 seconds
    - Reverse for 5 seconds: 3% -> 7%
    - Active brake and remain stopped

  Expected Waveshare Pico-Motor-Driver mapping:
    Motor 1 on C1/C2: PCA9685 channels 6 = PWM, 7 = IN1, 8 = IN2
    Motor 2 on D1/D2: PCA9685 channels 9 = PWM, 10 = IN1, 11 = IN2
  Check the mapping and I2C address against your board revision before
  connecting a motor.

  Encoder wiring:
    The MT6701 I2C address is fixed at 0x06, so the two encoders need separate
    buses. Encoder 1 shares I2C0 (Wire, GP20 = SDA, GP21 = SCL) with the PCA9685.
    Encoder 2 uses I2C1 (Wire1) on ENCODER2_SDA_PIN / ENCODER2_SCL_PIN.
    Power both MT6701 boards from 3.3 V.

  MT6701 I2C register map (per the MagnTek datasheet):
    0x03 = ANGLE_H: bits [13:6] of the 14-bit angle
    0x04 = ANGLE_L: bits [5:0] in bits [7:2]; bits [1:0] are status, unused here
    raw14 = (reg[0x03] << 6) | (reg[0x04] >> 2)
  Verify against your datasheet revision before trusting it on new hardware.
*/

#include <Wire.h>

constexpr uint8_t SDA_PIN = 20;
constexpr uint8_t SCL_PIN = 21;

constexpr uint8_t ENCODER2_SDA_PIN = 18;
constexpr uint8_t ENCODER2_SCL_PIN = 19;

constexpr uint8_t START_BUTTON_PIN = 15;
constexpr unsigned long BUTTON_DEBOUNCE_TIME_MS = 30;

constexpr uint8_t PCA9685_ADDRESS = 0x40;

// Motor 1 is PCA9685 channels 6-8, motor 2 is channels 9-11.
constexpr uint8_t MOTOR_1_PWM_CHANNEL = 6;
constexpr uint8_t MOTOR_1_IN1_CHANNEL = 7;
constexpr uint8_t MOTOR_1_IN2_CHANNEL = 8;
constexpr uint8_t MOTOR_2_PWM_CHANNEL = 9;
constexpr uint8_t MOTOR_2_IN1_CHANNEL = 10;
constexpr uint8_t MOTOR_2_IN2_CHANNEL = 11;

// Both motors' channels are contiguous (6 through 11), so they are written in
// one I2C transaction.
constexpr uint8_t MOTOR_CHANNEL_START = 6;
constexpr uint8_t MOTOR_CHANNEL_COUNT = 6;

constexpr uint16_t PCA9685_FULL_SCALE = 4095;

// The Pololu #998 is rated for 6 V, while the motor rail is 9 V.
constexpr float MOTOR_RATED_VOLTAGE = 6.0f;
constexpr float MOTOR_SUPPLY_VOLTAGE = 9.0f;

// The 6 V / 9 V ceiling is 66.7%. Do not raise these without checking current
// and temperature.
constexpr float START_DUTY_PERCENT = 5.0f;
constexpr float FORWARD_PEAK_DUTY_PERCENT = 30.0f;
constexpr float FORWARD_END_DUTY_PERCENT = 3.0f;
constexpr float REVERSE_START_DUTY_PERCENT = 3.0f;
constexpr float REVERSE_END_DUTY_PERCENT = 7.0f;

constexpr unsigned long FORWARD_PROFILE_TIME_MS = 5000;
constexpr unsigned long FORWARD_ACCELERATION_TIME_MS = 2500;
constexpr unsigned long REVERSE_PROFILE_TIME_MS = 5000;
constexpr unsigned long RAMP_UPDATE_INTERVAL_MS = 50;

// Proportional gain that pulls each motor's duty toward the two-motor average
// speed, in duty percent per RPM of error. Start low and raise while watching
// the telemetry; too high will overshoot.
constexpr float SPEED_MATCH_KP_PERCENT_PER_RPM = 0.05f;

constexpr unsigned long TELEMETRY_INTERVAL_MS = 250;

constexpr uint8_t PCA9685_MODE1 = 0x00;
constexpr uint8_t PCA9685_MODE2 = 0x01;
constexpr uint8_t PCA9685_PRESCALE = 0xFE;
constexpr uint8_t PCA9685_LED0_ON_L = 0x06;

constexpr uint8_t MT6701_I2C_ADDRESS = 0x06;
constexpr uint8_t MT6701_ANGLE_H_REG = 0x03;
constexpr uint8_t MT6701_ANGLE_L_REG = 0x04;
constexpr int32_t MT6701_COUNTS_PER_REV = 16384; // 14-bit raw angle

void pcaWriteRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(PCA9685_ADDRESS);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

uint8_t pcaReadRegister(uint8_t reg) {
  Wire.beginTransmission(PCA9685_ADDRESS);
  Wire.write(reg);

  if (Wire.endTransmission(false) != 0) {
    return 0;
  }

  if (Wire.requestFrom(PCA9685_ADDRESS, static_cast<uint8_t>(1)) != 1) {
    return 0;
  }

  return Wire.read();
}

void setFullOn(uint16_t &onValue, uint16_t &offValue) {
  onValue = 0x1000;
  offValue = 0x0000;
}

void setFullOff(uint16_t &onValue, uint16_t &offValue) {
  onValue = 0x0000;
  offValue = 0x1000;
}

void setPwm(uint16_t pwmValue, uint16_t &onValue, uint16_t &offValue) {
  if (pwmValue == 0) {
    setFullOff(onValue, offValue);
    return;
  }

  if (pwmValue >= PCA9685_FULL_SCALE) {
    setFullOn(onValue, offValue);
    return;
  }

  onValue = 0x0000;
  offValue = pwmValue;
}

// Writes channels 6 through 11 in one transaction.
void pcaWriteMotorChannels(const uint16_t onValues[MOTOR_CHANNEL_COUNT],
                           const uint16_t offValues[MOTOR_CHANNEL_COUNT]) {
  Wire.beginTransmission(PCA9685_ADDRESS);
  Wire.write(
      static_cast<uint8_t>(PCA9685_LED0_ON_L + (4 * MOTOR_CHANNEL_START)));

  for (uint8_t channel = 0; channel < MOTOR_CHANNEL_COUNT; ++channel) {
    Wire.write((uint8_t)(onValues[channel] & 0xFF));
    Wire.write((uint8_t)((onValues[channel] >> 8) & 0x1F));
    Wire.write((uint8_t)(offValues[channel] & 0xFF));
    Wire.write((uint8_t)((offValues[channel] >> 8) & 0x1F));
  }

  Wire.endTransmission();
}

void pcaSetPwmFrequency(uint16_t frequencyHz) {
  // The PCA9685 oscillator runs at 25 MHz.
  float prescaleValue = (25000000.0f / (4096.0f * frequencyHz)) - 1.0f;

  if (prescaleValue < 3.0f) {
    prescaleValue = 3.0f;
  }
  if (prescaleValue > 255.0f) {
    prescaleValue = 255.0f;
  }

  const uint8_t prescale = static_cast<uint8_t>(prescaleValue);
  const uint8_t oldMode1 = pcaReadRegister(PCA9685_MODE1);

  // PRE_SCALE can only be written in sleep mode.
  pcaWriteRegister(PCA9685_MODE1,
                   static_cast<uint8_t>((oldMode1 & 0x7F) | 0x10));
  pcaWriteRegister(PCA9685_PRESCALE, prescale);
  pcaWriteRegister(PCA9685_MODE1, oldMode1);
  delay(1);
  pcaWriteRegister(PCA9685_MODE1, static_cast<uint8_t>(oldMode1 | 0xA0));
}

uint16_t ratedVoltagePwmLimit() {
  const float limit =
      (MOTOR_RATED_VOLTAGE / MOTOR_SUPPLY_VOLTAGE) * PCA9685_FULL_SCALE;

  if (limit < 0.0f) {
    return 0;
  }
  if (limit > PCA9685_FULL_SCALE) {
    return PCA9685_FULL_SCALE;
  }
  return static_cast<uint16_t>(limit);
}

// Final clamp, applied after any encoder correction has been added.
uint16_t pwmValueForPercent(float dutyPercent) {
  const float requested = (dutyPercent / 100.0f) * PCA9685_FULL_SCALE;
  const uint16_t voltageLimit = ratedVoltagePwmLimit();

  if (requested < 0.0f) {
    return 0;
  }
  if (requested > voltageLimit) {
    return voltageLimit;
  }
  return static_cast<uint16_t>(requested);
}

void motorCoastBoth() {
  uint16_t onValues[MOTOR_CHANNEL_COUNT] = {0};
  uint16_t offValues[MOTOR_CHANNEL_COUNT] = {0};

  for (uint8_t channel = 0; channel < MOTOR_CHANNEL_COUNT; ++channel) {
    setFullOff(onValues[channel], offValues[channel]);
  }

  pcaWriteMotorChannels(onValues, offValues);
}

// Same direction for both motors, but each gets its own PWM so the encoder
// feedback can correct them independently.
void motorDriveIndividual(bool forward, uint16_t pwm1, uint16_t pwm2) {
  uint16_t onValues[MOTOR_CHANNEL_COUNT] = {0};
  uint16_t offValues[MOTOR_CHANNEL_COUNT] = {0};

  for (uint8_t channel = 0; channel < MOTOR_CHANNEL_COUNT; ++channel) {
    setFullOff(onValues[channel], offValues[channel]);
  }

  const uint8_t motor1PwmIndex = MOTOR_1_PWM_CHANNEL - MOTOR_CHANNEL_START;
  const uint8_t motor1In1Index = MOTOR_1_IN1_CHANNEL - MOTOR_CHANNEL_START;
  const uint8_t motor1In2Index = MOTOR_1_IN2_CHANNEL - MOTOR_CHANNEL_START;
  const uint8_t motor2PwmIndex = MOTOR_2_PWM_CHANNEL - MOTOR_CHANNEL_START;
  const uint8_t motor2In1Index = MOTOR_2_IN1_CHANNEL - MOTOR_CHANNEL_START;
  const uint8_t motor2In2Index = MOTOR_2_IN2_CHANNEL - MOTOR_CHANNEL_START;

  setPwm(pwm1, onValues[motor1PwmIndex], offValues[motor1PwmIndex]);
  setPwm(pwm2, onValues[motor2PwmIndex], offValues[motor2PwmIndex]);

  if (forward) {
    setFullOn(onValues[motor1In1Index], offValues[motor1In1Index]);
    setFullOff(onValues[motor1In2Index], offValues[motor1In2Index]);
    setFullOn(onValues[motor2In1Index], offValues[motor2In1Index]);
    setFullOff(onValues[motor2In2Index], offValues[motor2In2Index]);
  } else {
    setFullOff(onValues[motor1In1Index], offValues[motor1In1Index]);
    setFullOn(onValues[motor1In2Index], offValues[motor1In2Index]);
    setFullOff(onValues[motor2In1Index], offValues[motor2In1Index]);
    setFullOn(onValues[motor2In2Index], offValues[motor2In2Index]);
  }

  pcaWriteMotorChannels(onValues, offValues);
}

// TB6612FNG short brake: IN1 = HIGH, IN2 = HIGH, PWM = HIGH.
void motorActiveBrakeBoth() {
  uint16_t onValues[MOTOR_CHANNEL_COUNT] = {0};
  uint16_t offValues[MOTOR_CHANNEL_COUNT] = {0};

  for (uint8_t channel = 0; channel < MOTOR_CHANNEL_COUNT; ++channel) {
    setFullOn(onValues[channel], offValues[channel]);
  }

  pcaWriteMotorChannels(onValues, offValues);
}

bool mt6701CheckPresence(TwoWire &bus) {
  bus.beginTransmission(MT6701_I2C_ADDRESS);
  return bus.endTransmission() == 0;
}

// Reads the 14-bit angle (0..16383). Returns false, leaving rawOut unchanged,
// if the device does not ACK.
bool mt6701ReadAngleRaw(TwoWire &bus, uint16_t &rawOut) {
  bus.beginTransmission(MT6701_I2C_ADDRESS);
  bus.write(MT6701_ANGLE_H_REG);

  if (bus.endTransmission(false) != 0) {
    return false;
  }

  if (bus.requestFrom(MT6701_I2C_ADDRESS, static_cast<uint8_t>(2)) != 2) {
    return false;
  }

  const uint8_t angleHigh = bus.read();
  const uint8_t angleLow = bus.read();

  rawOut = (static_cast<uint16_t>(angleHigh) << 6) |
           (static_cast<uint16_t>(angleLow) >> 2);
  return true;
}

struct EncoderTracker {
  bool hasLastReading = false;
  uint16_t lastRaw = 0;
  unsigned long lastUpdateMs = 0;
  float rpm = 0.0f;
};

EncoderTracker motor1Encoder; // Wire, shares the PCA9685 bus
EncoderTracker motor2Encoder; // Wire1, dedicated bus

// Updates rpm from a new reading, handling the 0 <-> 16383 wrap. The first
// reading only primes the tracker and returns false.
bool updateEncoderRpm(EncoderTracker &tracker, uint16_t rawNow,
                     unsigned long nowMs) {
  if (!tracker.hasLastReading) {
    tracker.lastRaw = rawNow;
    tracker.lastUpdateMs = nowMs;
    tracker.hasLastReading = true;
    return false;
  }

  const unsigned long dtMs = nowMs - tracker.lastUpdateMs;
  if (dtMs == 0) {
    return false;
  }

  int32_t delta = static_cast<int32_t>(rawNow) - static_cast<int32_t>(tracker.lastRaw);

  if (delta > (MT6701_COUNTS_PER_REV / 2)) {
    delta -= MT6701_COUNTS_PER_REV;
  } else if (delta < -(MT6701_COUNTS_PER_REV / 2)) {
    delta += MT6701_COUNTS_PER_REV;
  }

  const float revolutions = static_cast<float>(delta) /
                            static_cast<float>(MT6701_COUNTS_PER_REV);
  tracker.rpm = revolutions * (60000.0f / static_cast<float>(dtMs));
  tracker.lastRaw = rawNow;
  tracker.lastUpdateMs = nowMs;
  return true;
}

float rawAngleToDegrees(uint16_t raw) {
  return static_cast<float>(raw) * (360.0f / static_cast<float>(MT6701_COUNTS_PER_REV));
}

float computeForwardBaseDutyPercent(unsigned long elapsedMs) {
  if (elapsedMs < FORWARD_ACCELERATION_TIME_MS) {
    const float progress = static_cast<float>(elapsedMs) /
                           static_cast<float>(FORWARD_ACCELERATION_TIME_MS);
    return START_DUTY_PERCENT +
           ((FORWARD_PEAK_DUTY_PERCENT - START_DUTY_PERCENT) * progress);
  }

  const unsigned long decelerationElapsed =
      elapsedMs - FORWARD_ACCELERATION_TIME_MS;
  const unsigned long decelerationTime =
      FORWARD_PROFILE_TIME_MS - FORWARD_ACCELERATION_TIME_MS;
  const float progress = static_cast<float>(decelerationElapsed) /
                         static_cast<float>(decelerationTime);
  return FORWARD_PEAK_DUTY_PERCENT +
         ((FORWARD_END_DUTY_PERCENT - FORWARD_PEAK_DUTY_PERCENT) * progress);
}

float computeReverseBaseDutyPercent(unsigned long elapsedMs) {
  const float progress = static_cast<float>(elapsedMs) /
                         static_cast<float>(REVERSE_PROFILE_TIME_MS);
  return REVERSE_START_DUTY_PERCENT +
         ((REVERSE_END_DUTY_PERCENT - REVERSE_START_DUTY_PERCENT) * progress);
}

unsigned long lastTelemetryMs = 0;

void maybePrintTelemetry(unsigned long nowMs, uint16_t raw1, uint16_t raw2,
                         float commandedDuty1, float commandedDuty2,
                         float correction1, float correction2) {
  if (nowMs - lastTelemetryMs < TELEMETRY_INTERVAL_MS) {
    return;
  }
  lastTelemetryMs = nowMs;

  Serial.print("M1 angle=");
  Serial.print(rawAngleToDegrees(raw1), 1);
  Serial.print(" rpm=");
  Serial.print(motor1Encoder.rpm, 1);
  Serial.print(" duty=");
  Serial.print(commandedDuty1, 2);
  Serial.print(" corr=");
  Serial.print(correction1, 2);

  Serial.print(" | M2 angle=");
  Serial.print(rawAngleToDegrees(raw2), 1);
  Serial.print(" rpm=");
  Serial.print(motor2Encoder.rpm, 1);
  Serial.print(" duty=");
  Serial.print(commandedDuty2, 2);
  Serial.print(" corr=");
  Serial.println(correction2, 2);
}

// Runs one segment for durationMs. baseDutyFn gives the time-based setpoint,
// and encoder speed matching corrects each motor around it.
void runEncoderCorrectedProfile(bool forward, unsigned long durationMs,
                                float (*baseDutyFn)(unsigned long)) {
  const unsigned long profileStart = millis();

  while (true) {
    const unsigned long nowMs = millis();
    const unsigned long elapsed = nowMs - profileStart;

    if (elapsed >= durationMs) {
      break;
    }

    const float baseDuty = baseDutyFn(elapsed);

    uint16_t raw1 = motor1Encoder.lastRaw;
    uint16_t raw2 = motor2Encoder.lastRaw;
    mt6701ReadAngleRaw(Wire, raw1);
    mt6701ReadAngleRaw(Wire1, raw2);
    updateEncoderRpm(motor1Encoder, raw1, nowMs);
    updateEncoderRpm(motor2Encoder, raw2, nowMs);

    const float averageRpm = (motor1Encoder.rpm + motor2Encoder.rpm) / 2.0f;
    const float correction1 =
        SPEED_MATCH_KP_PERCENT_PER_RPM * (averageRpm - motor1Encoder.rpm);
    const float correction2 =
        SPEED_MATCH_KP_PERCENT_PER_RPM * (averageRpm - motor2Encoder.rpm);

    float commandedDuty1 = baseDuty + correction1;
    float commandedDuty2 = baseDuty + correction2;
    if (commandedDuty1 < 0.0f) commandedDuty1 = 0.0f;
    if (commandedDuty2 < 0.0f) commandedDuty2 = 0.0f;

    const uint16_t pwm1 = pwmValueForPercent(commandedDuty1);
    const uint16_t pwm2 = pwmValueForPercent(commandedDuty2);

    motorDriveIndividual(forward, pwm1, pwm2);
    maybePrintTelemetry(nowMs, raw1, raw2, commandedDuty1, commandedDuty2,
                       correction1, correction2);

    delay(RAMP_UPDATE_INTERVAL_MS);
  }

  // Finish on the profile's end duty, without correction.
  const uint16_t endPwm = pwmValueForPercent(baseDutyFn(durationMs - 1));
  motorDriveIndividual(forward, endPwm, endPwm);
}

void motorForwardProfile() {
  runEncoderCorrectedProfile(true, FORWARD_PROFILE_TIME_MS,
                            computeForwardBaseDutyPercent);
}

void motorReverseProfile() {
  runEncoderCorrectedProfile(false, REVERSE_PROFILE_TIME_MS,
                            computeReverseBaseDutyPercent);
}

void runMotorTest() {
  // Refuse to run open-loop if either encoder is missing.
  const bool encoder1Present = mt6701CheckPresence(Wire);
  const bool encoder2Present = mt6701CheckPresence(Wire1);

  if (!encoder1Present || !encoder2Present) {
    Serial.println("ENCODER FAULT: cannot start test.");
    if (!encoder1Present) {
      Serial.println("  Motor 1 MT6701 (Wire, GP20/GP21) did not ACK.");
    }
    if (!encoder2Present) {
      Serial.println("  Motor 2 MT6701 (Wire1, ENCODER2_SDA_PIN/"
                     "ENCODER2_SCL_PIN) did not ACK.");
    }
    motorActiveBrakeBoth();
    return;
  }

  // Clear the tracking so the first tick of each segment doesn't see a stale
  // reading as a speed spike.
  motor1Encoder.hasLastReading = false;
  motor2Encoder.hasLastReading = false;

  Serial.println("Forward: 5% to 30%, then down to 3% over 5 seconds...");
  motorForwardProfile();

  motor1Encoder.hasLastReading = false;
  motor2Encoder.hasLastReading = false;

  Serial.println("Reverse: accelerating from 3% to 7% over 5 seconds...");
  motorReverseProfile();

  motorActiveBrakeBoth();
  Serial.println("Motor test complete; active brake applied.");
}

bool startButtonWasPressed() {
  if (digitalRead(START_BUTTON_PIN) != LOW) {
    return false;
  }

  delay(BUTTON_DEBOUNCE_TIME_MS);
  return digitalRead(START_BUTTON_PIN) == LOW;
}

void setup() {
  Serial.begin(115200);
  pinMode(START_BUTTON_PIN, INPUT_PULLUP);

  Wire.setSDA(SDA_PIN);
  Wire.setSCL(SCL_PIN);
  Wire.begin();
  Wire.setClock(400000);

  Wire1.setSDA(ENCODER2_SDA_PIN);
  Wire1.setSCL(ENCODER2_SCL_PIN);
  Wire1.begin();
  Wire1.setClock(400000);

  // Put the PCA9685 in a safe state before anything else.
  pcaWriteRegister(PCA9685_MODE1, 0x00);
  pcaWriteRegister(PCA9685_MODE2, 0x04); // totem-pole outputs
  motorCoastBoth();
  pcaSetPwmFrequency(1000);

  Serial.println("Ready. Press the momentary start button on GP15.");
}

void loop() {
  if (!startButtonWasPressed()) {
    motorActiveBrakeBoth();
    delay(50);
    return;
  }

  Serial.println("Start button pressed.");
  runMotorTest();

  // Wait for release so one press doesn't start a second run.
  while (digitalRead(START_BUTTON_PIN) == LOW) {
    delay(10);
  }

  delay(BUTTON_DEBOUNCE_TIME_MS);
}
