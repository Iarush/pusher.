/*
  Pololu #998 motor test (open loop)
  Raspberry Pi Pico 2 + Waveshare Pico-Motor-Driver
  Arduino core: Earle F. Philhower III RP2040/RP2350 core

  Sequence:
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
*/

#include <Wire.h>

constexpr uint8_t SDA_PIN = 20;
constexpr uint8_t SCL_PIN = 21;
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

constexpr uint8_t PCA9685_MODE1 = 0x00;
constexpr uint8_t PCA9685_MODE2 = 0x01;
constexpr uint8_t PCA9685_PRESCALE = 0xFE;
constexpr uint8_t PCA9685_LED0_ON_L = 0x06;

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

// Writes channels 6 through 11 in one transaction so both motors receive the
// same PWM and direction at the same moment.
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

void motorDriveBoth(bool forward, uint16_t pwmValue) {
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

  setPwm(pwmValue, onValues[motor1PwmIndex], offValues[motor1PwmIndex]);
  setPwm(pwmValue, onValues[motor2PwmIndex], offValues[motor2PwmIndex]);

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

void motorForwardProfile() {
  const uint16_t startPwm = pwmValueForPercent(START_DUTY_PERCENT);
  const uint16_t peakPwm = pwmValueForPercent(FORWARD_PEAK_DUTY_PERCENT);
  const uint16_t endPwm = pwmValueForPercent(FORWARD_END_DUTY_PERCENT);
  const unsigned long profileStart = millis();

  while (true) {
    const unsigned long elapsed = millis() - profileStart;

    if (elapsed >= FORWARD_PROFILE_TIME_MS) {
      break;
    }

    uint16_t pwmValue;

    if (elapsed < FORWARD_ACCELERATION_TIME_MS) {
      const float progress = static_cast<float>(elapsed) /
                             static_cast<float>(FORWARD_ACCELERATION_TIME_MS);

      pwmValue =
          static_cast<uint16_t>(startPwm + ((peakPwm - startPwm) * progress));
    } else {
      const unsigned long decelerationElapsed =
          elapsed - FORWARD_ACCELERATION_TIME_MS;

      const unsigned long decelerationTime =
          FORWARD_PROFILE_TIME_MS - FORWARD_ACCELERATION_TIME_MS;

      const float progress = static_cast<float>(decelerationElapsed) /
                             static_cast<float>(decelerationTime);

      pwmValue =
          static_cast<uint16_t>(peakPwm + ((endPwm - peakPwm) * progress));
    }

    motorDriveBoth(true, pwmValue);
    delay(RAMP_UPDATE_INTERVAL_MS);
  }

  motorDriveBoth(true, endPwm);
}

void motorReverseProfile() {
  const uint16_t startPwm = pwmValueForPercent(REVERSE_START_DUTY_PERCENT);
  const uint16_t endPwm = pwmValueForPercent(REVERSE_END_DUTY_PERCENT);
  const unsigned long profileStart = millis();

  while (true) {
    const unsigned long elapsed = millis() - profileStart;

    if (elapsed >= REVERSE_PROFILE_TIME_MS) {
      break;
    }

    const float progress = static_cast<float>(elapsed) /
                           static_cast<float>(REVERSE_PROFILE_TIME_MS);

    const uint16_t pwmValue =
        static_cast<uint16_t>(startPwm + ((endPwm - startPwm) * progress));

    motorDriveBoth(false, pwmValue);
    delay(RAMP_UPDATE_INTERVAL_MS);
  }

  motorDriveBoth(false, endPwm);
}

void motorActiveBrakeBoth() {
  // TB6612FNG short brake: IN1 = HIGH, IN2 = HIGH, PWM = HIGH.
  uint16_t onValues[MOTOR_CHANNEL_COUNT] = {0};
  uint16_t offValues[MOTOR_CHANNEL_COUNT] = {0};

  for (uint8_t channel = 0; channel < MOTOR_CHANNEL_COUNT; ++channel) {
    setFullOn(onValues[channel], offValues[channel]);
  }

  pcaWriteMotorChannels(onValues, offValues);
}

void runMotorTest() {
  Serial.println("Forward: 5% to 30%, then down to 3% over 5 seconds...");
  motorForwardProfile();

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
