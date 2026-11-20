// Pololu #998 motor test on the Pico SDK (no Arduino core).
// Waveshare Pico-Motor-Driver: PCA9685 PWM controller over I2C driving a
// TB6612FNG dual H-bridge.
//
// Sequence: forward for 3 s, reverse immediately for 5 s, then active brake.
//
// Motor 1 mapping used here: PCA9685 channels 0 = PWM, 1 = IN1, 2 = IN2.
// Confirm the channel labels in the schematic for your board revision before
// connecting a motor. If forward and reverse are swapped, exchange
// MOTOR_IN1_CHANNEL and MOTOR_IN2_CHANNEL.

#include <algorithm>
#include <cstdint>

#include "hardware/i2c.h"
#include "pico/stdlib.h"

namespace {

constexpr i2c_inst_t* I2C_PORT = i2c0;
constexpr uint SDA_PIN = 4;
constexpr uint SCL_PIN = 5;
constexpr uint I2C_BAUD_HZ = 400'000;

// Change this if the board's address jumpers are moved.
constexpr uint8_t PCA9685_ADDRESS = 0x40;

constexpr uint8_t MOTOR_PWM_CHANNEL = 0;
constexpr uint8_t MOTOR_IN1_CHANNEL = 1;
constexpr uint8_t MOTOR_IN2_CHANNEL = 2;

constexpr uint16_t PCA9685_FULL_SCALE = 4095;

// The motor is rated for 6 V and the supply is 9 V.
constexpr float MOTOR_RATED_VOLTAGE = 6.0f;
constexpr float SUPPLY_VOLTAGE = 9.0f;

// Set below the 6/9 = 66.7% ceiling on purpose. Raise it only after checking
// current, temperature, gearing and load.
constexpr float TEST_DUTY_PERCENT = 60.0f;

constexpr uint32_t FORWARD_TIME_MS = 3'000;
constexpr uint32_t REVERSE_TIME_MS = 5'000;

constexpr uint8_t PCA9685_MODE1 = 0x00;
constexpr uint8_t PCA9685_MODE2 = 0x01;
constexpr uint8_t PCA9685_PRESCALE = 0xFE;
constexpr uint8_t PCA9685_LED0_ON_L = 0x06;

enum class Direction : uint8_t {
  Forward,
  Reverse,
};

void pca9685_write_register(uint8_t reg, uint8_t value) {
  const uint8_t data[] = {reg, value};
  i2c_write_blocking(I2C_PORT, PCA9685_ADDRESS, data, sizeof(data), false);
}

void pca9685_set_channel(uint8_t channel, uint16_t on, uint16_t off) {
  if (channel > 15) {
    return;
  }

  const uint8_t register_address =
      static_cast<uint8_t>(PCA9685_LED0_ON_L + (4 * channel));
  const uint8_t data[] = {
      register_address,
      static_cast<uint8_t>(on & 0xFF),
      static_cast<uint8_t>((on >> 8) & 0x0F),
      static_cast<uint8_t>(off & 0xFF),
      static_cast<uint8_t>((off >> 8) & 0x0F),
  };

  i2c_write_blocking(I2C_PORT, PCA9685_ADDRESS, data, sizeof(data), false);
}

// Forces a channel fully on or fully off using the FULL_ON/FULL_OFF bit (bit 4)
// in the ON_H / OFF_H registers.
void pca9685_set_output(uint8_t channel, bool enabled) {
  if (channel > 15) {
    return;
  }

  const uint8_t register_address =
      static_cast<uint8_t>(PCA9685_LED0_ON_L + (4 * channel));
  const uint8_t data[] = {
      register_address,
      0x00,
      enabled ? 0x10 : 0x00,
      0x00,
      enabled ? 0x00 : 0x10,
  };

  i2c_write_blocking(I2C_PORT, PCA9685_ADDRESS, data, sizeof(data), false);
}

void pca9685_set_pwm_frequency(uint16_t frequency_hz) {
  // The PCA9685 oscillator runs at 25 MHz.
  const float prescale_value =
      (25'000'000.0f / (4096.0f * frequency_hz)) - 1.0f;
  const uint8_t prescale = static_cast<uint8_t>(
      std::clamp(prescale_value, 3.0f, 255.0f));

  uint8_t mode1 = 0;
  const uint8_t mode1_register[] = {PCA9685_MODE1};
  i2c_write_blocking(I2C_PORT, PCA9685_ADDRESS, mode1_register, 1, true);
  i2c_read_blocking(I2C_PORT, PCA9685_ADDRESS, &mode1, 1, false);

  // PRE_SCALE can only be written in sleep mode.
  pca9685_write_register(PCA9685_MODE1, static_cast<uint8_t>(
      (mode1 & 0x7F) | 0x10));
  pca9685_write_register(PCA9685_PRESCALE, prescale);
  pca9685_write_register(PCA9685_MODE1, mode1);
  sleep_us(500);
  pca9685_write_register(PCA9685_MODE1, static_cast<uint8_t>(mode1 | 0xA0));
}

uint16_t rated_voltage_pwm_limit() {
  const float limit = (MOTOR_RATED_VOLTAGE / SUPPLY_VOLTAGE) *
                      static_cast<float>(PCA9685_FULL_SCALE);
  return static_cast<uint16_t>(std::clamp(
      limit, 0.0f, static_cast<float>(PCA9685_FULL_SCALE)));
}

uint16_t test_pwm_value() {
  const float requested = (TEST_DUTY_PERCENT / 100.0f) *
                          static_cast<float>(PCA9685_FULL_SCALE);
  return static_cast<uint16_t>(std::clamp(
      requested, 0.0f, static_cast<float>(rated_voltage_pwm_limit())));
}

void motor_coast() {
  pca9685_set_channel(MOTOR_PWM_CHANNEL, 0, 0);
  pca9685_set_output(MOTOR_IN1_CHANNEL, false);
  pca9685_set_output(MOTOR_IN2_CHANNEL, false);
}

void motor_drive(Direction direction, uint16_t pwm_value) {
  const bool forward = direction == Direction::Forward;

  // Set the direction pins before enabling PWM so the bridge never runs with an
  // undefined direction.
  pca9685_set_output(MOTOR_IN1_CHANNEL, forward);
  pca9685_set_output(MOTOR_IN2_CHANNEL, !forward);
  pca9685_set_channel(MOTOR_PWM_CHANNEL, 0, pwm_value);
}

// TB6612FNG short brake: IN1 = HIGH, IN2 = HIGH, PWM = HIGH. Stops faster than
// coasting.
void motor_active_brake() {
  pca9685_set_output(MOTOR_IN1_CHANNEL, true);
  pca9685_set_output(MOTOR_IN2_CHANNEL, true);
  pca9685_set_output(MOTOR_PWM_CHANNEL, true);
}

void motor_test_sequence() {
  const uint16_t pwm = test_pwm_value();

  motor_drive(Direction::Forward, pwm);
  sleep_ms(FORWARD_TIME_MS);

  motor_drive(Direction::Reverse, pwm);
  sleep_ms(REVERSE_TIME_MS);

  motor_active_brake();
}

}  // namespace

int main() {
  stdio_init_all();

  i2c_init(I2C_PORT, I2C_BAUD_HZ);
  gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
  gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);
  gpio_pull_up(SDA_PIN);
  gpio_pull_up(SCL_PIN);

  // Keep the bridge inactive while the PCA9685 is configured.
  motor_coast();

  pca9685_write_register(PCA9685_MODE1, 0x00);
  pca9685_write_register(PCA9685_MODE2, 0x04);  // totem-pole outputs
  pca9685_set_pwm_frequency(1'000);

  // Run once after boot. The brake state is held indefinitely afterwards.
  motor_test_sequence();

  while (true) {
    tight_loop_contents();
  }
}
