# Pusher Robot

A two-wheel robot built for a STEM competition. It pushes forward past a target line, then backs up to a set point.

The firmware drives both wheels with a closed-loop PID controller. Two MT6701 magnetic encoders measure wheel travel, so the robot stops at the commanded distance even when the wheels or battery voltage change.

**CAD:** [View the robot design in Onshape](https://cad.onshape.com/documents/299f5818a612338b9a07fa4c/w/ddd003f91212fb30e6fe7471/e/41376dde1b726e0963889ca8?renderMode=0&uiState=6affbaaf4404645c096092cf)

## 1. Run sequence

The firmware (`firmware/PID_Drive/PID_Drive.ino`) performs one run per start:

1. Press the start button on **GP15**, or send `g` over serial.
2. Wait 2 s.
3. Drive forward `TARGET_CM` from the start position (default **1050 cm**) under PID control.
4. Brake, then wait 2 s at the target.
5. Reverse `REVERSE_CM` from where the forward leg stopped (default **200 cm**) under PID control.
6. Brake and stop.

The robot ends about `TARGET_CM − REVERSE_CM` from the start (default 850 cm).

Within each leg, the robot cruises at `CRUISE` duty, drops to `SLOW_DUTY` inside the last `SLOW_CM`, and uses PID to stop on the target. A heading loop trims the left and right motors so both wheels cover the same distance.

## 2. Repository layout

```
README.md
docs/
  pololu-motor-test.md        Motor test notes (Waveshare driver, 9 V supply, wiring)
  build-logs/                 Arduino IDE build logs (board package and flags)
  hardware/
    Wiring.md                 I2C wiring notes
    ENCODER_WIRING.md         MT6701 encoder wiring and pin assignments
    Hardware_Change_Log.md    Hardware faults, fixes, and follow-up items
  pid/
    PID_Constants.md          Tuned gains and parameters
firmware/
  tests/                      Bring-up sketches, run in stage order (section 3)
  pid/
    PID_Tuner/                Serial-tunable distance PID
  PID_Drive/                  Forward-and-reverse run with tuned constants
  distance_drive/            Earlier distance-drive variants
    analog_encoder/           Analog-output encoder version
    abi_encoder/              ABI quadrature encoder version
  reference/
    pico_motor_test_sdk/      Pico SDK version of the motor test (not an Arduino sketch)
```

Each Arduino sketch sits in a folder with the same name as its `.ino` file, which the Arduino IDE requires.

## 3. Development workflow

Each stage proves one part of the system before the next stage relies on it.

| Stage | Sketch | What it proves |
|---|---|---|
| 1. Power check | `firmware/tests/Board_Power_Test/Board_Power_Test.ino` | The Pico 2 runs and the onboard LED works. |
| 2. Motor check | `firmware/tests/Motor_Test/Motor_Test.ino`, `firmware/tests/pico_motor_test/pico_motor_test.ino` | The motor driver responds, and both motors run and brake. |
| 3. Encoder check | `firmware/tests/Encoder_Test/Encoder_Test.ino`, `firmware/tests/pico_motor_test_encoder/pico_motor_test_encoder.ino` | Both MT6701 encoders answer on the I2C bus, and the reading changes as the wheel turns. |
| 4. Distance check | `firmware/tests/pico_motor_test_encoder_distance/pico_motor_test_encoder_distance.ino`, `firmware/distance_drive/` | The robot drives a set distance measured by the encoders. |
| 5. PID tuning | `firmware/pid/PID_Tuner/PID_Tuner.ino` | You find gains that reach the target quickly with little overshoot. |
| 6. Drive run | `firmware/PID_Drive/PID_Drive.ino` | The tuned gains drive a full forward-and-reverse cycle. |

### 3.1 Tuning the controller

1. Upload `PID_Tuner.ino` to the Pico 2 with the Arduino IDE.
2. Open the Serial Monitor at **115200 baud**. Send `help` to list the commands.
3. Start a run with the button on GP15 or the `g` command.
4. Change one parameter at a time (`kp`, `kd`, `kh`, and so on) and run again. Use `history` to compare the last 10 runs.
5. Copy the final values into the constants at the top of `PID_Drive.ino`.
6. Record the final values in `docs/pid/PID_Constants.md`.

Tune in this order: `kp`, then `kd`, then `kh`, then `mind`, then `ki`, then `cruise`.

### 3.2 Recording hardware changes

Log every hardware fault, rewire, or repair in `docs/hardware/Hardware_Change_Log.md`. Use the entry template at the end of that file. Each entry lists the issue, the actions taken, the result, and the follow-up items.

### 3.3 Version control

- Commit each firmware change with a short message that names the change.
- Do not overwrite a working sketch with an experiment. Create a new file or folder instead.

## 4. Hardware

### 4.1 Parts list

| Part | Quantity | Role |
|---|---|---|
| Raspberry Pi Pico 2 (RP2350) | 1 | Main microcontroller. Runs the firmware, reads the encoders, and commands the motor driver. |
| Waveshare Pico-Motor-Driver | 1 | Motor driver board with a PCA9685 PWM controller and a TB6612FNG H-bridge (switches motor direction and braking). |
| Pololu #998 50:1 Micro Metal Gearmotor HP, 6 V | 2 | Drive motors. The 50:1 gearbox trades speed for torque. |
| MT6701 magnetic encoder (I2C mode) | 2 | Measures the angle of each wheel. One per wheel. |
| Wheels, 60.325 mm diameter (189.52 mm circumference) | 2 | Convert wheel rotation to distance. |
| 9 V supply | 1 | Powers the motor driver. See the open issues on voltage. |
| Momentary push button | 1 | Starts a run (GP15). |
| USB cable | 1 | Uploads firmware and provides the Serial Monitor. |

### 4.2 Design notes

- **Two I2C buses.** Both MT6701 encoders share the fixed address `0x06`, so they cannot share one bus. The left encoder uses I2C0 with the PCA9685 (address `0x40`). The right encoder uses a second bus.
- **PWM motor drive.** The PCA9685 generates the PWM signals at about 1 kHz and frees the Pico's GPIO pins. The TB6612FNG is rated for about 1.2 A continuous current.
- **Duty limit.** `MAX_DUTY` caps the duty cycle at the motors' rated voltage divided by the supply voltage. The current code uses `6.0 / 7.8`. See the open issues.
- **Minimum duty.** Each motor needs a minimum duty cycle (stiction floor) before it starts moving. `MIN_DUTY` sets this value in `PID_Drive.ino`.
- **Encoder resolution.** The MT6701 is a 14-bit sensor, which gives 16,384 counts per revolution.

### 4.3 Pin map

| Signal | Pico 2 pin | Used by |
|---|---|---|
| I2C0 SDA (PCA9685 and left encoder) | GP20 | All firmware |
| I2C0 SCL (PCA9685 and left encoder) | GP21 | All firmware |
| Right encoder SDA | GP2 (PID code) or GP18 (encoder test code) | See open issues |
| Right encoder SCL | GP3 (PID code) or GP19 (encoder test code) | See open issues |
| Start button | GP15 | All firmware |
| Onboard LED | `LED_BUILTIN` | `PID_Drive.ino` status codes |

PCA9685 channel map (Waveshare board):

| Motor | PWM | IN1 | IN2 |
|---|---|---|---|
| Motor 1 (output C) | 6 | 7 | 8 |
| Motor 2 (output D) | 9 | 10 | 11 |

The PCA9685 uses I2C address `0x40`. The MT6701 encoders use I2C address `0x06`.

### 4.4 Power and wiring rules

- Power the MT6701 encoders from **3.3 V**, not 5 V.
- Do not connect the 9 V supply to a Pico GPIO pin.
- Connect the supply ground, the driver ground, and the Pico ground together.
- The Pico 2 GPIO pins are **not 5 V tolerant**. Use 3.3 V logic only.
- Add a current limit or fuse to the motor supply. A stalled Pololu #998 can draw about 1.6 A at 6 V.

## 5. Open issues

- **Right encoder pins conflict.** `PID_Drive.ino`, `PID_Tuner.ino`, and `Wiring.md` use GP2/GP3 for the right encoder. `ENCODER_WIRING.md` and `pico_motor_test_encoder.ino` use GP18/GP19. Confirm which pins are wired before running the code, then update the other files to match.
- **Supply voltage does not match.** The README and `pico_motor_test` assume a 9 V supply (`6 V / 9 V`). `PID_Drive.ino` uses `MAX_DUTY = 6.0 / 7.8` with the comment "7.8 V battery". Confirm the supply and update the constant.
- **Minimum duty values differ.** `Hardware_Change_Log.md` records a minimum reverse duty of 0.055. `PID_Drive.ino` uses `MIN_DUTY = 0.085`, and `PID_Constants.md` now matches the code. Test on the robot, pick one value, and record it in the log.
- **USB detection root cause is unknown.** The Pico 2 was not detected over USB in one wiring setup. The wiring was reworked and the Pico works now, but the original fault was not identified. See the follow-up items in `Hardware_Change_Log.md`.
- **Wiring diagram is missing.** `Wiring.md` should show the current I2C wiring. Add photos or a diagram.
- **Motor voltage is not regulated.** The motors run from a 9 V supply with PWM limiting. A 6 V buck converter would give more accurate speed.

## 6. Safety

- Test with the wheels off the ground or the robot unloaded first.
- Keep a way to cut motor power quickly.
- The firmware brakes the motors when a run ends. Remove power after the robot stops.
- If a wheel moves backward during a run (more than `BACKWARD_ABORT_MM`), the firmware aborts the run. Check the encoder sign (`ENC_SIGN_L`, `ENC_SIGN_R`) and the motor direction (`LEFT_MOTOR_IS_M1`).
- Do the first test with the motor unloaded, a current-limited supply if available, and a way to disconnect power quickly. The final active-brake state is intentional; remove power after confirming both motors have stopped.
