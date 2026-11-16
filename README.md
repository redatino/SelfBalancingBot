# Self-Balancing Robot

Simple Arduino self-balancing robot using an MPU6050 IMU, complementary filter, and PID control. No external libraries (uses `Wire` only).

## Hardware
- Arduino Uno/Nano
- MPU6050
- L298N or TB6612FNG dual motor driver
- 2x geared DC motors + battery

## Wiring
| Part | Pin |
|---|---|
| MPU6050 SDA / SCL | A4 / A5 |
| Left motor | ENA→D5, IN1→D7, IN2→D8 |
| Right motor | ENB→D6, IN3→D9, IN4→D10 |
| Grounds | Battery GND tied to Arduino GND |

### Using a TB6612FNG instead of an L298N
The sketch doesn't change; only the wiring does. ENA/ENB→PWMA/PWMB, IN1–IN4→AIN1/AIN2/BIN1/BIN2, battery→VM, Arduino 5V→VCC. **Tie STBY to 5V** or the motors won't run.

## Setup
1. **Mounting:** assumes MPU X axis forward, Z up. Tilt forward → angle should read positive (Serial @ 115200). If not, flip the sign of `ax` in `accelAngle()`.
2. **Motor direction:** tilt forward → wheels should drive forward. If not, swap motor leads or negate `output`.
3. **Setpoint:** hold at the balance point, read the angle, put it in `setpoint`.

## Tuning
1. `Ki = Kd = 0`. Raise `Kp` until it oscillates, then cut ~30%.
2. Raise `Kd` until the oscillation damps out.
3. Add a small `Ki` (0.1–1) to stop drift.
4. Raise `MIN_PWM` if the motors hum without moving on small corrections.
