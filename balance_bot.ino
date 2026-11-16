// Simple Self-Balancing Robot
// Arduino Uno/Nano + MPU6050 + L298N (or TB6612) dual motor driver
// No external libraries required (uses Wire only).
//
// Wiring:
//   MPU6050: VCC->5V, GND->GND, SDA->A4, SCL->A5
//   L298N:   ENA->D5, IN1->D7, IN2->D8   (left motor)
//            ENB->D6, IN3->D9, IN4->D10  (right motor)
//            Motor supply from battery; tie battery GND to Arduino GND.

#include <Wire.h>

// ---------- Tuning (start here) ----------
float Kp = 25.0;      // Raise until it oscillates, then back off ~30%
float Ki = 0.5;       // Add slowly to remove drift
float Kd = 0.8;       // Add to damp the oscillation
float setpoint = 0.0; // Balance angle (deg). Adjust if it creeps one way.
const int   MIN_PWM   = 30;   // Overcomes motor deadband/static friction
const float FALL_ANGLE = 40.0; // Beyond this, cut motors (it fell over)
const float ALPHA = 0.98;      // Complementary filter: gyro vs accel trust

// ---------- Pins ----------
const int ENA = 5, IN1 = 7, IN2 = 8;
const int ENB = 6, IN3 = 9, IN4 = 10;

// ---------- MPU6050 ----------
const int MPU = 0x68;
float gyroBias = 0;
float angle = 0;

// ---------- PID state ----------
float integral = 0, lastError = 0;
unsigned long lastTime;
const unsigned long LOOP_US = 5000; // 200 Hz control loop

void mpuWrite(byte reg, byte val) {
  Wire.beginTransmission(MPU);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

// Reads accel X, accel Z, gyro Y (raw)
void readMPU(int16_t &ax, int16_t &az, int16_t &gy) {
  Wire.beginTransmission(MPU);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 14, true);
  ax = Wire.read() << 8 | Wire.read();
  Wire.read(); Wire.read();            // accel Y (unused)
  az = Wire.read() << 8 | Wire.read();
  Wire.read(); Wire.read();            // temp (unused)
  Wire.read(); Wire.read();            // gyro X (unused)
  gy = Wire.read() << 8 | Wire.read();
  Wire.read(); Wire.read();            // gyro Z (unused)
}

float accelAngle(int16_t ax, int16_t az) {
  return atan2(-(float)ax, (float)az) * 180.0 / PI;
}

void setMotors(int speed) {
  speed = constrain(speed, -255, 255);
  if (speed != 0) speed += (speed > 0 ? MIN_PWM : -MIN_PWM);
  speed = constrain(speed, -255, 255);

  bool fwd = speed >= 0;
  digitalWrite(IN1, fwd);  digitalWrite(IN2, !fwd);
  digitalWrite(IN3, fwd);  digitalWrite(IN4, !fwd);
  analogWrite(ENA, abs(speed));
  analogWrite(ENB, abs(speed));
}

void stopMotors() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}

void setup() {
  Serial.begin(115200);
  pinMode(ENA, OUTPUT); pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT); pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  stopMotors();

  Wire.begin();
  Wire.setClock(400000);
  mpuWrite(0x6B, 0x00); // Wake up
  mpuWrite(0x1A, 0x03); // Digital low-pass filter ~44 Hz
  mpuWrite(0x1B, 0x00); // Gyro ±250 °/s  (131 LSB per °/s)
  mpuWrite(0x1C, 0x00); // Accel ±2 g
  delay(100);

  // Calibrate gyro: keep robot STILL for ~1 second
  Serial.println("Calibrating... hold still");
  long sum = 0;
  int16_t ax, az, gy;
  for (int i = 0; i < 500; i++) {
    readMPU(ax, az, gy);
    sum += gy;
    delay(2);
  }
  gyroBias = sum / 500.0;

  readMPU(ax, az, gy);
  angle = accelAngle(ax, az);  // Start filter at accel reading
  lastTime = micros();
  Serial.println("Ready");
}

void loop() {
  unsigned long now = micros();
  if (now - lastTime < LOOP_US) return;
  float dt = (now - lastTime) / 1000000.0;
  lastTime = now;

  // --- Sense ---
  int16_t ax, az, gy;
  readMPU(ax, az, gy);
  float gyroRate = (gy - gyroBias) / 131.0;              // deg/s
  angle = ALPHA * (angle + gyroRate * dt) + (1 - ALPHA) * accelAngle(ax, az);

  // --- Safety: fallen over ---
  if (abs(angle) > FALL_ANGLE) {
    stopMotors();
    integral = 0;
    lastError = 0;
    return;
  }

  // --- PID ---
  float error = angle - setpoint;
  integral = constrain(integral + error * dt, -100, 100); // anti-windup
  float derivative = gyroRate;  // cleaner than d(error)/dt
  float output = Kp * error + Ki * integral + Kd * derivative;

  setMotors((int)output);

  // --- Debug (comment out once tuned; Serial slows the loop) ---
  static int n = 0;
  if (++n >= 20) { n = 0; Serial.print(angle); Serial.print('\t'); Serial.println(output); }
}
