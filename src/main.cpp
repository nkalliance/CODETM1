#include <Wire.h>
#include <Arduino.h>

// ======================= MODE & STEP =====================
// 0 = P only, 1 = PI, 2 = PD, 3 = PID
#define MODE 3              

// Setpoint akan 0 derajat dulu, lalu pindah ke STEP_TARGET setelah 2 detik
#define STEP_TARGET 10.0    

class MPU6050 {
  private:
    const uint8_t MPU_ADDR = 0x68;
    double aX, aY, aZ;
    double gX, gY, gZ;
    float suhu;

    float fusedAngle = 0.0;
    unsigned long lastMillis = 0;

  public:
    void begin() {
      Wire.begin();
      writeRegister(0x6B, 0x00);
      writeRegister(0x1C, 0x08); 
      writeRegister(0x1B, 0x08); 
      lastMillis = millis();
    }

    void readData() {
      Wire.beginTransmission(MPU_ADDR);
      Wire.write(0x3B);
      Wire.endTransmission();
      Wire.requestFrom(MPU_ADDR, 14);

      int16_t accel_x = (Wire.read() << 8) | Wire.read();
      int16_t accel_y = (Wire.read() << 8) | Wire.read();
      int16_t accel_z = (Wire.read() << 8) | Wire.read();
      int16_t tempRaw = (Wire.read() << 8) | Wire.read();
      int16_t gyro_x  = (Wire.read() << 8) | Wire.read();
      int16_t gyro_y  = (Wire.read() << 8) | Wire.read();
      int16_t gyro_z  = (Wire.read() << 8) | Wire.read();

      aX = accel_x / 8192.0;
      aY = accel_y / 8192.0;
      aZ = accel_z / 8192.0;
      suhu = (tempRaw / 340.0) + 36.53;
      gX = gyro_x / 65.5;
      gY = gyro_y / 65.5;
      gZ = gyro_z / 65.5;

      unsigned long now = millis();
      float dt = (now - lastMillis) / 1000.0f;
      if (dt <= 0) dt = 0.001;

      float accelAngle = atan2(aY, aZ) * 180.0 / PI;
      float gyroAngle = gX * dt;

      const float alpha = 0.98;
      fusedAngle = alpha * (fusedAngle + gyroAngle) + (1 - alpha) * accelAngle;

      lastMillis = now;
    }

    float getAngle() {
      return fusedAngle;
    }

    void printData() {
      Serial.print("Angle: "); Serial.println(fusedAngle);
    }

  private:
    void writeRegister(uint8_t reg, uint8_t data) {
      Wire.beginTransmission(MPU_ADDR);
      Wire.write(reg);
      Wire.write(data);
      Wire.endTransmission();
    }
};

MPU6050 imu;
const int servoPin = 4;

void writeServoMicros(int pin, int pulseMicros) {
  digitalWrite(pin, HIGH);
  delayMicroseconds(pulseMicros);
  digitalWrite(pin, LOW);
  delayMicroseconds(20000 - pulseMicros); 
}

int angleToPulse(int angle) {
  if (angle < 0) angle = 0;
  if (angle > 180) angle = 180;
  return map(angle, 0, 180, 1000, 2000);
}

// ======================= PID ============================
float Kp = 8.0;
float Ki = 0.0;
float Kd = 1.2;

float setpoint = 1.0;
float integral = 0.0;
float lastError = 0.0;

unsigned long lastPID = 0;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  imu.begin();

  pinMode(servoPin, OUTPUT);
}

void loop() {
  imu.readData();

  unsigned long now = millis();
  float dt = (now - lastPID) / 1000.0f;
  if (dt <= 0) dt = 0.001;
  if (dt > 0.2) dt = 0.02;

  // ===== STEP INPUT: 0 derajat dulu, lalu loncat ke STEP_TARGET =====
  if (now < 2000) {
    setpoint = 0.0;
  } else {
    setpoint = STEP_TARGET;
  }

  float measured = imu.getAngle();
  float error = setpoint - measured;

  // ===== P TERM (selalu aktif) =====
  float P = Kp * error;

  // ===== I TERM (aktif hanya di MODE 1 & 3) =====
  integral += error * dt;
  if (integral > 300) integral = 300;
  if (integral < -300) integral = -300;
  float I = 0;
  if (MODE == 1 || MODE == 3) {
    I = Ki * integral;
  }

  // ===== D TERM (aktif hanya di MODE 2 & 3) =====
  float derivative = (error - lastError) / dt;
  float D = 0;
  if (MODE == 2 || MODE == 3) {
    D = Kd * derivative;
  }

  float output = P + I + D;

  if (output > 90) output = 90;
  if (output < -90) output = -90;

  float servoAngle = 90 + output;

  int pulseMicros = angleToPulse((int)servoAngle);
  writeServoMicros(servoPin, pulseMicros);

  Serial.print("Set: "); Serial.print(setpoint);
  Serial.print(" | Angle: "); Serial.print(measured);
  Serial.print(" | Out: "); Serial.print(output);
  Serial.print(" | Servo: "); Serial.println(servoAngle);

  lastError = error;
  lastPID = now;
  delay(10);
}
