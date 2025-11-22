#include <Wire.h>
#include <Arduino.h>

// ======================= MODE & STEP =====================
#define MODE 3              
#define STEP_TARGET 10.0    

class KalmanFilter {
  public:
    float angle = 0.0f; 
    float bias = 0.0f;
    float P[2][2] = {{0, 0}, {0, 0}};

    float Q_angle = 0.001f;
    float Q_bias  = 0.003f;
    float R_measure = 0.03f;

    bool initialized = false;

    float update(float accelAngle, float gyroRate, float dt) {

      if (!initialized) {
        angle = accelAngle;
        bias = 0;
        P[0][0] = P[0][1] = P[1][0] = P[1][1] = 0;
        initialized = true;
        return angle;
      }

      // Predict
      angle += dt * (gyroRate - bias);

      P[0][0] += dt * (dt*P[1][1] - P[0][1] - P[1][0] + Q_angle);
      P[0][1] -= dt * P[1][1];
      P[1][0] -= dt * P[1][1];
      P[1][1] += Q_bias * dt;

      // Update
      float S = P[0][0] + R_measure;
      float K0 = P[0][0] / S;
      float K1 = P[1][0] / S;

      float y = accelAngle - angle;

      angle += K0 * y;
      bias  += K1 * y;

      float P00 = P[0][0];
      float P01 = P[0][1];

      P[0][0] -= K0 * P00;
      P[0][1] -= K0 * P01;
      P[1][0] -= K1 * P00;
      P[1][1] -= K1 * P01;

      return angle;
    }
};


class MPU6050 {
  private:
    const uint8_t MPU_ADDR = 0x68;
    double aX, aY, aZ;
    double gX, gY, gZ;
    //float suhu;
    KalmanFilter kalman;
    float fusedAngle = 0.0f;
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
      //suhu = (tempRaw / 340.0) + 36.53;
      gX = gyro_x / 65.5;
      gY = gyro_y / 65.5;
      gZ = gyro_z / 65.5;

      unsigned long now = millis();
      float dt = (now - lastMillis) / 1000.0; //f;
      if (dt <= 0) dt = 0.001;

      float accelAngle = atan2(aY, aZ) * 180.0 / PI;
      float gyroRate = gX; //* dt;

      //const float alpha = 0.98;
      //fusedAngle = alpha * (fusedAngle + gyroAngle) + (1 - alpha) * accelAngle;
      fusedAngle = kalman.update(accelAngle, gyroRate, dt);

      lastMillis = now;
    }

    float getAngle() {
      return fusedAngle;
    }

  private:
    void writeRegister(uint8_t reg, uint8_t data) {
      Wire.beginTransmission(MPU_ADDR);
      Wire.write(reg);
      Wire.write(data);
      Wire.endTransmission();
    }
};

// ======================= OBJECT ============================
MPU6050 imu;

// ======================= SERVO (LEDC – ESP32) ================
const int servoPin = 4;
const int pwmChannel = 0;
const int pwmFrequency = 50;         // servo = 50Hz
const int pwmResolution = 16;        // 16-bit resolution

void writeServoAngle(float angle) {
  if (angle < 0) angle = 0;
  if (angle > 180) angle = 180;

  int pulseMicros = map(angle, 0, 180, 500, 2500);
  int duty = (pulseMicros * 65535) / 20000;  
  ledcWrite(pwmChannel, duty);
}

// ======================= PID ============================
float Kp = 20.0;
float Ki = 0.0;
float Kd = 10.0;

float setpoint = 0.0;
float integral = 0.0;
float lastError = 0.0;

unsigned long lastPID = 0;

// ======================= SETUP ============================
void setup() {
  Serial.begin(115200);

  Wire.begin();
  imu.begin();

  ledcSetup(pwmChannel, pwmFrequency, pwmResolution);
  ledcAttachPin(servoPin, pwmChannel);
}

// ======================= LOOP =============================
void loop() {
  imu.readData();

  unsigned long now = millis();
  float dt = (now - lastPID) / 1000.0f;
  if (dt <= 0) dt = 0.001;
  if (dt > 0.05) dt = 0.05;  // stabilizer 20Hz–200Hz

  if (now < 2000) setpoint = 0.0;
  else setpoint = STEP_TARGET;

  float measured = imu.getAngle();
  float error = setpoint - measured;

  // P
  float P = Kp * error;

  // I
  integral += error * dt;
  if (integral > 300) integral = 300;
  if (integral < -300) integral = -300;
  float I = Ki * integral;

  // D
  float derivative = (error - lastError) / dt;
  if (derivative > 300) derivative = 300;
  if (derivative < -300) derivative = -300;
  float D = Kd * derivative;

  float output = P + I + D;
  if (output > 200) output = 200;
  if (output < -200) output = -200;

  float servoAngle = map(output, -200, 200, 0, 180);
  writeServoAngle(servoAngle);

  Serial.print("Set: "); Serial.print(setpoint);
  Serial.print(" | Angle: "); Serial.print(measured);
  Serial.print(" | Out: "); Serial.print(output);
  Serial.print(" | Servo: "); Serial.println(servoAngle);

  lastError = error;
  lastPID = now;

  delay(5);
}
