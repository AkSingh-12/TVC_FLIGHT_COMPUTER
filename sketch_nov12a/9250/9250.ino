 #include <Wire.h>
#include <Servo.h>
#include <MPU9250.h>       // Hideakitai MPU9250 library
#include <Adafruit_BMP085.h>

// Kalman filter struct
struct KalmanFilter {
  float angle = 0;
  float bias = 0;
  float P[2][2] = {{1, 0}, {0, 1}};
};

// PID struct
struct PID {
  float kp, ki, kd;
  float integral = 0;
  float prev_error = 0;
};

// Hardware
MPU9250 mpu;
Adafruit_BMP085 bmp;
Servo servoX, servoY;

// Filters and Controllers
KalmanFilter kalX, kalY;
PID pidX = {1.2, 0.02, 0.1};  
PID pidY = {1.2, 0.02, 0.1};  

// Time tracking
unsigned long prevTime = 0;

// Kalman filter update
float kalmanUpdate(KalmanFilter &kf, float newAngle, float newRate, float dt) {
  float Q_angle = 0.001, Q_bias = 0.003, R_measure = 0.03;

  // Predict
  kf.angle += dt * (newRate - kf.bias);
  kf.P[0][0] += dt * (dt * kf.P[1][1] - kf.P[0][1] - kf.P[1][0] + Q_angle);
  kf.P[0][1] -= dt * kf.P[1][1];
  kf.P[1][0] -= dt * kf.P[1][1];
  kf.P[1][1] += Q_bias * dt;

  // Update
  float y = newAngle - kf.angle;
  float S = kf.P[0][0] + R_measure;
  float K0 = kf.P[0][0] / S;
  float K1 = kf.P[1][0] / S;

  kf.angle += K0 * y;
  kf.bias += K1 * y;

  float P00_temp = kf.P[0][0];
  float P01_temp = kf.P[0][1];

  kf.P[0][0] -= K0 * P00_temp;
  kf.P[0][1] -= K0 * P01_temp;
  kf.P[1][0] -= K1 * P00_temp;
  kf.P[1][1] -= K1 * P01_temp;

  return kf.angle;
}

// PID control
float computePID(PID &pid, float setpoint, float measured, float dt) {
  float error = setpoint - measured;
  pid.integral += error * dt;
  float derivative = (error - pid.prev_error) / dt;
  pid.prev_error = error;
  return pid.kp * error + pid.ki * pid.integral + pid.kd * derivative;
}

void setup() {
  Serial.begin(115200);
  Wire.begin();

  // WHO_AM_I register for MPU9250
  const uint8_t WHO_AM_I_REG = 0x75;
  const uint8_t EXPECTED_WHOAMI = 0x71; // or 0x70 for some MPU9255 variants

  Serial.println("Initializing MPU9250...");

  if (!mpu.setup(0x68)) {
    Serial.println("MPU9250 setup failed. Reading WHO_AM_I...");
    uint8_t whoami = mpu.read_byte(WHO_AM_I_REG);
    Serial.print("WHO_AM_I: 0x"); Serial.println(whoami, HEX);
    if (whoami != EXPECTED_WHOAMI && whoami != 0x70) {
      Serial.println("MPU9250 not connected or wrong device!");
      while (1);
    }
  }

  Serial.println("MPU9250 ready.");

  // BMP085 init
  if (!bmp.begin()) {
    Serial.println("BMP085 not detected!");
    while (1);
  }
  Serial.println("BMP085 ready.");

  // Servo setup
  servoX.attach(5);
  servoY.attach(6);

  prevTime = millis();
}

void loop() {
  unsigned long now = millis();
  float dt = (now - prevTime) / 1000.0;
  prevTime = now;

  mpu.update();

  float accelX = mpu.getAccX();
  float accelY = mpu.getAccY();
  float accelZ = mpu.getAccZ();
  float gyroX = mpu.getGyroX();
  float gyroY = mpu.getGyroY();

  // Pitch and Roll from accelerometer
  float pitchAcc = atan2(accelY, accelZ) * RAD_TO_DEG;
  float rollAcc = atan2(-accelX, sqrt(accelY * accelY + accelZ * accelZ)) * RAD_TO_DEG;

  float pitch = kalmanUpdate(kalX, pitchAcc, gyroX, dt);
  float roll  = kalmanUpdate(kalY, rollAcc, gyroY, dt);

  // PID control
  float controlX = computePID(pidX, 0, pitch, dt);
  float controlY = computePID(pidY, 0, roll, dt);

  // Servo output
  int servoX_val = constrain(90 + controlX, 45, 135);
  int servoY_val = constrain(90 + controlY, 45, 135);

  servoX.write(servoX_val);
  servoY.write(servoY_val);

  // Altitude
  float altitude = bmp.readAltitude();

  // Debug print
  Serial.print("Pitch: "); Serial.print(pitch);
  Serial.print("  Roll: "); Serial.print(roll);
  Serial.print("  Altitude: "); Serial.println(altitude);

  delay(10);
}








