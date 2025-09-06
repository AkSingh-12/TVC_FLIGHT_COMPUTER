 #include <Wire.h>
#include <MPU6050.h>
#include <Servo.h>
#include <SD.h>
#include <SPI.h>
#include <Adafruit_BMP085.h>
#include <TinyGPS++.h>

MPU6050 mpu;
Servo servoRoll;
Servo servoPitch;
Adafruit_BMP085 bmp;
TinyGPSPlus gps;

HardwareSerial &gpsSerial = Serial1;  // GPS on Serial1 (pins 0 and 1 on Teensy 4.1)

const int rollServoPin = 4;
const int pitchServoPin = 3;
const int chipSelect = BUILTIN_SDCARD;

float roll = 0.0, pitch = 0.0;
float previousRoll = 0.0, previousPitch = 0.0;
float accAngleX, accAngleY, gyroRateX, gyroRateY;
float kalmanRoll = 0.0, kalmanPitch = 0.0;

float targetRoll = 0.0;
float rollError, rollIntegral = 0.0, rollDerivative, previousRollError = 0.0;
float Kp = 1.5, Ki = 0.02, Kd = 1.2;

float altitude = 0.0;
float groundAltitude = 0.0;

unsigned long previousTime = 0;
File logFile;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  gpsSerial.begin(9600);

  mpu.initialize();
  if (!bmp.begin()) {
    Serial.println("BMP180 not detected!");
    while (1);
  }

  if (!mpu.testConnection()) {
    Serial.println("MPU6050 not connected!");
    while (1);
  }

  servoRoll.attach(rollServoPin);
  servoPitch.attach(pitchServoPin);
  servoRoll.write(120);
  servoPitch.write(120);

  if (!SD.begin(chipSelect)) {
    Serial.println("SD init failed!");
    while (1);
  }

  logFile = SD.open("rocket.csv", FILE_WRITE);
  if (logFile) {
    logFile.println("Time,Roll,Pitch,Altitude,GPS_Lat,GPS_Lng,Roll_Error,PID_Output,Servo_Angle");
    logFile.close();
  }

  delay(1000);
  groundAltitude = bmp.readAltitude();
}

void loop() {
  unsigned long currentTime = millis();
  float dt = (currentTime - previousTime) / 1000.0;
  previousTime = currentTime;

  // Read MPU6050
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  accAngleX = atan2(ay, az) * 180 / PI;
  accAngleY = atan2(-ax, az) * 180 / PI;
  gyroRateX = gx / 131.0;
  gyroRateY = gy / 131.0;

  roll = kalmanFilter(accAngleX, gyroRateX, dt);
  pitch = kalmanFilter(accAngleY, gyroRateY, dt);

  // PID control for Roll
  rollError = targetRoll - roll;
  rollIntegral += rollError * dt;
  rollDerivative = (rollError - previousRollError) / dt;
  float pidOutput = Kp * rollError + Ki * rollIntegral + Kd * rollDerivative;
  previousRollError = rollError;

  // Limit and apply servo angle
  int servoAngle = constrain(90 + pidOutput, 90, 180);
  servoRoll.write(servoAngle);
  

  // Get altitude
  altitude = bmp.readAltitude() - groundAltitude;

  // Read GPS
  while (gpsSerial.available()) {
    gps.encode(gpsSerial.read());
  }

  float latitude = gps.location.isValid() ? gps.location.lat() : 0.0;
  float longitude = gps.location.isValid() ? gps.location.lng() : 0.0;

  // Log data to SD card
  logFile = SD.open("rocket.csv", FILE_WRITE);
  if (logFile) {
    logFile.print(currentTime);
    logFile.print(',');
    logFile.print(roll, 2);
    logFile.print(',');
    logFile.print(pitch, 2);
    logFile.print(',');
    logFile.print(altitude, 2);
    logFile.print(',');
    logFile.print(latitude, 6);
    logFile.print(',');
    logFile.print(longitude, 6);
    logFile.print(',');
    logFile.print(rollError, 2);
    logFile.print(',');
    logFile.print(pidOutput, 2);
    logFile.print(',');
    logFile.println(servoAngle);
    logFile.close();
  }

  // ✅ Real-time output for Python 3D Visualization
  Serial.print(currentTime); Serial.print(',');
Serial.print(roll); Serial.print(',');
Serial.print(pitch); Serial.print(',');
Serial.print(0.0); Serial.print(',');  // Yaw placeholder
Serial.print(latitude, 6); Serial.print(',');
Serial.print(longitude, 6); Serial.print(',');
Serial.print(altitude); Serial.print(',');
Serial.print(rollError); Serial.print(',');
Serial.print(pidOutput); Serial.print(',');
Serial.println(servoAngle);

}

float kalmanFilter(float accAngle, float gyroRate, float dt) {
  static float angle = 0.0;
  static float bias = 0.0;
  static float P[2][2] = {{0, 0}, {0, 0}};
  float Q_angle = 0.001;
  float Q_bias = 0.003;
  float R_measure = 0.03;

  float rate = gyroRate - bias;
  angle += dt * rate;

  P[0][0] += dt * (dt*P[1][1] - P[1][0] - P[0][1] + Q_angle);
  P[0][1] -= dt * P[1][1];
  P[1][0] -= dt * P[1][1];
  P[1][1] += Q_bias * dt;

  float S = P[0][0] + R_measure;
  float K[2];
  K[0] = P[0][0] / S;
  K[1] = P[1][0] / S;

  float y = accAngle - angle;
  angle += K[0] * y;
  bias += K[1] * y;

  float P00_temp = P[0][0], P01_temp = P[0][1];
  P[0][0] -= K[0] * P00_temp;
  P[0][1] -= K[0] * P01_temp;
  P[1][0] -= K[1] * P00_temp;
  P[1][1] -= K[1] * P01_temp;

  return angle;
}

