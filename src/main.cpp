 // ================== Teensy 4.1 Firmware ==================
// Sensors: MPU9250 (I2C, accel/gyro only), BMP085, Neo-6M GPS (TinyGPS++)
// WiFi/UI: ESP-01 on Serial1 (ESP runs its own firmware below)
// ---------------------------------------------------------

#include <Wire.h>
#include <Servo.h>
#include <SD.h>
#include <SPI.h>
#include <Adafruit_BMP085.h>
#include <TinyGPS++.h>

// ---------------- Pins / Serials ----------------
HardwareSerial &espSerial = Serial1;   // ESP-01 UART (TX1/RX1 on Teensy 4.1)
HardwareSerial &gpsSerial = Serial2;   // Neo-6M GPS

const int rollServoPin  = 4;
const int pitchServoPin = 5;
const int chipSelect    = BUILTIN_SDCARD;

// ---------------- Sensors ----------------
Adafruit_BMP085 bmp;   // BMP180/BMP085
const uint8_t MPU_ADDR = 0x68; // MPU9250 accel/gyro are like MPU6500/6050 regs

// --------------- Servos & Control ---------------
Servo servoRoll, servoPitch;

// Control gains (roll only shown; extend to pitch if you actuate both)
float Kp = 1.0, Ki = 0.02, Kd = 1.2;
float targetRoll = 0.0f;
float roll = 0, pitch = 0, yaw = 0;
float rollIntegral = 0, prevRollErr = 0;

// --------------- GPS ---------------
TinyGPSPlus gps;

// --------------- State / timing / logging ---------------
enum Mode : uint8_t { MODE_SAFE=0, MODE_STABILIZE=1, MODE_ASCENT=2, MODE_DESCENT=3, MODE_LAND=4, MODE_SIM=5 };
Mode mode = MODE_SAFE;

File logFile;
float groundAltitude = 0;
unsigned long prevMillis = 0;
unsigned long lastTX = 0;

// --------------- Simple Kalman (1D) for angle ---------------
float kalmanAngleX=0, kalmanBiasX=0;
float PX[2][2] = {{0,0},{0,0}};

float kalmanUpdate(float accAngle, float gyroRate, float dt,
                   float &angle, float &bias, float P[2][2]) {
  const float Q_angle=0.001f, Q_bias=0.003f, R_measure=0.03f;

  float rate = gyroRate - bias;
  angle += dt * rate;

  P[0][0] += dt * (dt*P[1][1] - P[1][0] - P[0][1] + Q_angle);
  P[0][1] -= dt * P[1][1];
  P[1][0] -= dt * P[1][1];
  P[1][1] += Q_bias * dt;

  float S = P[0][0] + R_measure;
  float K0 = P[0][0] / S;
  float K1 = P[1][0] / S;

  float y = accAngle - angle;
  angle += K0 * y;
  bias  += K1 * y;

  float P00 = P[0][0], P01 = P[0][1];
  P[0][0] -= K0 * P00;
  P[0][1] -= K0 * P01;
  P[1][0] -= K1 * P00;
  P[1][1] -= K1 * P01;

  return angle;
}

// --------------- MPU I2C helpers ----------------
int16_t read16(uint8_t reg) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom((int)MPU_ADDR, 2);
  while (Wire.available() < 2) {}
  uint8_t hi = Wire.read(), lo = Wire.read();
  return (int16_t)((hi<<8)|lo);
}
void write8(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

const char* modeName(Mode m) {
  switch(m){
    case MODE_SAFE: return "SAFE";
    case MODE_STABILIZE: return "STABILIZE";
    case MODE_ASCENT: return "ASCENT";
    case MODE_DESCENT: return "DESCENT";
    case MODE_LAND: return "LAND";
    case MODE_SIM: return "SIM";
    default: return "UNKNOWN";
  }
}

void setMode(Mode m) {
  mode = m;
  // one-time mode entry actions
  if (mode == MODE_SAFE) {
    servoRoll.write(90);
    servoPitch.write(90);
  }
}

// --------------- Setup ----------------
void setup() {
  Serial.begin(115200);
  Wire.begin();
  espSerial.begin(115200);
  gpsSerial.begin(9600);

  // --- MPU9250 init (accel/gyro only) ---
  delay(100);
  // wake
  write8(0x6B, 0x00);   // PWR_MGMT_1: wake
  delay(50);
  // gyro ±250 dps
  write8(0x1B, 0x00);
  // accel ±2g
  write8(0x1C, 0x00);

  // --- BMP085/180 ---
  if (!bmp.begin()) {
    Serial.println("BMP180 not detected!");
    while(1) {}
  }
  groundAltitude = bmp.readAltitude();

  // --- Servos ---
  servoRoll.attach(rollServoPin);
  servoPitch.attach(pitchServoPin);
  servoRoll.write(90);
  servoPitch.write(90);

  // --- SD ---
  if (!SD.begin(chipSelect)) {
    Serial.println("SD init failed!");
  } else {
    logFile = SD.open("rocket.csv", FILE_WRITE);
    if (logFile) {
      logFile.println("time,mode,roll,pitch,yaw,alt,lat,lng,rollErr,pid,servo");
      logFile.close();
    }
  }

  setMode(MODE_SAFE);
  Serial.println("Teensy ready.");
}

// --------------- Command parsing from ESP ----------------
String rxLine;
void handleESPCommands() {
  while (espSerial.available()) {
    char c = espSerial.read();
    if (c == '\n') {
      rxLine.trim();
      if (rxLine.startsWith("MODE:")) {
        int m = rxLine.substring(5).toInt();
        if (m >= 0 && m <= 5) {
          setMode((Mode)m);
          Serial.print("Mode set to "); Serial.println(modeName(mode));
        }
      }
      rxLine = "";
    } else {
      rxLine += c;
    }
  }
}

// --------------- Main loop ----------------
void loop() {
  unsigned long now = millis();
  float dt = (now - prevMillis) / 1000.0f;
  if (dt <= 0) dt = 0.001f;
  prevMillis = now;

  // ---- Read sensors ----
  // Accel/gyro raw regs (MPU9250)
  int16_t ax = read16(0x3B);
  int16_t ay = read16(0x3D);
  int16_t az = read16(0x3F);
  int16_t gx = read16(0x43);
  int16_t gy = read16(0x45);
  int16_t gz = read16(0x47);

  // scale
  float axf = ax / 16384.0f;
  float ayf = ay / 16384.0f;
  float azf = az / 16384.0f;
  float gxf = gx / 131.0f;   // dps
  float gyf = gy / 131.0f;
  float gzf = gz / 131.0f;

  // angles (deg)
  float accAngleX = atan2f(ayf, azf) * 180.0f / PI;
  float accAngleY = atan2f(-axf, azf) * 180.0f / PI;

  // simple 1D Kalman on roll; mirror for pitch using same function if you like
  roll  = kalmanUpdate(accAngleX, gxf, dt, kalmanAngleX, kalmanBiasX, PX);
  // for brevity, a light complementary approach on pitch:
  static float pitch_est=0;
  pitch_est = 0.98f*(pitch_est + gyf*dt) + 0.02f*accAngleY;
  pitch = pitch_est;

  // altitude (relative to ground)
  float altitude = bmp.readAltitude() - groundAltitude;

  // GPS feed
  while (gpsSerial.available()) gps.encode(gpsSerial.read());
  double lat = gps.location.isValid() ? gps.location.lat() : 0.0;
  double lng = gps.location.isValid() ? gps.location.lng() : 0.0;

  // ---- Mode behavior & PID (roll) ----
  float pidOutput = 0;
  int servoAngle = 90;

  if (mode == MODE_SIM) {
    // synthetic demo motion
    roll  = 10.0f * sinf(now/700.0f);
    pitch = 5.0f  * cosf(now/900.0f);
  }

  if (mode == MODE_STABILIZE || mode == MODE_ASCENT || mode == MODE_DESCENT) {
    float err = targetRoll - roll;
    rollIntegral += err * dt;
    float d = (err - prevRollErr) / dt;
    pidOutput = Kp*err + Ki*rollIntegral + Kd*d;
    prevRollErr = err;

    servoAngle = constrain(90 + (int)pidOutput, 60, 120); // tighten if needed
    servoRoll.write(servoAngle);
    // keep pitch neutral here (or add its own PID)
    servoPitch.write(90);
  } else {
    // SAFE / LAND: neutral
    servoRoll.write(90);
    servoPitch.write(90);
    rollIntegral = 0; prevRollErr = 0;
  }

  // ---- Send telemetry to ESP (as one-line JSON) ----
  if (now - lastTX >= 100) { // 10 Hz
    lastTX = now;
    espSerial.print("{\"t\":");      espSerial.print(now);
    espSerial.print(",\"mode\":\""); espSerial.print(modeName(mode)); espSerial.print('"');
    espSerial.print(",\"roll\":");   espSerial.print(roll, 2);
    espSerial.print(",\"pitch\":");  espSerial.print(pitch, 2);
    espSerial.print(",\"yaw\":");    espSerial.print(yaw, 2);
    espSerial.print(",\"alt\":");    espSerial.print(altitude, 2);
    espSerial.print(",\"lat\":");    espSerial.print(lat, 6);
    espSerial.print(",\"lng\":");    espSerial.print(lng, 6);
    espSerial.print(",\"servo\":");  espSerial.print(servoAngle);
    espSerial.println("}");
  }

  // ---- Log (optional) ----
  if (SD.mediaPresent()) {
    logFile = SD.open("rocket.csv", FILE_WRITE);
    if (logFile) {
      logFile.print(now); logFile.print(',');
      logFile.print(modeName(mode)); logFile.print(',');
      logFile.print(roll,2); logFile.print(',');
      logFile.print(pitch,2); logFile.print(',');
      logFile.print(yaw,2); logFile.print(',');
      logFile.print(altitude,2); logFile.print(',');
      logFile.print(lat,6); logFile.print(',');
      logFile.print(lng,6); logFile.print(',');
      float err = (targetRoll - roll);
      float d   = (err - prevRollErr) / (dt>0?dt:1);
      float pid = Kp*err + Ki*rollIntegral + Kd*d;
      logFile.print(err,2); logFile.print(',');
      logFile.print(pid,2); logFile.print(',');
      logFile.println(servoAngle);
      logFile.close();
    }
  }

  // ---- Handle ESP control commands ----
  handleESPCommands();