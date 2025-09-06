 #include <Wire.h>
#include <MPU9250_asukiaaa.h>
MPU9250_asukiaaa mpu;

void setup() {
  Serial.begin(9600);
  while (!Serial);  // Wait for USB Serial to be ready (Teensy)

  Wire.begin();
  mpu.setWire(&Wire);

  mpu.beginAccel();
  mpu.beginGyro();
  mpu.beginMag();

  delay(1000);  // Allow time for sensor to stabilize
  Serial.println("MPU9250 initialized.");
}

void loop() {
  mpu.accelUpdate();
  mpu.gyroUpdate();
  mpu.magUpdate();

  float ax = mpu.accelX();
  float ay = mpu.accelY();
  float az = mpu.accelZ();
  float mx = mpu.magX();
  float my = mpu.magY();
  float mz = mpu.magZ();

  // Calculate Roll, Pitch using accelerometer
  float roll  = atan2(ay, az) * 180 / PI;
  float pitch = atan2(-ax, sqrt(ay * ay + az * az)) * 180 / PI;

  // Yaw using magnetometer (very basic tilt compensation)
  float yaw = atan2(my, mx) * 180 / PI;

  // Send data in the expected format
  Serial.print("Roll: "); Serial.print(roll, 2);
  Serial.print(", Pitch: "); Serial.print(pitch, 2);
  Serial.print(", Yaw: "); Serial.println(yaw, 2);

  delay(100);  // ~10 updates per second
}








