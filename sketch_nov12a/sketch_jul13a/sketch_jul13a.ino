 #include <Wire.h>
#include <MPU9250.h>

MPU9250 mpu;

void setup() {
  Serial.begin(115200);
  Wire.begin();

  if (!mpu.setup(0x68)) {  // change to 0x69 if AD0 pin is HIGH
    Serial.println("MPU9250 connection failed. Please check wiring.");
    while (1);
  }

  Serial.println("MPU9250 initialized successfully.");
}

void loop() {
  if (mpu.update()) {
    Serial.print("Accel [g]: ");
    Serial.print(mpu.getAccX(), 3); Serial.print(", ");
    Serial.print(mpu.getAccY(), 3); Serial.print(", ");
    Serial.print(mpu.getAccZ(), 3); Serial.print(" | ");

    Serial.print("Gyro [deg/s]: ");
    Serial.print(mpu.getGyroX(), 3); Serial.print(", ");
    Serial.print(mpu.getGyroY(), 3); Serial.print(", ");
    Serial.print(mpu.getGyroZ(), 3); Serial.print(" | ");

    Serial.print("Mag [uT]: ");
    Serial.print(mpu.getMagX(), 3); Serial.print(", ");
    Serial.print(mpu.getMagY(), 3); Serial.print(", ");
    Serial.print(mpu.getMagZ(), 3); Serial.println();
  }

  delay(200);
}

