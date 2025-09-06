 #include <Wire.h>
#include <MPU6050.h>
#include <HardwareSerial.h>

MPU6050 mpu;
HardwareSerial &espSerial = Serial1; // RX2 (7), TX2 (8)

String wifiSSID = "abhishek-IdeaPad-Flex-5-14ITL05";
String wifiPASS = "lH5eI1Ta";

void sendAT(String cmd, int wait = 2000) {
  espSerial.println(cmd);
  unsigned long t = millis();
  while (millis() - t < wait) {
    if (espSerial.available()) Serial.write(espSerial.read());
  }
}

void setupESP() {
  sendAT("AT+RST", 3000);
  sendAT("AT+CWMODE=1");
  sendAT("AT+CWJAP=\"" + wifiSSID + "\",\"" + wifiPASS + "\"", 8000);
  sendAT("AT+CIFSR", 2000);
  sendAT("AT+CIPMUX=1");
  sendAT("AT+CIPSERVER=1,80");
}

void setup() {
  Serial.begin(115200);
  espSerial.begin(115200);  // Make sure ESP AT firmware is at 115200
  Wire.begin();
  mpu.initialize();

  Serial.println("Setting up ESP...");
  setupESP();
}

void sendSensorData(String connId, float ax, float ay, float az) {
  String data = "HTTP/1.1 200 OK\r\nContent-Type: text/html\r\n\r\n";
  data += "<!DOCTYPE html><html><head><meta http-equiv='refresh' content='1'><title>MPU6050</title></head><body>";
  data += "<h2>MPU6050 Sensor</h2>";
  data += "Accel X: " + String(ax) + "<br>";
  data += "Accel Y: " + String(ay) + "<br>";
  data += "Accel Z: " + String(az) + "<br>";
  data += "</body></html>";

  String cmd = "AT+CIPSEND=" + connId + "," + String(data.length());
  sendAT(cmd);
  delay(100);
  espSerial.print(data);
  delay(100);
  sendAT("AT+CIPCLOSE=" + connId);
}

void loop() {
  if (espSerial.available()) {
    String resp = espSerial.readStringUntil('\n');
    Serial.println(resp);

    if (resp.indexOf("+IPD") != -1) {
      int connId = resp.substring(5, 6).toInt();
      int16_t ax, ay, az;
      mpu.getAcceleration(&ax, &ay, &az);
      sendSensorData(String(connId), ax / 16384.0, ay / 16384.0, az / 16384.0);
    }
  }
}

 


