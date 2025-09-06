 // Upload this to the ESP8266 (via Arduino IDE)
#include <ESP8266WiFi.h>

const char* ssid = "Y20A2021";
const char* password = "123456789";

void setup() {
  Serial.begin(115200);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.println("Connecting...");
  }
  Serial.println("WiFi connected!");
}

void loop() {
  // Listen for commands from Teensy
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    if (cmd == "STATUS") {
      Serial.println("WiFi OK");
    }
  }
}









