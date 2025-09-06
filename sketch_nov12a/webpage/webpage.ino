
// ================== ESP-01 Firmware (ESP8266) ==================
#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <LittleFS.h>
#include <ESP8266WebServer.h>

ESP8266WebServer server(80);

// ---- WiFi ----
const char* ssid = "YOUR_WIFI_SSID";
const char* pass = "YOUR_WIFI_PASSWORD";

// ---- UART to Teensy ----
// ESP-01 has UART0 on GPIO1(TX) / GPIO3(RX). Connect to Teensy Serial1.
String lastJSON = "{}";
unsigned long lastReq = 0;

void handleRoot() {
  if (!LittleFS.exists("/index.html")) {
    server.send(500, "text/plain", "index.html not found in LittleFS");
    return;
  }
  File f = LittleFS.open("/index.html", "r");
  server.streamFile(f, "text/html");
  f.close();
}

void handleTelemetry() {
  server.send(200, "application/json", lastJSON);
}

void handleMode() {
  if (!server.hasArg("m")) {
    server.send(400, "text/plain", "missing m");
    return;
  }
  int m = server.arg("m").toInt();
  if (m < 0 || m > 5) {
    server.send(400, "text/plain", "m out of range");
    return;
  }
  // Forward to Teensy
  Serial.print("MODE:");
  Serial.println(m);
  server.send(200, "text/plain", "OK");
}

void setup() {
  Serial.begin(115200);
  delay(50);

  LittleFS.begin();
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, pass);
  while (WiFi.status() != WL_CONNECTED) { delay(300); }

  server.on("/", HTTP_GET, handleRoot);
  server.on("/telemetry", HTTP_GET, handleTelemetry);
  server.on("/mode", HTTP_POST, handleMode);
  server.begin();
}

void loop() {
  server.handleClient();

  // Read lines of JSON from Teensy
  static String line;
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      line.trim();
      if (line.startsWith("{") && line.endsWith("}")) {
        lastJSON = line;
      }
      line = "";
    } else {
      line += c;
    }
  }
}
