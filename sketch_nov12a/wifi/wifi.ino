 #include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>

const char* ssid = "abhishek-IdeaPad-Flex-5-14ITL05";  // Replace with your open hotspot SSID
const char* password = "lH5eI1Ta";               // Open network

ESP8266WebServer server(80);

const int ledPin = 2;    // GPIO2 = onboard LED for ESP-01
bool ledState = false;

void handleRoot() {
  int rssi = WiFi.RSSI(); // Wi-Fi signal strength in dBm

  String html = R"rawliteral(
    <html>
    <head>
      <title>ESP-01 Dashboard</title>
      <meta name="viewport" content="width=device-width, initial-scale=1.0">
      <style>
        body { font-family: sans-serif; text-align: center; margin-top: 40px; }
        h1 { color: #0077cc; }
        .box { padding: 20px; margin: 20px auto; width: 90%; max-width: 300px; border: 1px solid #ccc; border-radius: 10px; box-shadow: 2px 2px 8px rgba(0,0,0,0.1); }
        button { padding: 10px 20px; font-size: 16px; background: #28a745; color: white; border: none; border-radius: 5px; }
        button:hover { background: #218838; }
      </style>
    </head>
    <body>
      <h1>📶 ESP-01 Dashboard</h1>
      <div class="box">
        <p><strong>Wi-Fi Signal Strength:</strong></p>
        <p>)rawliteral";
  html += rssi;
  html += " dBm</p><p><strong>LED State: </strong>";
  html += (ledState ? "ON" : "OFF");
  html += R"rawliteral(</p>
        <a href="/toggle"><button>Toggle LED</button></a>
      </div>
    </body>
    </html>
  )rawliteral";

  server.send(200, "text/html", html);
}

void handleToggle() {
  ledState = !ledState;
  digitalWrite(ledPin, ledState ? LOW : HIGH); // LOW = ON for ESP-01 onboard LED
  handleRoot(); // Redirect back to main page
}

void setup() {
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, HIGH); // Turn LED OFF initially

  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  Serial.println("🔌 Connecting to Wi-Fi...");
  int retry = 0;
  while (WiFi.status() != WL_CONNECTED && retry < 30) {
    delay(500);
    Serial.print(".");
    retry++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\n✅ Connected to Wi-Fi!");
    Serial.print("📱 Open http://");
    Serial.println(WiFi.localIP());

    server.on("/", handleRoot);
    server.on("/toggle", handleToggle);
    server.begin();
    Serial.println("🌐 Web server started.");
  } else {
    Serial.println("\n❌ Failed to connect to hotspot.");
  }
}

void loop() {
  server.handleClient();
}

