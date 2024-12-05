#include <WiFi.h>
#include <WiFiUdp.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <Adafruit_VL53L0X.h>
#include "wall_follow.h"
#include "web.h"

// Wi-Fi network details (connect to auto.ino's AP)
const char* ssid = "GM Lab Public WIFI";   // auto.ino's Wi-Fi SSID
const char* password = "";                 // auto.ino's Wi-Fi password

// Static IP configuration for sensor.ino
IPAddress local_IP(192, 168, 1, 105);      // Your sensor's IP
IPAddress gateway(192, 168, 1, 1);         // Router/Gateway IP (usually 192.168.1.1)
IPAddress subnet(255, 255, 255, 0);        // Subnet mask

// IP and port of the auto.ino board
const char* udpAddress = "192.168.1.104";  // IP address of auto.ino
const unsigned int udpPort = 8888;         // UDP port to send data to

WiFiUDP udp;
WebServer server(80);
WebSocketsServer webSocket = WebSocketsServer(81);

// Function prototypes
void sendSteeringCommand(int angle, const char* direction);
void broadcastSensorData();
void handleRoot();
void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length);

/**
 * Main setup function:
 * 1. Initializes WiFi connection to auto.ino's network
 * 2. Sets up web server and WebSocket for sensor data display
 * 3. Initializes ToF sensors
 */
void setup() {
  Serial.begin(115200);
  
  // Initialize sensors first
  Serial.println("Initializing ToF sensors...");
  initToFSensors();
  Serial.println("ToF sensors initialized successfully!");
  
  // Wi-Fi setup as STA mode
  Serial.println("Connecting to WiFi...");
  WiFi.mode(WIFI_STA);
  if (!WiFi.config(local_IP, gateway, subnet)) {
    Serial.println("Failed to configure static IP");
  }
  WiFi.begin(ssid, password);
  
  // Wait for connection with timeout and status updates
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(500);
    Serial.print(".");
    attempts++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nConnected to WiFi");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());

    // Start web server
    server.on("/", HTTP_GET, handleRoot);
    server.begin();
    Serial.println("HTTP server started");

    // Start WebSocket server
    webSocket.begin();
    webSocket.onEvent(webSocketEvent);
    Serial.println("WebSocket server started");

    // Start UDP
    if(udp.begin(udpPort)) {
      Serial.printf("UDP client started at local port %d\n", udpPort);
    } else {
      Serial.println("UDP client failed to start");
    }
  }
}

/**
 * Main loop handles:
 * 1. Web server and WebSocket communication
 * 2. Wall following logic
 * 3. Sensor data broadcasting
 */
void loop() {
  if (WiFi.status() == WL_CONNECTED) {
    webSocket.loop();
    server.handleClient();
    
    static unsigned long lastUpdate = 0;
    if (millis() - lastUpdate >= 100) {  // Update every 100ms
      broadcastSensorData();
      lastUpdate = millis();
    }
    
    // Call wall following logic
    wallFollowLogic();
  } else {
    // Try to reconnect to WiFi if connection is lost
    static unsigned long lastReconnectAttempt = 0;
    unsigned long currentTime = millis();
    
    if (currentTime - lastReconnectAttempt >= 5000) {  // Try every 5 seconds
      Serial.println("Reconnecting to WiFi...");
      WiFi.reconnect();
      lastReconnectAttempt = currentTime;
    }
  }
}

/**
 * Sends steering commands to auto.ino via UDP
 * @param angle: Desired turning angle
 * @param direction: "LEFT", "RIGHT", or "FORWARD"
 */
void sendSteeringCommand(int angle, const char* direction) {
  StaticJsonDocument<200> doc;
  doc["angle"] = angle;
  doc["direction"] = direction;
  char jsonString[200];
  serializeJson(doc, jsonString);

  // Send UDP packet with proper error handling
  if(udp.beginPacket(udpAddress, udpPort)) {
    udp.write((uint8_t*)jsonString, strlen(jsonString));
    if(udp.endPacket()) {
      Serial.printf("[Successful] Sent packet: %s\n", jsonString);
    } else {
      Serial.println("Failed to send UDP packet");
    }
  } else {
    Serial.println("Could not begin UDP packet");
  }
}

/**
 * Broadcasts sensor data to connected web clients via WebSocket
 * Sends front, left, and right distances in JSON format
 */
void broadcastSensorData() {
  int d_front, d_left, d_right;
  readToFSensors(d_front, d_left, d_right);

  StaticJsonDocument<200> doc;
  doc["front"] = d_front;
  doc["left"] = d_left;
  doc["right"] = d_right;
  String jsonString;
  serializeJson(doc, jsonString);

  webSocket.broadcastTXT(jsonString);
}

// Implement handleRoot function similar to auto.ino
void handleRoot() {
  server.send_P(200, "text/html", WEBPAGE);
}

void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
  switch(type) {
    case WStype_DISCONNECTED:
      Serial.printf("[%u] Disconnected!\n", num);
      break;
    case WStype_CONNECTED:
      {
        Serial.printf("[%u] Connected from URL: %s\n", num, payload);
      }
      break;
    case WStype_TEXT:
      Serial.printf("[%u] get Text: %s\n", num, payload);
      break;
  }
}