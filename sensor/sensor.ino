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
IPAddress local_IP(192, 168, 1, 105);      // Set an IP address for sensor.ino
IPAddress gateway(192, 168, 1, 104);       // auto.ino's IP address as gateway
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

/**
 * Main setup function:
 * 1. Initializes WiFi connection to auto.ino's network
 * 2. Sets up web server and WebSocket for sensor data display
 * 3. Initializes ToF sensors
 */
void setup() {
  Serial.begin(115200);
  delay(100); // Give serial time to start up

  // Initialize sensors first
  Serial.println("Initializing ToF sensors...");
  initToFSensors();
  Serial.println("ToF sensors initialized successfully!");
  delay(500); // Give sensors time to stabilize

  // Now continue with WiFi and other setup
  Serial.println("Connecting to WiFi...");
  WiFi.mode(WIFI_STA);
  WiFi.config(local_IP, gateway, subnet);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.print("\nConnected to ");
  Serial.println(ssid);
  Serial.print("Sensor IP address: ");
  Serial.println(WiFi.localIP());

  // Start the web server
  server.on("/", handleRoot);
  server.begin();
  Serial.println("Web server started.");

  // Start WebSocket server
  webSocket.begin();
  webSocket.onEvent(webSocketEvent);
  Serial.println("WebSocket server started.");
}

/**
 * Main loop handles:
 * 1. Web server and WebSocket communication
 * 2. Wall following logic
 * 3. Sensor data broadcasting
 */
void loop() {
  // Handle web server
  server.handleClient();

  // Handle WebSocket
  webSocket.loop();

  // Call the wall-following logic
  wallFollowLogic();

  // Broadcast sensor data to webpage
  broadcastSensorData();

  delay(100); // Adjust as needed
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

  // Send UDP packet
  udp.beginPacket(udpAddress, udpPort);
  udp.write((uint8_t*)jsonString, strlen(jsonString));
  udp.endPacket();
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