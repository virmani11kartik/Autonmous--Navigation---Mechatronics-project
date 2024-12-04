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

// Constants
const int wall_distance_setpoint = 6;         // Desired wall distance (in inches)
const int front_collision_threshold = 4;      // Minimum distance to obstacle in front
const float Kp_steering = 2.0;                // Proportional gain for wall alignment
const float Kp_alignment = 1.5;               // Proportional gain for misalignment correction

// Sensor instances
Adafruit_VL53L0X loxFront = Adafruit_VL53L0X();
Adafruit_VL53L0X loxLeft = Adafruit_VL53L0X();
Adafruit_VL53L0X loxRight = Adafruit_VL53L0X();

// Function prototypes
void initToFSensors();
void readToFSensors(int &d_front, int &d_left, int &d_right);
void wallFollowLogic();
void sendSteeringCommand(int angle, const char* direction);
void handleRoot();          // Function to handle the root web page
void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length);
void broadcastSensorData();

/**
 * Main setup function:
 * 1. Initializes WiFi connection to auto.ino's network
 * 2. Sets up web server and WebSocket for sensor data display
 * 3. Initializes ToF sensors
 */
void setup() {
  Serial.begin(115200);

  // Connect to auto.ino's Wi-Fi network with static IP
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

  // Initialize sensors
  initToFSensors(); // Initialize sensors from wall_follow.h
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

// Function to initialize the sensors
void initToFSensors() {
  // Initialize front sensor
  if (!loxFront.begin()) {
    Serial.println("Failed to initialize front sensor!");
    while (1);
  }
  // Initialize left sensor
  if (!loxLeft.begin()) {
    Serial.println("Failed to initialize left sensor!");
    while (1);
  }
  // Initialize right sensor
  if (!loxRight.begin()) {
    Serial.println("Failed to initialize right sensor!");
    while (1);
  }
  Serial.println("All sensors initialized.");
}

// Function to read distances from sensors
void readToFSensors(int &d_front, int &d_left, int &d_right) {
  VL53L0X_RangingMeasurementData_t measure;

  // Front sensor
  loxFront.rangingTest(&measure, false);
  if (measure.RangeStatus != 4) {
    d_front = measure.RangeMilliMeter;
  } else {
    d_front = -1; // Out of range
  }

  // Left sensor
  loxLeft.rangingTest(&measure, false);
  if (measure.RangeStatus != 4) {
    d_left = measure.RangeMilliMeter;
  } else {
    d_left = -1; // Out of range
  }

  // Right sensor
  loxRight.rangingTest(&measure, false);
  if (measure.RangeStatus != 4) {
    d_right = measure.RangeMilliMeter;
  } else {
    d_right = -1; // Out of range
  }
}

// Wall-following logic function
void wallFollowLogic() {
  int d_front, d_left, d_right;
  readToFSensors(d_front, d_left, d_right);

  // Debugging: print sensor values
  Serial.print("Front: "); Serial.print(d_front);
  Serial.print(" Left: "); Serial.print(d_left);
  Serial.print(" Right: "); Serial.println(d_right);

  int angle = 0;
  const char* direction = "FORWARD";

  // Collision avoidance
  if (d_front > 0 && d_front < front_collision_threshold * 25.4) { // Convert inches to mm
    if (d_left > d_right) {
      angle = 45;
      direction = "LEFT";
    } else {
      angle = 45;
      direction = "RIGHT";
    }
    delay(500);
  }
  // Wall alignment and following logic
  else {
    // Wall alignment logic
    int alignment_error = d_right - d_left;
    if (abs(alignment_error) > 50) { // If misaligned by more than 50 mm
      int correction_angle = Kp_alignment * alignment_error;
      if (alignment_error > 0) {
        angle = correction_angle;
        direction = "RIGHT";
      } else {
        angle = abs(correction_angle);
        direction = "LEFT";
      }
      delay(100);
    } else {
      // Wall-following logic
      if (abs(d_left - wall_distance_setpoint * 25.4) < abs(d_right - wall_distance_setpoint * 25.4)) {
        // Follow left wall
        int error = wall_distance_setpoint * 25.4 - d_left;
        int steering_angle = Kp_steering * error;

        if (error > 0) {
          angle = steering_angle;
          direction = "LEFT";
        } else if (error < 0) {
          angle = abs(steering_angle);
          direction = "RIGHT";
        } else {
          angle = 0;
          direction = "FORWARD";
        }
      } else {
        // Follow right wall
        int error = wall_distance_setpoint * 25.4 - d_right;
        int steering_angle = Kp_steering * error;

        if (error > 0) {
          angle = steering_angle;
          direction = "RIGHT";
        } else if (error < 0) {
          angle = abs(steering_angle);
          direction = "LEFT";
        } else {
          angle = 0;
          direction = "FORWARD";
        }
      }
    }
  }

  // Send steering command to auto.ino
  sendSteeringCommand(angle, direction);
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

// Web server handler for root page
void handleRoot() {
  server.send_P(200, "text/html", WEBPAGE);
}

// WebSocket event handler
void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
  // Handle WebSocket events 
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
