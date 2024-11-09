#include <WiFi.h>
#include <WebServer.h>
#include <webpage.h>

#define LEDC_RESOLUTION_BITS 10
#define LEDC_RESOLUTION ((1 << LEDC_RESOLUTION_BITS) - 1)
#define LEDC_FREQUENCY 50  // Frequency in Hz


// Wi-Fi network name and password
const char* ssid = "vaibhav-esp32";  // Wi-Fi network name
const char* password = "12345678";   // Wi-Fi password (empty for no password)

// IP configuration for AP mode
IPAddress local_IP(192, 168, 1, 104);  // ESP32's IP address
IPAddress gateway(192, 168, 1, 104);   // Gateway IP (same as ESP32 in AP mode)
IPAddress subnet(255, 255, 255, 0);    // Subnet mask

WebServer server(80);

const int pwmPin1 = 4;  // PWM output pin (connected to EN pin on H-Bridge)
const int pwmPin2 = 5;
const int dirPin1 = 18;  // Direction control pin 1 (connected to IN1)
const int dirPin2 = 19;  // Direction control pin 2 (connected to IN2)
const int encoderPin1 = 1;
const int encoderPin2 = 10;

int pinState1, pinState2;
int pulseCount1, pulseCount2;
int rotation_count1, rotation_count2;
int prev_time1, prev_time2;
int rpm_measure1, rpm_measure2;
int time_elapsed1, time_elapsed2;

// Initialize motor parameters
int motorSpeed = 0;  // Motor speed (0-100%)
String forwardBackward = "Neutral";
int directionPercent = 0;  // Direction adjustment (0-50%)
String leftRight = "Neutral";

void setup() {
  Serial.begin(115200);
  pinMode(dirPin1, OUTPUT);
  pinMode(dirPin2, OUTPUT);
  pinMode(pwmPin1, OUTPUT);
  pinMode(pwmPin2, OUTPUT);
  pinMode(encoderPin1, INPUT_PULLUP);
  pinMode(encoderPin2, INPUT_PULLUP);
  digitalWrite(dirPin1, LOW);
  digitalWrite(dirPin2, LOW);
  ledcAttach(pwmPin1, LEDC_FREQUENCY, LEDC_RESOLUTION_BITS);
  ledcAttach(pwmPin2, LEDC_FREQUENCY, LEDC_RESOLUTION_BITS);
  ledcWrite(pwmPin1, 0);
  ledcWrite(pwmPin2, 0);

  // Set up Access Point
  Serial.println("Setting up Access Point...");
  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(local_IP, gateway, subnet);
  WiFi.softAP(ssid, password);
  Serial.print("AP IP address: ");
  Serial.println(WiFi.softAPIP());
  server.on("/", handleRoot);
  server.on("/setMotor", handleSetMotor);
  server.begin();
  Serial.print("HTTP server started at internal IP: ");
  Serial.print(local_IP);
  Serial.print("\n");
}

void loop() {
  server.handleClient();
  // rpm_counter();
}

void rpm_counter() {
  int currPinState1 = digitalRead(encoderPin1);
  if (pinState1 != currPinState1) {
    pinState1 = currPinState1;
    pulseCount1 += 1;
    if (pulseCount1 == 40) {
      rotation_count1 += 1;
      pulseCount1 = 0;
      int curr_time1 = millis();
      time_elapsed1 = curr_time1 - prev_time1;
      prev_time1 = curr_time1;
      rpm_measure1 = (int)(60000.0 / time_elapsed1);
    }
  }
  int currPinState2 = digitalRead(encoderPin2);
  if (pinState2 != currPinState2) {
    pinState2 = currPinState2;
    pulseCount2 += 1;
    if (pulseCount2 == 40) {
      rotation_count2 += 1;
      pulseCount2 = 0;
      int curr_time2 = millis();
      time_elapsed2 = curr_time2 - prev_time2;
      prev_time2 = curr_time2;
      rpm_measure2 = (int)(60000.0 / time_elapsed2);
    }
  }

  // Display RPM values for both encoders
  Serial.print("Encoder 1 - Pulse: ");
  Serial.print(pulseCount1);
  Serial.print(" RPM: ");
  Serial.print(rpm_measure1);
  Serial.print(" | Encoder 2 - Pulse: ");
  Serial.print(pulseCount2);
  Serial.print(" RPM: ");
  Serial.print(rpm_measure2);
  Serial.print("\n");
  delay(5);
}

// Function to handle root URL "/"
void handleRoot() {
  server.send_P(200, "text/html", INDEX_HTML);
}

// TODO: PID Control

// TODO: Position Feedback

// TODO (optional) helper function

#define RPM_TO_RAD_PER_SEC 0.10471975512                   // Conversion factor from RPM to rad/s
#define MAX_RPM 130                                        // Maximum RPM of the motor
#define MAX_WHEEL_VELOCTY (MAX_RPM * RPM_TO_RAD_PER_SEC)   // Maximum wheel velocity in rad/s
#define WHEEL_RADIUS 50                                    // Wheel radius in millimeters

// V = (wl*r + wr*r) / 2
#define MAX_LINEAR_VELOCTY (MAX_WHEEL_VELOCTY * WHEEL_RADIUS) // Maximum linear velocity in m/s

// w_combined = (wl - wr)*r/L
#define WHEEL_BASE 300  // Distance between wheels in millimeters
#define MAX_ANGULAR_VELOCITY ((MAX_WHEEL_VELOCTY * WHEEL_RADIUS)/WHEEL_BASE)

// const float wheelRadius = 0.05;          // Wheel radius in meters
// const float wheelBase = 0.3;             // Distance between wheels in meters
// const float maxAngularVelocity = 22.57;  // Maximum angular velocity (rad/s), adjust as needed
// const float maxLinearVelocity = 1.0;

float degreesToRadians(float degrees) {
  return degrees * (M_PI / 180.0);
}

float mapf(float value, float inMin, float inMax, float outMin, float outMax) {
  return (value - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
}

void sendMotorSignals(float omega, int direction_pin, int pwm_pin, int& pwm_ref) {
  float pwm_fval = 0.0;
  if (omega > 0.0) {
    digitalWrite(direction_pin, HIGH);
    pwm_fval = mapf(omega, 0, MAX_WHEEL_VELOCTY, 0, (float)LEDC_RESOLUTION);
    pwm_ref = min((int)pwm_fval, LEDC_RESOLUTION);
  } else {
    digitalWrite(direction_pin, LOW);
    pwm_fval = mapf(-omega, 0, MAX_WHEEL_VELOCTY, 0, (float)LEDC_RESOLUTION);
    pwm_ref = min((int)pwm_fval, LEDC_RESOLUTION);
  }
}

void calculateDifferentialDrive(float linearVelocity, float angularVelocity, int& leftPWM, int& rightPWM) {
  float omegaL = (linearVelocity + angularVelocity * WHEEL_BASE / 2) / WHEEL_RADIUS;
  float omegaR = (linearVelocity - angularVelocity * WHEEL_BASE / 2) / WHEEL_RADIUS;
  Serial.printf("OmegaL: %f, OmegaR: %f\n", omegaL, omegaR);
  
  sendMotorSignals(omegaL, dirPin1, pwmPin1, leftPWM);
  sendMotorSignals(omegaR, dirPin2, pwmPin2, rightPWM);
}

void handleSetMotor() {
  if (server.hasArg("speed") && server.hasArg("forwardBackward") && server.hasArg("turnRate")) {

    float motorSpeed = server.arg("speed").toFloat(); // percent from 0 to 100
    String forwardBackward = server.arg("forwardBackward"); // "Forward" or "Backward"
    float turnRate = server.arg("turnRate").toFloat(); // percent from -50 to 50

    // Convert speed input to linear velocity
    float linearVelocity = (forwardBackward == "Forward" ? 1 : -1) * (motorSpeed / 100.0) * MAX_LINEAR_VELOCTY;
    float angularVelocity = (turnRate/50.0) * MAX_ANGULAR_VELOCITY;

    int leftPWM, rightPWM;

    calculateDifferentialDrive(linearVelocity, angularVelocity, leftPWM, rightPWM);
    // Set the PWM for each wheel
    ledcWrite(pwmPin1, leftPWM);
    ledcWrite(pwmPin2, rightPWM);

    // Send response
    server.sendHeader("Access-Control-Allow-Origin", "*");
    server.send(200, "text/plain", "Motor updated");

    // Debugging output
    Serial.printf("Linear velocity: %.2f, Angular velocity: %.2f\n", linearVelocity, angularVelocity);
    Serial.printf("Left PWM: %d, Right PWM: %d\n", leftPWM, rightPWM);
  } else {
    server.send(400, "text/plain", "Bad Request");
  }
}
