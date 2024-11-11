#include <WiFi.h>
#include <WebServer.h>
#include <webpage.h>
#include <pid.h>

#define LEDC_RESOLUTION_BITS 10
#define LEDC_RESOLUTION ((1 << LEDC_RESOLUTION_BITS) - 1)
#define LEDC_FREQUENCY 50  // Frequency in Hz
#define EXTRA_CARE 1 // whether to make motor stop before switching direction
#define MOTOR_STOP_DELAY 10

#define RPM_TO_RAD_PER_SEC 0.10471975512                   // Conversion factor from RPM to rad/s
#define MAX_RPM 120                                        // Maximum RPM of the motor
#define MAX_WHEEL_VELOCTY (MAX_RPM * RPM_TO_RAD_PER_SEC)   // Maximum wheel velocity in rad/s
#define WHEEL_RADIUS 36                                    // Wheel radius in millimeters

// V = (wl*r + wr*r) / 2
#define MAX_LINEAR_VELOCTY (MAX_WHEEL_VELOCTY * WHEEL_RADIUS) // Maximum linear velocity in m/s

// w_combined = (wl - wr)*r/L
#define WHEEL_BASE 185  // Distance between wheels in millimeters
#define MAX_ANGULAR_VELOCITY ((MAX_WHEEL_VELOCTY * WHEEL_RADIUS)/WHEEL_BASE)

// Wi-Fi network name and password
const char* ssid = "GM Lab Public WIFI";  // Wi-Fi network name
const char* password = "";   // Wi-Fi password (empty for no password)

// IP configuration for AP mode
IPAddress local_IP(192, 168, 1, 104);  // ESP32's IP address
IPAddress gateway(192, 168, 1, 104);   // Gateway IP (same as ESP32 in AP mode)
IPAddress subnet(255, 255, 255, 0);    // Subnet mask

WebServer server(80);
volatile unsigned long serverPrevTime = 0;
const unsigned long serverInterval = 50;

// Define the pins for the motor control
const int pwmPinLeft = 4;  // PWM output pin (connected to EN pin on H-Bridge)
const int pwmPinRight = 5;  // PWM output pin (connected to EN pin on H-Bridge)
const int dirPinLeft = 18;  // Direction control pin 1 (for left motor)
const int dirPinRight = 19;  // Direction control pin 2 (for right motor)

// Gloal variable to store current desired PWM for left and right motors
int desiredLeftPWM = 0;
int desiredLeftDirection = 1;
int desiredRightPWM = 0;
int desiredRightDirection = 1;

// Global variables to store the control signals for the left and right motors
int controlSignalLeft = 0;
int controlSignalRight = 0;
float KP = 0.0;
float KI = 0.0;
float KD = 0.0;
int ENABLE_CONTROL = 0;

// Define the pins for the encoder A and B channels
const int encoderPinLeftA = 6;
const int encoderPinLeftB = 7;
const int encoderPinRightA = 0;
const int encoderPinRightB = 1;
const int pulsesPerRevolution = 48;

// Variables for the encoder
volatile int leftDirection = 1;   // Direction of the left motor
volatile int rightDirection = 1;  // Direction of the right motor
volatile int leftPulseCount = 0;  // Pulse count for the left encoder
volatile int rightPulseCount = 0; // Pulse count for the right encoder
int lastEncodedLeft = 0;          // Store the last encoded state (A and B)
int lastEncodedRight = 0;         // Store the last encoded state (A and B)

// Rpm calculation variables
volatile unsigned long prevRpmCalcTime = 0;
const unsigned long rpmCalcInterval = 100;
unsigned int leftRPM = 0;
unsigned int rightRPM = 0;

// Interrupt function to update the encoder position, add IRAM_ATTR to run in IRAM
void IRAM_ATTR updateLeftEncoder() {
  // Read the current state of both encoder pins for left motor
  int left_encoder_A = digitalRead(encoderPinLeftA);  // Most Significant Bit
  int left_encoder_B = digitalRead(encoderPinLeftB);  // Least Significant Bit

  // Call encoder reading function for left motor
  int current_direction = 0;
  int current_pulse_count = 0;
  readEncoderValue(left_encoder_A, left_encoder_B, lastEncodedLeft, current_direction, current_pulse_count);
  leftPulseCount += current_pulse_count;
  leftDirection = current_direction * -1; // Motor is the opposite direction for wheels to go in the same direction
}

// Interrupt function to update the encoder position, add IRAM_ATTR to run in IRAM
void IRAM_ATTR updateRightEncoder() {
  // Read the current state of both encoder pins for right motor
  int right_encoder_A = digitalRead(encoderPinRightA);  // Most Significant Bit
  int right_encoder_B = digitalRead(encoderPinRightB);  // Least Significant Bit

  // Call encoder reading function for right motor
  int current_direction = 0;
  int current_pulse_count = 0;
  readEncoderValue(right_encoder_A, right_encoder_B, lastEncodedRight, current_direction, current_pulse_count);
  rightPulseCount += current_pulse_count;
  rightDirection = current_direction;
}

void setup() {
  Serial.begin(115200);
  pinMode(dirPinLeft, OUTPUT);
  pinMode(dirPinRight, OUTPUT);
  pinMode(pwmPinLeft, OUTPUT);
  pinMode(pwmPinRight, OUTPUT);
  pinMode(encoderPinLeftA, INPUT);
  pinMode(encoderPinLeftB, INPUT);
  pinMode(encoderPinRightA, INPUT);
  pinMode(encoderPinRightB, INPUT);
  digitalWrite(dirPinLeft, LOW);
  digitalWrite(dirPinRight, LOW);
  ledcAttach(pwmPinLeft, LEDC_FREQUENCY, LEDC_RESOLUTION_BITS);
  ledcAttach(pwmPinRight, LEDC_FREQUENCY, LEDC_RESOLUTION_BITS);
  ledcWrite(pwmPinLeft, 0);
  ledcWrite(pwmPinRight, 0);

  // Attach the interrupt service routine for the encoder pins
  attachInterrupt(digitalPinToInterrupt(encoderPinLeftA), updateLeftEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPinLeftB), updateLeftEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPinRightA), updateRightEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPinRightB), updateRightEncoder, CHANGE);

  // Set up Access Point
  Serial.println("Setting up Access Point...");
  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(local_IP, gateway, subnet);
  WiFi.softAP(ssid, password);
  Serial.print("AP IP address: ");
  Serial.println(WiFi.softAPIP());
  server.on("/", handleRoot);
  server.on("/setMotor", handleSetMotor);
  server.on("/setPID", handleControl);
  server.begin();
  Serial.print("HTTP server started at internal IP: ");
  Serial.print(local_IP);
  Serial.print("\n");
}

void loop() {
  // process the request and change motor speed
  if (millis() - serverPrevTime > serverInterval) {
    server.handleClient();
    serverPrevTime = millis();
  }

  // Pulse from encoder is done in interrupts, here we just calculate RPM and update
  // control signals with a defined frequency.
  if (millis() - prevRpmCalcTime > rpmCalcInterval) {
    // Calculate the RPM for the left and right motors
    rpmCalculation();
    // Log the RPM values
    Serial.printf("Left RPM: %d, Left Direction: %d, Right RPM: %d, Right Direction: %d\n", leftRPM, leftDirection, rightRPM, rightDirection);
    // Update the control signals based on the current RPM
    updateControlSignals();
    // Log the control signals
    // Serial.printf("Control Signal Left: %d, Control Signal Right: %d\n", controlSignalLeft, controlSignalRight);


    // Reset the pulse count for the next interval
    leftPulseCount = 0;
    rightPulseCount = 0;

    // Update the time for the next interval
    prevRpmCalcTime = millis();
  }
}

// Function to handle root URL "/"
void handleRoot() {
  server.send_P(200, "text/html", INDEX_HTML);
}

// Function to handle the motor control request
void handleSetMotor() {
  if (server.hasArg("speed") && server.hasArg("forwardBackward") && server.hasArg("turnRate")) {

    float motor_speed = server.arg("speed").toFloat(); // percent from 0 to 100
    String forward_backward = server.arg("forwardBackward"); // "Forward" or "Backward"
    float turn_rate = server.arg("turnRate").toFloat(); // percent from -50 to 50

    // Convert speed input to linear velocity
    float linear_velocity = (forward_backward == "Forward" ? 1 : -1) * (motor_speed / 100.0) * MAX_LINEAR_VELOCTY;
    float angular_velocity = (turn_rate/50.0) * MAX_ANGULAR_VELOCITY;

    int current_pwm_left, current_pwm_right;
    int current_direction_left, current_direction_right;
    prepareMotorSignals(linear_velocity, angular_velocity, 
                        current_pwm_left, current_direction_left, 
                        current_pwm_right, current_direction_right);

    // Send these signals to the motors
    sendMotorSignals(current_pwm_left, current_direction_left, current_pwm_right, current_direction_right);

    // Update the desired PWM values for the next iteration
    desiredLeftPWM = current_pwm_left;
    desiredRightPWM = current_pwm_right;
    desiredLeftDirection = current_direction_left;
    desiredRightDirection = current_direction_right;

    // Send response
    server.sendHeader("Access-Control-Allow-Origin", "*");
    server.send(200, "text/plain", "Motor updated");

    // Debugging output
    Serial.printf("Linear velocity: %.2f, Angular velocity: %.2f\n", linear_velocity, angular_velocity);
    Serial.printf("Left PWM: %d, Right PWM: %d\n", current_pwm_left, current_pwm_right);
  } else {
    server.send(400, "text/plain", "Bad Request");
  }
}

// Function to get Kp, Ki, Kd values from the webpage, along with the boolean value for control
void handleControl() {
  if (server.hasArg("kp") && server.hasArg("ki") && server.hasArg("kd") && server.hasArg("enabled")) {
    KP = server.arg("kp").toFloat();
    KI = server.arg("ki").toFloat();
    KD = server.arg("kd").toFloat();
    ENABLE_CONTROL = server.arg("enabled").toInt();
    server.send(200, "text/plain", "Control parameters updated");
    Serial.printf("Control parameters updated: KP: %f, KI: %f, KD: %f, ENABLED: %d\n", KP, KI, KD, ENABLE_CONTROL);
  } else {
    server.send(400, "text/plain", "Bad Request");
  }
}

// Function to prepare the motor signals based on the linear and angular velocity
void prepareMotorSignals(
  float linear_velocity, 
  float angular_velocity, 
  int& left_pwm,
  int& left_direction,
  int& right_pwm,
  int& right_direction
) {
  // Calculate the angular velocity of each wheel
  float omega_l = (linear_velocity - angular_velocity * WHEEL_BASE / 2) / WHEEL_RADIUS;
  float omega_r = (linear_velocity + angular_velocity * WHEEL_BASE / 2) / WHEEL_RADIUS;
  Serial.printf("OmegaL: %f, OmegaR: %f\n", omega_l, omega_r);

  // Convert the angular velocity to PWM signals
  convertAngularVelocityToPWM(omega_l, left_pwm, left_direction);
  convertAngularVelocityToPWM(omega_r, right_pwm, right_direction);

  // Incorporate control signals if needed
  if (ENABLE_CONTROL) {
    left_pwm = controlSignalLeft + left_pwm;
    right_pwm = controlSignalRight + right_pwm;
  }

  // Clip to the limits of PWM
  left_pwm = min(max(left_pwm, 0), LEDC_RESOLUTION);
  right_pwm = min(max(right_pwm, 0), LEDC_RESOLUTION);
}

// Function to convert angular velocity to PWM signal
void convertAngularVelocityToPWM(float omega, int& pwm_ref, int& direction) {
  if (omega > 0.0) {
    direction = 1.0;
    pwm_ref = mapf(omega, 0, MAX_WHEEL_VELOCTY, 0, (float)LEDC_RESOLUTION);
  } else {
    direction = LOW;
    pwm_ref = mapf(-omega, 0, MAX_WHEEL_VELOCTY, 0, (float)LEDC_RESOLUTION);
  }
}

// Function to map a value from one range to another in floating point
float mapf(float value, float inMin, float inMax, float outMin, float outMax) {
  return (value - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
}

// Function to send the motor signals to the motors
void sendMotorSignals(
  int left_pwm, 
  int left_direction, 
  int right_pwm, 
  int right_direction
) {
  // Set the direction of the motors - we need to be careful with the direction.
#if EXTRA_CARE
  // Extra care is needed to stop the motor before switching the direction.
  // If the previous direction is different from the current direction, we need to stop the motor first.
  if (left_direction != desiredLeftDirection) {
    ledcWrite(pwmPinLeft, 0);
  }
  if (right_direction != desiredRightDirection) {
    ledcWrite(pwmPinRight, 0);
  }
  delay(MOTOR_STOP_DELAY);
#endif

  // Set the direction of the motors
  digitalWrite(dirPinLeft, left_direction);
  digitalWrite(dirPinRight, right_direction);

  // Set the PWM signals for the motors
  ledcWrite(pwmPinLeft, left_pwm);
  ledcWrite(pwmPinRight, right_pwm);
}

void rpmCalculation() {
  leftRPM = (abs(leftPulseCount) * 60000) / ((pulsesPerRevolution) * (millis() - prevRpmCalcTime));
  rightRPM = (abs(rightPulseCount) * 60000) / ((pulsesPerRevolution) * (millis() - prevRpmCalcTime));
}

void updateControlSignals() {
  // Calculate the desired PWM for the left and right motors
  int current_pwm_left = map(leftRPM, 0, MAX_RPM, 0, LEDC_RESOLUTION);
  int current_pwm_right = map(rightRPM, 0, MAX_RPM, 0, LEDC_RESOLUTION);

  // PID control
  controlSignalLeft = pidControl(desiredLeftPWM, current_pwm_left, 1000, -LEDC_RESOLUTION, LEDC_RESOLUTION, KP, KI, KD);
  controlSignalRight = pidControl(desiredRightPWM, current_pwm_right, 1000, -LEDC_RESOLUTION, LEDC_RESOLUTION, KP, KI, KD);
}

// Purpose: update the direction and pulse count of the encoder based on the current encoder values
// Inputs: encoder_A_val, encoder_B_val, last_encoded_val, direction, pulse_count
// Outputs: last_encoded_val, direction, pulse_count
void readEncoderValue(
    int encoder_A_val,
    int encoder_B_val,
    int& last_encoded_val,
    int& direction,
    int& pulse_count
) {
  int MSB = encoder_A_val;
  int LSB = encoder_B_val;

  // Combine the two bits into a single integer for easier comparison
  int encoded = (MSB << 1) | LSB;
  
  // Compute the direction based on the changes in the encoder states
  int sum = (last_encoded_val << 2) | encoded;  // Combine previous and current state

  // Update the encoder position based on the direction
  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) {
    direction = 1;  // Clockwise
    pulse_count++;
  } else if (sum == 0b1110 || sum == 0b1000 || sum == 0b0001 || sum == 0b0111) {
    direction = -1;  // Counterclockwise
    pulse_count--;
  }

  // Update the last encoded state
  last_encoded_val = encoded;

  return;
}
