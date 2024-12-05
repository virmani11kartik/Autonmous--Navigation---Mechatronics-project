#ifndef WALL_FOLLOW_H
#define WALL_FOLLOW_H

#include <Wire.h>
#include <Adafruit_VL53L0X.h>

// Constants
const int wall_distance_setpoint = 6;         // Desired wall distance (in inches)
const int front_collision_threshold = 4;      // Minimum distance to obstacle in front
const float Kp_steering = 2.0;                // Proportional gain for wall alignment
const float Kp_alignment = 1.5;               // Proportional gain for misalignment correction

// Function prototypes
void initToFSensors();
void readToFSensors(int &d_front, int &d_left, int &d_right);
void wallFollowLogic();
void sendSteeringCommand(int angle, const char* direction); // Changed from steer to sendSteeringCommand

// Define pins - use only 3.3V capable GPIO pins
#define SDA_PIN 19    // Changed from 21
#define SCL_PIN 4     // Changed from 22
#define XSHUT_FRONT 1 // Changed from 5
#define XSHUT_LEFT 5  // Changed from 18
#define XSHUT_RIGHT 18 // Changed from 23

// Create instances for each sensor
Adafruit_VL53L0X loxFront = Adafruit_VL53L0X();
Adafruit_VL53L0X loxLeft = Adafruit_VL53L0X();
Adafruit_VL53L0X loxRight = Adafruit_VL53L0X();

// Add sensor status flags
bool frontSensorOK = false;
bool leftSensorOK = false;
bool rightSensorOK = false;

// Simplify the initToFSensors() function to match working implementation
void initToFSensors() {
  // Configure I2C
  Wire.begin(SDA_PIN, SCL_PIN);
  delay(10);
  
  // Configure XSHUT pins as outputs
  pinMode(XSHUT_FRONT, OUTPUT);
  pinMode(XSHUT_LEFT, OUTPUT);
  pinMode(XSHUT_RIGHT, OUTPUT);

  // Power down all sensors
  digitalWrite(XSHUT_FRONT, LOW);
  digitalWrite(XSHUT_LEFT, LOW);
  digitalWrite(XSHUT_RIGHT, LOW);
  delay(10);

  // Initialize Front sensor
  digitalWrite(XSHUT_FRONT, HIGH);
  delay(50);
  frontSensorOK = loxFront.begin(0x30);
  if (!frontSensorOK) {
    Serial.println("Failed to initialize front sensor");
  }

  // Initialize Left sensor
  digitalWrite(XSHUT_LEFT, HIGH);
  delay(50);
  leftSensorOK = loxLeft.begin(0x31);
  if (!leftSensorOK) {
    Serial.println("Failed to initialize left sensor");
  }

  // Initialize Right sensor
  digitalWrite(XSHUT_RIGHT, HIGH);
  delay(50);
  rightSensorOK = loxRight.begin(0x32);
  if (!rightSensorOK) {
    Serial.println("Failed to initialize right sensor");
  }

  // Check if at least one sensor is working
  if (!frontSensorOK && !leftSensorOK && !rightSensorOK) {
    Serial.println("No sensors initialized! System halted.");
    while(1);
  }
  
  Serial.println("System initialized with working sensors!");
}

// Modified sensor reading functions
int getFrontDistance() {
  if (!frontSensorOK) return -1;
  VL53L0X_RangingMeasurementData_t measure;
  loxFront.rangingTest(&measure, false);
  return (measure.RangeStatus != 4) ? measure.RangeMilliMeter : -1;
}

int getLeftDistance() {
  if (!leftSensorOK) return -1;
  VL53L0X_RangingMeasurementData_t measure;
  loxLeft.rangingTest(&measure, false);
  return (measure.RangeStatus != 4) ? measure.RangeMilliMeter : -1;
}

int getRightDistance() {
  if (!rightSensorOK) return -1;
  VL53L0X_RangingMeasurementData_t measure;
  loxRight.rangingTest(&measure, false);
  return (measure.RangeStatus != 4) ? measure.RangeMilliMeter : -1;
}

// Function to read distances from all sensors
void readToFSensors(int &d_front, int &d_left, int &d_right) {
  d_front = getFrontDistance();
  d_left = getLeftDistance();
  d_right = getRightDistance();
}

// Modified wall-following logic to handle sensor failures
void wallFollowLogic() {
  int d_front, d_left, d_right;
  readToFSensors(d_front, d_left, d_right);

  // Debugging: print sensor values
  Serial.print("Front: "); Serial.print(d_front < 0 ? "N/A" : String(d_front));
  Serial.print(" Left: "); Serial.print(d_left < 0 ? "N/A" : String(d_left));
  Serial.print(" Right: "); Serial.println(d_right < 0 ? "N/A" : String(d_right));

  // Collision avoidance - only if front sensor is working
  if (frontSensorOK && d_front > 0 && d_front < front_collision_threshold) {
    // Prefer the working side sensor for turning decision
    if (!leftSensorOK && !rightSensorOK) {
      sendSteeringCommand(45, "RIGHT"); // Default turn if no side sensors
    } else if (!leftSensorOK) {
      sendSteeringCommand(45, "RIGHT");
    } else if (!rightSensorOK) {
      sendSteeringCommand(45, "LEFT");
    } else {
      sendSteeringCommand(45, (d_left > d_right) ? "LEFT" : "RIGHT");
    }
    delay(500);
    return;
  }

  // Wall following - adapt to working sensors
  if (leftSensorOK && (!rightSensorOK || d_left < d_right)) {
    // Follow left wall
    int error = wall_distance_setpoint - d_left;
    int steering_angle = Kp_steering * error;
    sendSteeringCommand(abs(steering_angle), error > 0 ? "LEFT" : "RIGHT");
  } else if (rightSensorOK) {
    // Follow right wall
    int error = wall_distance_setpoint - d_right;
    int steering_angle = Kp_steering * error;
    sendSteeringCommand(abs(steering_angle), error > 0 ? "RIGHT" : "LEFT");
  } else {
    // No working side sensors, move forward with caution
    sendSteeringCommand(0, "FORWARD");
  }
}

#endif