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
#define XSHUT_RIGHT 10 // Changed from 23

// Create instances for each sensor
Adafruit_VL53L0X loxFront = Adafruit_VL53L0X();
Adafruit_VL53L0X loxLeft = Adafruit_VL53L0X();
Adafruit_VL53L0X loxRight = Adafruit_VL53L0X();

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
  if (!loxFront.begin(0x30)) {
    Serial.println("Failed to initialize front sensor");
    while(1);
  }

  // Initialize Left sensor
  digitalWrite(XSHUT_LEFT, HIGH);
  delay(50);
  if (!loxLeft.begin(0x31)) {
    Serial.println("Failed to initialize left sensor");
    while(1);
  }

  // Initialize Right sensor
  digitalWrite(XSHUT_RIGHT, HIGH);
  delay(50);
  if (!loxRight.begin(0x32)) {
    Serial.println("Failed to initialize right sensor");
    while(1);
  }
}

// Function to get distance from front sensor
int getFrontDistance() {
  VL53L0X_RangingMeasurementData_t measure;
  loxFront.rangingTest(&measure, false);
  return (measure.RangeStatus != 4) ? measure.RangeMilliMeter : -1;
}

// Function to get distance from left sensor
int getLeftDistance() {
  VL53L0X_RangingMeasurementData_t measure;
  loxLeft.rangingTest(&measure, false);
  return (measure.RangeStatus != 4) ? measure.RangeMilliMeter : -1;
}

// Function to get distance from right sensor
int getRightDistance() {
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

// Wall-following logic function
void wallFollowLogic() {
  int d_front, d_left, d_right;
  readToFSensors(d_front, d_left, d_right);

  // Debugging: print sensor values
  Serial.print("Front: "); Serial.print(d_front);
  Serial.print(" Left: "); Serial.print(d_left);
  Serial.print(" Right: "); Serial.println(d_right);

  // Collision avoidance
  if (d_front > 0 && d_front < front_collision_threshold) {
    if (d_left > d_right) {
      sendSteeringCommand(45, "LEFT");
    } else {
      sendSteeringCommand(45, "RIGHT");
    }
    delay(500);
    return;
  }

  // Wall alignment logic
  int alignment_error = d_right - d_left;
  if (abs(alignment_error) > 2) {
    int correction_angle = Kp_alignment * alignment_error;
    if (alignment_error > 0) {
      sendSteeringCommand(correction_angle, "RIGHT");
    } else {
      sendSteeringCommand(abs(correction_angle), "LEFT");
    }
    delay(100);
    return;
  }

  // Wall-following logic
  if (abs(d_left - wall_distance_setpoint) < abs(d_right - wall_distance_setpoint)) {
    // Follow left wall
    int error = wall_distance_setpoint - d_left;
    int steering_angle = Kp_steering * error;

    if (error > 0) {
      sendSteeringCommand(steering_angle, "LEFT");
    } else if (error < 0) {
      sendSteeringCommand(abs(steering_angle), "RIGHT");
    } else {
      sendSteeringCommand(0, "FORWARD");
    }
  } else {
    // Follow right wall
    int error = wall_distance_setpoint - d_right;
    int steering_angle = Kp_steering * error;

    if (error > 0) {
      sendSteeringCommand(steering_angle, "RIGHT");
    } else if (error < 0) {
      sendSteeringCommand(abs(steering_angle), "LEFT");
    } else {
      sendSteeringCommand(0, "FORWARD");
    }
  }
}

#endif