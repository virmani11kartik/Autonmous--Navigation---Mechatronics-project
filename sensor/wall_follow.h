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
void steer(int angle, const char* direction); // Implemented in auto.ino

// Define pins for the ToF sensors
#define SDA_PIN_FRONT 5
#define SCL_PIN_FRONT 4

#define SDA_PIN_LEFT 10
#define SCL_PIN_LEFT 1

#define SDA_PIN_RIGHT 19
#define SCL_PIN_RIGHT 18

// Create instances for each sensor
Adafruit_VL53L0X loxFront = Adafruit_VL53L0X();
Adafruit_VL53L0X loxLeft = Adafruit_VL53L0X();
Adafruit_VL53L0X loxRight = Adafruit_VL53L0X();

// Function to initialize the sensors
void initToFSensors() {
  // FRONT sensor
  Wire.begin(SDA_PIN_FRONT, SCL_PIN_FRONT);
  if (!loxFront.begin()) {
    Serial.println("Failed to initialize front ToF sensor");
    while (1);
  }

  // LEFT sensor
  Wire.begin(SDA_PIN_LEFT, SCL_PIN_LEFT);
  if (!loxLeft.begin()) {
    Serial.println("Failed to initialize left ToF sensor");
    while (1);
  }

  // RIGHT sensor
  Wire.begin(SDA_PIN_RIGHT, SCL_PIN_RIGHT);
  if (!loxRight.begin()) {
    Serial.println("Failed to initialize right ToF sensor");
    while (1);
  }
}

// Function to get distance from front sensor
int getFrontDistance() {
  VL53L0X_RangingMeasurementData_t measure;
  loxFront.rangingTest(&measure, false);
  if (measure.RangeStatus != 4) {
    return measure.RangeMilliMeter;
  } else {
    return -1; // Out of range
  }
}

// Function to get distance from left sensor
int getLeftDistance() {
  VL53L0X_RangingMeasurementData_t measure;
  loxLeft.rangingTest(&measure, false);
  if (measure.RangeStatus != 4) {
    return measure.RangeMilliMeter;
  } else {
    return -1; // Out of range
  }
}

// Function to get distance from right sensor
int getRightDistance() {
  VL53L0X_RangingMeasurementData_t measure;
  loxRight.rangingTest(&measure, false);
  if (measure.RangeStatus != 4) {
    return measure.RangeMilliMeter;
  } else {
    return -1; // Out of range
  }
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
    // Obstacle ahead, decide to turn
    if (d_left > d_right) {
      steer(45, "LEFT"); // Turn left
    } else {
      steer(45, "RIGHT"); // Turn right
    }
    delay(500); // Wait to complete the turn
    return;     // Skip further processing
  }

  // Wall alignment logic
  int alignment_error = d_right - d_left;
  if (abs(alignment_error) > 2) { // If misaligned
    int correction_angle = Kp_alignment * alignment_error;
    if (alignment_error > 0) {
      steer(correction_angle, "RIGHT");
    } else {
      steer(abs(correction_angle), "LEFT");
    }
    delay(100); // Allow time to adjust
    return;
  }

  // Wall-following logic
  if (abs(d_left - wall_distance_setpoint) < abs(d_right - wall_distance_setpoint)) {
    // Follow left wall
    int error = wall_distance_setpoint - d_left;
    int steering_angle = Kp_steering * error;

    if (error > 0) {
      steer(steering_angle, "LEFT");
    } else if (error < 0) {
      steer(abs(steering_angle), "RIGHT");
    } else {
      steer(0, "FORWARD");
    }
  } else {
    // Follow right wall
    int error = wall_distance_setpoint - d_right;
    int steering_angle = Kp_steering * error;

    if (error > 0) {
      steer(steering_angle, "RIGHT");
    } else if (error < 0) {
      steer(abs(steering_angle), "LEFT");
    } else {
      steer(0, "FORWARD");
    }
  }
}

#endif