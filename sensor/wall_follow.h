#ifndef WALL_FOLLOW_H
#define WALL_FOLLOW_H

#include <Wire.h>
#include <Adafruit_VL53L0X.h>
// #include <Adafruit_VL53L1X.h> 

/*
TOOD: Tuning parameters:
Wall follow threshold - Complete
Corner Hanlding - Complete for 90 degrees
Vive - Incomplete
Hybrid Mode - Incomplete
*/

/*
const int front_collision_threshold = 130;      // Minimum distance to obstacle in front
const float Kp_steering = 1.5;                  // Adjusted proportional gain for wall alignment
const float Kp_alignment = 0.5;                 // Adjusted proportional gain for misalignment correctio
*/

// Add new constants
const int OUT_OF_RANGE = -1;
const int MAX_SENSOR_RANGE = 2000;  // Maximum measurable disftance of the sensor in mm
const int WALL_FOLLOW_THRESHOLD = 150;          // Main parameter for wall following
// const int LEFT_WALL_THRESHOLD_PUSH = 180;       // NOT USED
// const int LEFT_WALL_THRESHOLD_PULL = 200;       // NOT USED
const int MAX_STEERING_ANGLE_PERCENT = 5;       // Maximum steering angle as a percentage of the maximum
const int CORNER_DETECTION_DISTANCE = 370;      // Main parameter for corner detection
const float SHARP_TURN_ANGLE = 22.5;            // Maximum turn angle for corners
// const float CORNER_KP = 2.0;                 // Aggressive steering for corners
const float NORMAL_TURN_ANGLE = 6.0;              // Stronger correction for wall following 
// const float normal_turn_offset = 1.8;        // Offset for normal turn angle (left)
const float NORMAL_TURN_PUSH = 4.5;              // AWAY FROM WALL (Wall following parameter)
const float NORMAL_TURN_PULL = 13;               // TOWARDS WALL (Wall following parameter)


// Speed control constants (%)
const int NORMAL_SPEED = 50;    // Reduced normal speed for better control
const int CORNER_SPEED = 50;    // Speed for corners
const int WALL_CLOSE_SPEED = 50;  // Speed when close to wall

// Filter parameters (NOT USED)
const int FILTER_WINDOW = 5;  // Number of samples to average
const int INVALID_READING = -999;  // Sentinel value for invalid readings

// Function declarations
void initToFSensors();
void readToFSensors(int &d_front, int &d_left, int &d_right);
void wallFollowLogic();
void sendSteeringCommand(int angle, const char* direction, int speed); // Changed from steer to sendSteeringCommand
float calculateSteeringAngle(float error);  // Add this line
float calculatePullSteeringAngle(int leftDistance); // Add function prototype

// Define pins
#define SDA_PIN 19    // Common SDA (Data line)
#define SCL_PIN 4     // Common SCL (Clock line)
#define XSHUT_FRONT 1 // Front sensor XSHUT pin
#define XSHUT_LEFT 5  // Left sensor XSHUT pin
#define XSHUT_RIGHT 18 // Right sensor XSHUT pin

// Adafruit_VL53L1X loxFront = Adafruit_VL53L1X(); // Uncomment for VL53L1X)
Adafruit_VL53L0X loxFront = Adafruit_VL53L0X();
Adafruit_VL53L0X loxLeft = Adafruit_VL53L0X();
Adafruit_VL53L0X loxRight = Adafruit_VL53L0X();

// PID controller for wall following - controls the steering angle
const float Kp_steering = 1.5;  // Proportional gain
const float Ki_steering = 0.0;  // Integral gain
const float Kd_steering = 0.0;  // Derivative gain
PIDController pidSteering(Kp_steering, Ki_steering, Kd_steering, 10, -MAX_STEERING_ANGLE, MAX_STEERING_ANGLE);

// Add sensor status flags
bool frontSensorOK = false;
bool leftSensorOK = false;
bool rightSensorOK = false;

// Simplify the initToFSensors() function to match working implementation
void initToFSensors() {
  // Configure I2C
  Wire.begin(SDA_PIN, SCL_PIN);
  delay(10);
  pinMode(XSHUT_FRONT, OUTPUT);
  pinMode(XSHUT_LEFT, OUTPUT);
  pinMode(XSHUT_RIGHT, OUTPUT);

  // Power down all sensors
  digitalWrite(XSHUT_FRONT, LOW);
  digitalWrite(XSHUT_LEFT, LOW);
  digitalWrite(XSHUT_RIGHT, LOW);
  delay(10);

  // // Initialize Front sensor (VL53L1X)
  // digitalWrite(XSHUT_FRONT, HIGH);
  // delay(50);
  // frontSensorOK = loxFront.begin(0x29);  // VL53L1X uses 0x29 by default
  // if (frontSensorOK) {
  //   loxFront.startRanging();
  //   loxFront.setTimingBudget(200); 
  // } else {
  //   Serial.println("Failed to initialize front sensor");
  // }

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

// int getFrontDistance() {
//   if (!frontSensorOK) return OUT_OF_RANGE;
//   if (loxFront.dataReady()) {
//     int distance = loxFront.distance();
//     loxFront.clearInterrupt();
//     return distance;
//   }
//   return OUT_OF_RANGE;
// }

// int getFrontDistance() {
//   if (!frontSensorOK) return OUT_OF_RANGE;
//   if (loxFront.dataReady()) {
//     int distance = loxFront.distance();
//     loxFront.clearInterrupt();
//     return distance;
//   }
//   return OUT_OF_RANGE;
// }

int getFrontDistance() {
  if (!frontSensorOK) return OUT_OF_RANGE;
  VL53L0X_RangingMeasurementData_t measure;
  loxFront.rangingTest(&measure, false);
  return (measure.RangeStatus != 4) ? measure.RangeMilliMeter : OUT_OF_RANGE;
}

int getLeftDistance() {
  if (!leftSensorOK) return OUT_OF_RANGE;
  VL53L0X_RangingMeasurementData_t measure;
  loxLeft.rangingTest(&measure, false);
  return (measure.RangeStatus != 4) ? measure.RangeMilliMeter : OUT_OF_RANGE;
}

int getRightDistance() {
  if (!rightSensorOK) return OUT_OF_RANGE;
  VL53L0X_RangingMeasurementData_t measure;
  loxRight.rangingTest(&measure, false);
  return (measure.RangeStatus != 4) ? measure.RangeMilliMeter : OUT_OF_RANGE;
}
// Function to read distances from all sensors
void readToFSensors(int &d_front, int &d_left, int &d_right) {
  d_front = getFrontDistance();
  d_left = getLeftDistance();
  d_right = getRightDistance();
}

void readSteeringResult(int &angle, const char* direction, int &speed) {
  // Placeholder function to read steering commands from web interface
  angle = 0;
  direction = "FORWARD";
  speed = NORMAL_SPEED;
}

// Simplify the wallFollowLogic function
void wallFollowLogic() {
    int d_front, d_left, d_right;
    readToFSensors(d_front, d_left, d_right);

    // Convert out-of-range readings to max sensor range
    int effective_front = (d_front == OUT_OF_RANGE || d_front > 8000) ? MAX_SENSOR_RANGE : d_front;
    int effective_left  = (d_left == OUT_OF_RANGE || d_left > 8000) ? MAX_SENSOR_RANGE : d_left;
    int effective_right = (d_right == OUT_OF_RANGE || d_right > 8000) ? MAX_SENSOR_RANGE : d_right;

    // Debugging: print sensor values
    Serial.print("Front: "); Serial.print(d_front == OUT_OF_RANGE ? "OOR" : String(effective_front));
    Serial.print(" Left: "); Serial.print(d_left == OUT_OF_RANGE ? "OOR" : String(effective_left));
    Serial.print(" Right: "); Serial.println(d_right == OUT_OF_RANGE ? "OOR" : String(effective_right));

    float steering_angle = 0;
    const char* direction = "FORWARD";
    int speed = NORMAL_SPEED;  // Default speed

    // Enhanced wall following logic
    // if (effective_front <= CORNER_DETECTION_DISTANCE) {
    //     // Corner handling
    //     if (effective_left > effective_right) {
    //         direction = "LEFT";
    //         steering_angle = SHARP_TURN_ANGLE;
    //     } else {
    //         direction = "RIGHT";
    //         steering_angle = SHARP_TURN_ANGLE;
    //     }
    //     speed = CORNER_SPEED;
    // } 
    // Compute steering angle based on left distance using PID controller
    steering_angle = pidSteering.compute(WALL_FOLLOW_THRESHOLD, effective_left);

    if (steering_angle > 0) {
        direction = "RIGHT";
        speed = WALL_CLOSE_SPEED;
        Serial.print("Turning right - steering angle: ");
        Serial.println(steering_angle);
    } else if (steering_angle < 0) {
        direction = "LEFT";
        speed = WALL_CLOSE_SPEED;
        Serial.print("Turning left - steering angle: ");
        Serial.println(steering_angle);
        steering_angle = -steering_angle;  // Make angle positive
    } else {
        direction = "FORWARD";
        speed = NORMAL_SPEED;
        Serial.println("Moving forward");
        steering_angle = 0;
    }
    // if (effective_left <= LEFT_WALL_THRESHOLD_PUSH) {
    //     // Left wall following - stronger corrections
    //     // direction = (effective_left < wall_distance_setpoint) ? "RIGHT" : "LEFT";
    //     // if (effective_left < wall_distance_setpoint) {
    //     //     direction = "RIGHT";
    //     //     steering_angle = NORMAL_TURN_ANGLE;
    //     // } else {
    //     //     direction = "LEFT";
    //     //     steering_angle = NORMAL_TURN_ANGLE;
    //     // }
        // direction = "RIGHT";
        // speed = WALL_CLOSE_SPEED;
        // // steering_angle = NORMAL_TURN_ANGLE;
        // steering_angle = NORMAL_TURN_PUSH;
        // Serial.print("Left wall following - steering angle: ");
        // Serial.println(steering_angle);
        // sendSteeringCommand((int)steering_angle, direction, speed);
    //   return;
    //     // // Adjust speed based on distance from wall
    //     // if (abs(effective_left - wall_distance_setpoint) > 100) {
    //     //     speed = WALL_CLOSE_SPEED;  // Slower when far from desired distance
    //     // }
    // } else if (effective_left > WALL_FOLLOW_THRESHOLD || effective_right <= WALL_FOLLOW_THRESHOLD) {
      
    //   else if (effective_left > WALL_FOLLOW_THRESHOLD) {
    // else if (effective_left > LEFT_WALL_THRESHOLD_PULL) {
    //     // Right wall following - stronger corrections
    //     // direction = (effective_right < wall_distance_setpoint) ? "LEFT" : "RIGHT";
    //     // steering_angle = NORMAL_TURN_ANGLE;
    //     // if (effective_right < wall_distance_setpoint) {
    //     //     direction = "LEFT";
    //     //     steering_angle = NORMAL_TURN_ANGLE;
    //     // } else {
    //     //     direction = "RIGHT";
    //     //     steering_angle = NORMAL_TURN_ANGLE;
    //     // }
        // direction = "LEFT";
        // speed = WALL_CLOSE_SPEED;
        // steering_angle = normal_turn_offset * NORMAL_TURN_ANGLE;
        // steering_angle = calculatePullSteeringAngle(effective_left);
        // Serial.print("Right wall following - steering angle: ");
        // Serial.println(steering_angle);

    //     // // Adjust speed based on distance from wall
    //     // if (abs(effective_right - wall_distance_setpoint) > 100) {
    //     //     speed = WALL_CLOSE_SPEED;  // Slower when far from desired distance
    //     // }
    //   sendSteeringCommand((int)steering_angle, direction, speed);
    //   return;
    // }

    // If no wall detected, move forward
    sendSteeringCommand((int)steering_angle, direction, speed);
}

// float calculateSteeringAngle(float error) {
//   float adjusted_error = tanh(Kp_steering * error); // Output between -1 and 1
//   float raw_steering = abs(adjusted_error) * MAX_STEERING_ANGLE;
//   if (raw_steering > 0) {
//     raw_steering = max(raw_steering, (float)MIN_STEERING_ANGLE);
//   }
  
//   return raw_steering;
// }


/*
  Calculate the steering angle based on the left distance (towards the wall)
  This function is used to adjust the steering angle when following the left wall
  param leftDistance: the distance from the left wall
  return: the steering angle in degrees
*/
float calculatePullSteeringAngle(int leftDistance) {
  if (leftDistance <= WALL_FOLLOW_THRESHOLD * 1.1) {
    return NORMAL_TURN_PULL / 3;
  } else if (leftDistance <= WALL_FOLLOW_THRESHOLD * 1.2) {
    return NORMAL_TURN_PULL / 2;
  } else if (leftDistance <= WALL_FOLLOW_THRESHOLD * 1.4) {
    return NORMAL_TURN_PULL / 1.5;
  } else {
    return NORMAL_TURN_PULL;
  }
}


// Optional: Add function for push steering angle (away from wall)

#endif
