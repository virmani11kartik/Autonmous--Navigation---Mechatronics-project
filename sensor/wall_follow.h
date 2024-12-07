#ifndef WALL_FOLLOW_H
#define WALL_FOLLOW_H

#include <Wire.h>
#include <Adafruit_VL53L0X.h>

/*
TOOD: Tuning parameters:
Wall follow threshold - Needs tuning for proper wall following (only tune wall distance setpoint after
susccessful tuning the threshold)
Normal Steering angle and Speed - Needs tuning for proper wall following
Corner tuning: Good for 180 degrees, still needs tuning for 90 degrees

*/


// Constants
const int wall_distance_setpoint = 150;         // Target distance from wall
const int front_collision_threshold = 150;      // Minimum distance to obstacle in front
const float Kp_steering = 1.5;                  // Adjusted proportional gain for wall alignment
const float Kp_alignment = 0.5;                 // Adjusted proportional gain for misalignment correction

// Add new constants
const int OUT_OF_RANGE = -1;
const int MAX_SENSOR_RANGE = 2000;  // Maximum measurable disftance of the sensor in mm
const int WALL_FOLLOW_THRESHOLD = 250;          // Detect walls
const int MAX_STEERING_ANGLE = 30; // Maximum steering angle in degrees
const int MIN_STEERING_ANGLE = 10; // Minimum steering angle when wall is detected

// Scaling factor to normalize error (e.g., wall_distance_setpoint)
const float ERROR_SCALING_FACTOR = wall_distance_setpoint;

// Add these new constants after other constants
const int CORNER_DETECTION_DISTANCE = 300;      // Detect corners earlier
const int SHARP_TURN_ANGLE = 45;               // Maximum turn angle for corners
const float CORNER_KP = 2.0;                 // Aggressive steering for corners
const int NORMAL_TURN_ANGLE = 20;              // Stronger correction for wall following (increased from 20)

// Add new constants for speed control
const int NORMAL_SPEED = 400;    // Reduced normal speed for better control
const int CORNER_SPEED = 300;    // Speed for corners
const int WALL_CLOSE_SPEED = 350;  // Speed when close to wall

// Add filter constants after other constants
const int FILTER_WINDOW = 5;  // Number of samples to average
const int INVALID_READING = -999;  // Sentinel value for invalid readings

// Function prototypes
void initToFSensors();
void readToFSensors(int &d_front, int &d_left, int &d_right);
void wallFollowLogic();
void sendSteeringCommand(int angle, const char* direction, int speed); // Changed from steer to sendSteeringCommand
float calculateSteeringAngle(float error);  // Add this line

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

// Add filter buffers
struct SensorBuffer {
    int values[FILTER_WINDOW];
    int index;
    bool isFull;
    
    SensorBuffer() : index(0), isFull(false) {
        for(int i = 0; i < FILTER_WINDOW; i++) values[i] = INVALID_READING;
    }
};

SensorBuffer frontBuffer;
SensorBuffer leftBuffer;
SensorBuffer rightBuffer;

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

// Simplify the wallFollowLogic function
void wallFollowLogic() {
    int d_front, d_left, d_right;
    readToFSensors(d_front, d_left, d_right);

    // Convert out-of-range readings to max sensor range
    int effective_front = (d_front == OUT_OF_RANGE || d_front > 8000) ? MAX_SENSOR_RANGE : d_front;
    int effective_left  = (d_left == OUT_OF_RANGE || d_left > 8000) ? MAX_SENSOR_RANGE : d_left;
    int effective_right = (d_right == OUT_OF_RANGE || d_right > 8000) ? MAX_SENSOR_RANGE : d_right;

    // Debugging: print sensor values
    Serial.print("Front: "); Serial.print(d_front == OUT_OF_RANGE ? "OOR" : String(d_front));
    Serial.print(" Left: "); Serial.print(d_left == OUT_OF_RANGE ? "OOR" : String(d_left));
    Serial.print(" Right: "); Serial.println(d_right == OUT_OF_RANGE ? "OOR" : String(d_right));

    float steering_angle = 0;
    const char* direction = "FORWARD";
    int speed = NORMAL_SPEED;  // Default speed

    // Enhanced wall following logic
    if (effective_front <= CORNER_DETECTION_DISTANCE) {
        // Corner handling
        if (effective_left > effective_right) {
            direction = "LEFT";
            steering_angle = SHARP_TURN_ANGLE;
        } else {
            direction = "RIGHT";
            steering_angle = SHARP_TURN_ANGLE;
        }
        speed = CORNER_SPEED;
    } else if (effective_left <= WALL_FOLLOW_THRESHOLD) {
        // Left wall following - stronger corrections
        // direction = (effective_left < wall_distance_setpoint) ? "RIGHT" : "LEFT";
        // if (effective_left < wall_distance_setpoint) {
        //     direction = "RIGHT";
        //     steering_angle = NORMAL_TURN_ANGLE;
        // } else {
        //     direction = "LEFT";
        //     steering_angle = NORMAL_TURN_ANGLE;
        // }
        direction = "RIGHT";
        speed = WALL_CLOSE_SPEED;
        steering_angle = NORMAL_TURN_ANGLE;
        
        // // Adjust speed based on distance from wall
        // if (abs(effective_left - wall_distance_setpoint) > 100) {
        //     speed = WALL_CLOSE_SPEED;  // Slower when far from desired distance
        // }
    } else if (effective_right <= WALL_FOLLOW_THRESHOLD) {
        // Right wall following - stronger corrections
        // direction = (effective_right < wall_distance_setpoint) ? "LEFT" : "RIGHT";
        // steering_angle = NORMAL_TURN_ANGLE;
        // if (effective_right < wall_distance_setpoint) {
        //     direction = "LEFT";
        //     steering_angle = NORMAL_TURN_ANGLE;
        // } else {
        //     direction = "RIGHT";
        //     steering_angle = NORMAL_TURN_ANGLE;
        // }
        direction = "LEFT";
        speed = WALL_CLOSE_SPEED;
        steering_angle = NORMAL_TURN_ANGLE;

        // // Adjust speed based on distance from wall
        // if (abs(effective_right - wall_distance_setpoint) > 100) {
        //     speed = WALL_CLOSE_SPEED;  // Slower when far from desired distance
        // }
    }

    // Send steering command with speed
    sendSteeringCommand((int)steering_angle, direction, speed);
}

// Add a new function to calculate the steering angle
float calculateSteeringAngle(float error) {
  // Use a non-linear function to start steering earlier
  float adjusted_error = tanh(Kp_steering * error); // Output between -1 and 1
  float raw_steering = abs(adjusted_error) * MAX_STEERING_ANGLE;
  
  // Apply minimum steering angle if there's any significant error
  if (raw_steering > 0) {
    raw_steering = max(raw_steering, (float)MIN_STEERING_ANGLE);
  }
  
  return raw_steering;
}

#endif
