// Constants
const int ideal_distance = 15; // Ideal distance from the wall in cm
const int distance_tolerance = 3; // Allowable deviation in cm
const int obstacle_threshold = 10; // Threshold for detecting an obstacle in cm

// Function prototypes
int readTOF(); // Function to read TOF sensor (left, right, front)
void sendSteeringCommand(int angle, String direction); // Function to steer the vehicle

void wallFollowingRobot() {
    while (true) {
        // Read TOF sensor data
        int front_distance = readTOF("front");
        int left_distance = readTOF("left");
        int right_distance = readTOF("right");

        // Obstacle detection
        if (front_distance < obstacle_threshold) {
            // Obstacle ahead! Decide turn direction
            if (left_distance > right_distance) {
                sendSteeringCommand(45, "left"); // Turn left
                delay(500); // Allow time to reorient
            } else {
                sendSteeringCommand(45, "right"); // Turn right
                delay(500);
            }
            continue; // Skip to next loop iteration
        }

        // Wall-following logic
        if (left_distance < ideal_distance - distance_tolerance) {
            // Too close to the wall on the left, steer right
            sendSteeringCommand(5, "right");
        } else if (left_distance > ideal_distance + distance_tolerance) {
            // Too far from the wall on the left, steer left
            sendSteeringCommand(5, "left");
        } else {
            // Within the ideal distance range, move straight
            sendSteeringCommand(0, "straight");
        }

        // Small delay for stability
        delay(100);
    }
}
