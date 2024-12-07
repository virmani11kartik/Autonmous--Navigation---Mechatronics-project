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

        // Determine which sensor to use for wall following
        int wall_distance = isLeftWall ? left_distance : right_distance;
        String steer_direction_close = isLeftWall ? "right" : "left";
        String steer_direction_far = isLeftWall ? "left" : "right";

        // Wall-following logic
        if (wall_distance < ideal_distance - distance_tolerance) {
            // Too close to the wall, steer away
            sendSteeringCommand(5, steer_direction_close);
        } else if (wall_distance > ideal_distance + distance_tolerance) {
            // Too far from the wall, steer toward
            sendSteeringCommand(5, steer_direction_far);
        } else {
            // Within the ideal distance range, move straight
            sendSteeringCommand(0, "straight");
        }

        // Small delay for stability
        delay(100);
    }
}
