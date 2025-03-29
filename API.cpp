#include "API.h"

/* Wall detection functions commented out - LIDAR functionality removed
bool API::wallFront(Drivetrain* dr) {
    return dr->checkFrontWall();
}

bool API::wallRight(Drivetrain* dr) {
    return dr->checkRightWall();
}

bool API::wallLeft(Drivetrain* dr) {
    return dr->checkLeftWall();
}
*/

// Simple stubs for wall detection - return false since we have no LIDAR
bool API::wallFront(Drivetrain *dr) {
  // Simulation - always assume no wall
  return false;
}

bool API::wallRight(Drivetrain *dr) {
  // Simulation - always assume no wall
  return false;
}

bool API::wallLeft(Drivetrain *dr) {
  // Simulation - always assume no wall
  return false;
}

void API::moveForward(Drivetrain *dr) {
  // Modified to use standard forwardDistance function
  // This assumes one square is approximately 180 encoder counts
  // Adjust this value based on your specific robot and encoder setup
  dr->forwardDistance(180);
}

void API::turnRight(Drivetrain *dr) {
  // Turn 90 degrees to the right
  dr->turn(90);
}

void API::turnLeft(Drivetrain *dr) {
  // Turn 90 degrees to the left
  dr->turn(-90);
}