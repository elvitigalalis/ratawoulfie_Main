#include "API.h"

bool API::wallFront(Drivetrain* dr) {
    return dr->checkFrontWall();
}

bool API::wallRight(Drivetrain* dr) {
    return dr->checkRightWall();
}

bool API::wallLeft(Drivetrain* dr) {
    return dr->checkLeftWall();
}

void API::moveForward(Drivetrain* dr) {
    dr->driveForwardsOneSquare();
}

void API::turnRight(Drivetrain* dr) {
    dr->turnRight();
}

void API::turnLeft(Drivetrain* dr) {
    dr->turnLeft();
}
