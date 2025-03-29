#pragma once
#include "Drivetrain.h"
#include <cstdlib>

class API {

public:
    static bool wallFront(Drivetrain* dr);
    static bool wallRight(Drivetrain* dr);
    static bool wallLeft(Drivetrain* dr);

    static void moveForward(Drivetrain* dr);
    static void turnRight(Drivetrain* dr);
    static void turnLeft(Drivetrain* dr);
};
