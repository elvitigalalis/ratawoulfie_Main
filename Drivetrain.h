/**
 * @file Drivetrain.h
 * @brief Simplified differential drive system controller for two-wheeled robot
 */

#ifndef DRIVETRAIN_H
#define DRIVETRAIN_H

#include "Motor.h"
#include "pico/stdlib.h"
#include <math.h>

// Constants for turning control
#define ANGLEERROR 100 // Angle error threshold in centidegrees (0.01 degrees)
#define RIGHTADD 0.1   // Motor adjustment factor for right motor
#define LEFTADD 0.1    // Motor adjustment factor for left motor

class Drivetrain {
public:
  /**
   * @brief Construct a new Drivetrain object
   *
   * @param forwardRPM Default forward speed in RPM
   * @param KP Proportional gain for turning control
   * @param KI Integral gain for turning control
   * @param maxTurning Maximum turning speed in RPM
   * @param motorOne Pointer to left motor
   * @param motorTwo Pointer to right motor
   */
  Drivetrain(float forwardRPM, float KP, float KI, float maxTurning,
             Motor *motorOne, Motor *motorTwo);

  /**
   * @brief Move forward at the default speed
   */
  void forward();

  /**
   * @brief Stop both motors
   */
  void stop();

  /**
   * @brief Move forward by a specific distance
   *
   * @param dis Distance to move in encoder counts
   */
  void forwardDistance(int dis);

  /**
   * @brief Turn by a specific angle
   *
   * @param deg Angle in degrees (positive for right, negative for left)
   */
  void turn(int deg);

  /**
   * @brief Main control loop to be called periodically
   */
  void controlLoop();

private:
  // Motors
  Motor *leftMotor;
  Motor *rightMotor;

  // Motion control parameters
  float targetrpm;       // Default forward speed
  float kp;              // Proportional gain for turning
  float ki;              // Integral gain for turning
  float maxrpm;          // Maximum turning speed
  float turningIntegral; // Accumulated error for turning

  // State variables
  bool turnLoop; // Flag indicating turning in progress
  bool distance; // Flag indicating forward movement in progress
  int targetDis; // Target distance to travel
  int targetYaw; // Target orientation
  int yaw;       // Current orientation (simulated without IMU)

  // Timing
  absolute_time_t lastTime; // Last update time

  // Helper methods
  int positiveMod(int a, int b);
  void turningLoop();
  void driveLoop();
};

#endif // DRIVETRAIN_H