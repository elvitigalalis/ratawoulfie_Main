/**
 * @file API.h
 * @brief Simple API for robot control without LIDAR functionality
 */

#ifndef API_H
#define API_H

#include "Drivetrain.h"

class API {
public:
  /**
   * @brief Check if there is a wall in front
   * @note This is a stub since LIDAR is not available
   *
   * @param dr Drivetrain pointer
   * @return bool Always returns false (no wall detection)
   */
  static bool wallFront(Drivetrain *dr);

  /**
   * @brief Check if there is a wall to the right
   * @note This is a stub since LIDAR is not available
   *
   * @param dr Drivetrain pointer
   * @return bool Always returns false (no wall detection)
   */
  static bool wallRight(Drivetrain *dr);

  /**
   * @brief Check if there is a wall to the left
   * @note This is a stub since LIDAR is not available
   *
   * @param dr Drivetrain pointer
   * @return bool Always returns false (no wall detection)
   */
  static bool wallLeft(Drivetrain *dr);

  /**
   * @brief Move forward one square
   *
   * @param dr Drivetrain pointer
   */
  static void moveForward(Drivetrain *dr);

  /**
   * @brief Turn 90 degrees to the right
   *
   * @param dr Drivetrain pointer
   */
  static void turnRight(Drivetrain *dr);

  /**
   * @brief Turn 90 degrees to the left
   *
   * @param dr Drivetrain pointer
   */
  static void turnLeft(Drivetrain *dr);
};

#endif // API_H