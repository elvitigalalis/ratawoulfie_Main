/**
 * @file Drivetrain.cpp
 * @brief Implementation of simplified differential drive system controller
 */

#include "Drivetrain.h"

Drivetrain::Drivetrain(float forwardRPM, float KP, float KI, float maxTurning,
                       Motor *motorOne, Motor *motorTwo) {
  // Store motion control parameters
  targetrpm = forwardRPM;
  kp = KP;
  ki = KI;
  maxrpm = maxTurning;

  // Store motor pointers
  leftMotor = motorOne;
  rightMotor = motorTwo;

  // Initialize state variables
  turnLoop = false;
  distance = false;
  targetDis = 0;
  targetYaw = 0;
  yaw = 0;
  turningIntegral = 0.0f;

  // Initialize timing
  lastTime = get_absolute_time();

  /* IMU and LIDAR initialization removed
  // Initialize IMU buffer
  chars_rxed = 0;
  for (int i = 0; i < 19; i++) {
      readingbuff[i] = 0;
  }

  // Initialize LIDAR buffers
  bufferIndex = 0;
  lidarPointIndex = 0;
  for (int i = 0; i < 331; i++) {
      buffer[i] = 0;
  }
  for (int i = 0; i < 320; i++) {
      lidarPoints[i] = 0;
      xcor[i] = 0.0f;
      ycor[i] = 0.0f;
  }
  */
}

int Drivetrain::positiveMod(int a, int b) {
  if (a > b) {
    return positiveMod(a - b, b);
  } else if (a < 0) {
    return positiveMod(a + b, b);
  } else {
    return a;
  }
}

/* LIDAR functions removed
void Drivetrain::cartesianConvert() {
    // Takes 320 LIDAR points and converts them to x, y coordinates.
    for (int i = 0; i < 80; i++) {
        int r = lidarPoints[i];
        if (r == 0) {
            xcor[i] = 0;
            ycor[i] = 0;
        }
        else {
            float theta = 2 * 3.1415 * (52.5 / 79 * i - 52.5)/ 360;
            xcor[i] = r * cos(theta);
            ycor[i] = r * sin(theta);
        }
    }
    // ... rest of function
}

float Drivetrain::checkFrontWallDistance() {
    // Takes LIDAR points 150-170 (front points) and averages the y distance.
    int pointOne = 150; // first point corresponding to front wall (position in
array) int pointTwo = 170; // last point corresponding to front wall int sum =
0; int zeroCount = 0; cartesianConvert(); for (int i = pointOne; i < pointTwo +
1; i++) { if (ycor[i] == 0) { zeroCount += 1; } else { sum += ycor[i];
        }
    }
    return sum / (float(pointTwo - pointOne + 1) - zeroCount);
}
*/

void Drivetrain::forward() {
  leftMotor->setRPM(targetrpm);
  rightMotor->setRPM(targetrpm);
}

void Drivetrain::stop() {
  leftMotor->stop();
  rightMotor->stop();
}

void Drivetrain::forwardDistance(int dis) {
  // Store current position as starting point
  targetDis = leftMotor->getPosition() + dis;
  distance = true;

  // This will be handled by controlLoop()
  while (distance) {
    sleep_ms(1); // Prevent CPU hogging

    /* Wall detection removed
    frontWallDist = checkFrontWallDistance();
    */
  }
}

void Drivetrain::turn(int deg) { // positive is right, negative is left
  stop(); // Moving forward commands do not stop the motors

  // Calculate new target yaw
  // In the original code, updateYaw() would be called here to read from IMU
  targetYaw = positiveMod((deg + targetYaw + 17999), 36000) - 17999;
  turnLoop = true;

  // This will be handled by controlLoop()
  while (turnLoop) {
    sleep_ms(1); // Prevent CPU hogging
  }

  turningIntegral = 0;
  stop();
}

void Drivetrain::turningLoop() {
  /*
      Called from controlLoop() during turning.
  */
  absolute_time_t currentTime = get_absolute_time();

  // For demo purposes, simulate the yaw changing
  // In real application with IMU, this would use updateYaw() to get sensor data
  yaw += (targetYaw > yaw) ? 100 : -100;

  // Determines direction of turn
  bool right = false;
  if (yaw - targetYaw > 0) {
    right = false;
  } else {
    right = true;
  }
  if (abs(yaw - targetYaw) > 18000) {
    right = !right;
  }

  // Determines turning RPM
  float error = abs(yaw - targetYaw);
  if (error > 18000) {
    error = 36000 - error;
  }
  turningIntegral +=
      error * (absolute_time_diff_us(lastTime, currentTime) / 60000000.0);
  float turningrpm = kp * error + ki * turningIntegral;

  // Limit turning RPM to maximum
  turningrpm = (turningrpm > maxrpm) ? maxrpm : turningrpm;

  if (abs(error) < ANGLEERROR) {
    // Stops turning when target angle is reached
    turningrpm = 0;
    turnLoop = false;
  }

  // Apply turning speeds to motors
  if (right) {
    leftMotor->setRPM(turningrpm);
    rightMotor->setRPM(turningrpm * -1);
  } else {
    leftMotor->setRPM(turningrpm * -1);
    rightMotor->setRPM(turningrpm);
  }
  lastTime = currentTime;
}

void Drivetrain::driveLoop() {
  /*
      Called from controlLoop() during straight movement.
  */
  // In original code: if ((leftMotor->getPosition() > targetDis - 10) ||
  // (frontWallDist < WALLCUTOFF)) Removed wall detection condition
  if (leftMotor->getPosition() > targetDis - 10) {
    distance = false;
  } else {
    // Adjusts RPM of either motor if angle is off
    float error = abs(yaw - targetYaw);
    if (error > 18000) {
      error = 36000 - error;
    }
    bool right = false;
    if (yaw - targetYaw > 0) {
      right = false;
    } else {
      right = true;
    }
    if (abs(yaw - targetYaw) > 18000) {
      right = !right;
    }
    if (right) {
      error *= -1;
    }

    // Adjust motor speeds to maintain straight line
    float rightSpeed = targetrpm + RIGHTADD * error;
    float leftSpeed = targetrpm - LEFTADD * error;

    // Limit speed adjustments
    rightSpeed =
        (rightSpeed > targetrpm + 100) ? (targetrpm + 100) : rightSpeed;
    leftSpeed = (leftSpeed > targetrpm + 100) ? (targetrpm + 100) : leftSpeed;

    rightMotor->setRPM(rightSpeed);
    leftMotor->setRPM(leftSpeed);
  }
}

void Drivetrain::controlLoop() {
  /*
      Determines what the robot does based on active flags.
      Call this periodically from a timer.
  */
  if (turnLoop) {
    turningLoop();
  } else if (distance) {
    driveLoop();
  }
  // No else case needed - robot is idle
}

/* IMU and LIDAR functions removed
void Drivetrain::setYaw() {
    // UART interrupt callback for receiving data from the IMU.
    if (UART_IMU == NULL) {
        return; // No IMU configured
    }

    while (uart_is_readable(UART_IMU)) {
        uint8_t ch = uart_getc(UART_IMU);
        readingbuff[chars_rxed] = ch;
        chars_rxed++;
        if (chars_rxed > 18) {
            chars_rxed = 0;
        }
    }
}

void Drivetrain::updateYaw() {
    // Used for processing data from the IMU.
    uint8_t csum_raw, csum_calc;

    // Toggle debug pin if available
    #ifdef PICO_DEFAULT_LED_PIN
    gpio_xor_mask(0x04);
    #endif

    if (chars_rxed == 3) {
        // Not enough data received yet
    }

    // Extract yaw from buffer
    yaw = (readingbuff[4] << 8) | readingbuff[3];

    // Verify checksum
    csum_raw = readingbuff[18];
    csum_calc = readingbuff[2] + readingbuff[3] + readingbuff[4] +
readingbuff[5] + readingbuff[6] + readingbuff[7] + readingbuff[8] +
readingbuff[9] + readingbuff[10] + readingbuff[11] + readingbuff[12] +
readingbuff[13] + readingbuff[14];

    if (chars_rxed != 19) {
        // Not a complete packet
    }

    if (csum_raw != csum_calc) {
        // Checksum error
    }

    chars_rxed = 0;
}

void Drivetrain::initLidar() {
    // Part 2 of LIDAR initialization. Sends a test command, verifies the
response,
    // and sends a command to start scanning.
    if (UART_LIDAR == NULL) {
        return; // No LIDAR configured
    }

    uint8_t command[9] = {0xA5, 0xA5, 0xA5, 0xA5, 0x00, 0x60, 0x00, 0x00, 0x60};
    uart_write_blocking(UART_LIDAR, command, 9);

    bool check = true;
    // Waits for a full response
    while (buffer[8] != 0x62) {
        // Wait for correct response
        sleep_ms(1); // Small delay to prevent CPU hogging
    }

    // Checks if response is good
    for (int i = 0; i < 8; i++) {
        if (i == 4) {
            if (buffer[i] != 0x01) {
                check = false;
            }
        } else {
            if (buffer[i] != command[i]) {
                check = false;
            }
        }
    }

    // Changes command array to send command to start scanning
    command[5] = 0x63;
    command[8] = 0x63;
    bufferIndex = 0;

    // Sends command
    uart_write_blocking(UART_LIDAR, command, 9);
    while (buffer[8] != 0x65) {
        // Wait for correct response
        sleep_ms(1); // Small delay to prevent CPU hogging
    }

    sleep_ms(10);
    bufferIndex = 0;

    // Resets buffer
    #ifdef PICO_DEFAULT_LED_PIN
    gpio_put(PICO_DEFAULT_LED_PIN, 1);
    #endif

    for (int i = 0; i < 331; i++) {
        buffer[i] = 0;
    }
}

void Drivetrain::uart_lidar_irq() {
    // UART interrupt callback for receiving LIDAR data.
    if (UART_LIDAR == NULL) {
        return; // No LIDAR configured
    }

    while (uart_is_readable(UART_LIDAR)) {
        buffer[bufferIndex] = uart_getc(UART_LIDAR);
        if (bufferIndex == 330) {
            bufferIndex = 0;
            for (int i = 10; i < 330; i += 2) {
                int distance = (buffer[i] | ((buffer[i + 1] << 8))) & 0x01ff;
                lidarPointIndex = (i - 8) / 2;

                if (lidarPointIndex < 81) {
                    lidarPointIndex = 80 - lidarPointIndex;
                } else {
                    lidarPointIndex = -lidarPointIndex + 240;
                }

                int address = buffer[4];
                if (address == 0x1) {
                    lidarPoints[159 - (lidarPointIndex - 1)] = distance;
                }
                if (address == 0x2) {
                    lidarPoints[320 - (lidarPointIndex)] = distance;
                }
            }
        } else {
            bufferIndex += 1;
        }
    }
}

bool Drivetrain::checkFrontWall() {
    // Counts number of points within a space in front of the mouse to determine
if there's a wall. cartesianConvert(); int count = 0; for (int i = 0; i < 320;
i++) { if (ycor[i] < 70 && ycor[i] > 20) { if (xcor[i] < 65 && xcor[i] > -65) {
                count += 1;
            }
        }
    }
    return count > 45;
}

bool Drivetrain::checkRightWall() {
    cartesianConvert();
    int count = 0;
    for (int i = 0; i < 320; i++) {
        if (ycor[i] < 130 && ycor[i] > -60) {
            if (xcor[i] < 90 && xcor[i] > 30) {
                count += 1;
            }
        }
    }
    return count > 50;
}

bool Drivetrain::checkLeftWall() {
    cartesianConvert();
    int count = 0;
    for (int i = 0; i < 320; i++) {
        if (ycor[i] < 130 && ycor[i] > -60) {
            if (xcor[i] < -30 && xcor[i] > -90) {
                count += 1;
            }
        }
    }
    return count > 50;
}
*/