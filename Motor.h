/**
 * @file Motor.h
 * @brief Motor control class with PID control for use with the PIO encoder
 */

#ifndef MOTOR_H
#define MOTOR_H

#include "encoder.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "pico/stdlib.h"
#include <stdint.h>

class Motor {
public:
  /**
   * @brief Construct a new Motor object with PIO encoder
   *
   * @param motorPin1 First H-bridge control pin (PWM)
   * @param motorPin2 Second H-bridge control pin (PWM)
   * @param encoder_instance Pointer to an Encoder instance
   * @param counts_per_rev Number of encoder counts per motor revolution
   */
  Motor(uint motorPin1, uint motorPin2, Encoder *encoder_instance,
        uint counts_per_rev = 360);

  /**
   * @brief Set target RPM for the motor
   *
   * @param rpm Desired RPM (negative for reverse)
   */
  void setRPM(float rpm);

  /**
   * @brief Stop the motor
   */
  void stop();

  /**
   * @brief Get current motor position
   *
   * @return int32_t Current position in encoder counts
   */
  int32_t getPosition();

  /**
   * @brief Get current motor RPM
   *
   * @return float Current RPM
   */
  float getRPM();

  /**
   * @brief Get target motor RPM
   *
   * @return float Target RPM
   */
  float getTargetRPM();

  /**
   * @brief Set PID control variables
   *
   * @param p Proportional gain
   * @param i Integral gain
   * @param d Derivative gain (optional)
   * @param ff Feed-forward gain (optional)
   */
  void setPIDVariables(float p, float i, float d = 0.0f, float ff = 0.0f);

  /**
   * @brief Update motor PWM based on PID control
   * This should be called periodically from a timer
   */
  void updatePWM();

private:
  // Hardware pins
  uint H_BRIDGE_PIN_1;
  uint H_BRIDGE_PIN_2;

  // PWM configuration
  uint slice;
  uint channel1;
  uint channel2;

  // Encoder
  Encoder *encoder;
  uint MOTOR_CEPR; // Counts per revolution

  // Motor state
  float targetRPM;
  float currentRPM;
  int32_t currentPosition;
  bool motorOn;

  // PID control variables
  float kp;
  float ki;
  float kd;
  float feedforward;
  float integral;
  float lastError;
  absolute_time_t lastPIDTime;

  // Setup functions
  void setup();
  void initializePWM();

  // Utility functions
  float abs(float num);
};

#endif // MOTOR_H