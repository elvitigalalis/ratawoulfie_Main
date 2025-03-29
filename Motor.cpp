/**
 * @file Motor.cpp
 * @brief Implementation of the Motor class with PID control
 */

#include "Motor.h"

// Helper function to clamp values
template <typename T> T clamp(const T &value, const T &low, const T &high) {
  return (value < low) ? low : ((value > high) ? high : value);
}

Motor::Motor(uint motorPin1, uint motorPin2, Encoder *encoder_instance,
             uint counts_per_rev) {
  H_BRIDGE_PIN_1 = motorPin1;
  H_BRIDGE_PIN_2 = motorPin2;
  encoder = encoder_instance;
  MOTOR_CEPR = counts_per_rev;

  targetRPM = 0.0f;
  currentRPM = 0.0f;
  currentPosition = 0;
  motorOn = false;

  // Default PID values
  kp = 0.5f;
  ki = 0.2f;
  kd = 0.0f;
  feedforward = 0.0f;
  integral = 0.0f;
  lastError = 0.0f;
  lastPIDTime = get_absolute_time();

  setup();
  initializePWM();
}

void Motor::setup() {
  // Configure motor driver pins for PWM
  gpio_set_function(H_BRIDGE_PIN_1, GPIO_FUNC_PWM);
  gpio_set_function(H_BRIDGE_PIN_2, GPIO_FUNC_PWM);
}

void Motor::initializePWM() {
  // Get PWM slice and channels for the H-bridge pins
  channel1 = pwm_gpio_to_channel(H_BRIDGE_PIN_1);
  channel2 = pwm_gpio_to_channel(H_BRIDGE_PIN_2);
  slice = pwm_gpio_to_slice_num(H_BRIDGE_PIN_1);

  // Configure PWM with default settings
  pwm_config config = pwm_get_default_config();

  // Set wrap value for 1000 steps (0-999)
  // With 125MHz system clock, this gives ~125kHz PWM frequency
  pwm_config_set_wrap(&config, 999);

  // Initialize PWM with stopped motor
  pwm_init(slice, &config, true);
  pwm_set_both_levels(slice, 0, 0);
}

float Motor::abs(float num) { return num < 0 ? -num : num; }

void Motor::setRPM(float rpm) {
  targetRPM = rpm;
  motorOn = true;

  // Reset integral when direction changes to avoid windup
  if ((targetRPM > 0 && integral < 0) || (targetRPM < 0 && integral > 0)) {
    integral = 0;
  }
}

void Motor::stop() {
  targetRPM = 0.0f;
  motorOn = false;
  integral = 0.0f;

  // Stop motor immediately
  pwm_set_both_levels(slice, 0, 0);
}

int32_t Motor::getPosition() { return encoder->getCount(); }

float Motor::getRPM() { return encoder->getRPM(MOTOR_CEPR); }

float Motor::getTargetRPM() { return targetRPM; }

void Motor::setPIDVariables(float p, float i, float d, float ff) {
  kp = p;
  ki = i;
  kd = d;
  feedforward = ff;
}

void Motor::updatePWM() {
  // Update encoder calculations
  encoder->update();
  currentRPM = encoder->getRPM(MOTOR_CEPR);
  currentPosition = encoder->getCount();

  // If motor is not enabled, don't update PWM
  if (!motorOn) {
    return;
  }

  // Calculate time since last update
  absolute_time_t currentTime = get_absolute_time();
  float deltaTime =
      absolute_time_diff_us(lastPIDTime, currentTime) / 1000000.0f;

  // PID control loop
  float error = abs(targetRPM) - abs(currentRPM);

  // Integral term with anti-windup
  integral += error * deltaTime;
  integral = clamp(integral, -100.0f, 100.0f);

  // Derivative term
  float derivative = (error - lastError) / deltaTime;
  lastError = error;

  // Calculate feed-forward term (baseline PWM based on target RPM)
  float ff = feedforward * abs(targetRPM);
  if (ff == 0.0f) {
    // Default feed-forward if not set (empirical: 300 + 0.7 * RPM)
    ff = 300.0f + 0.7f * abs(targetRPM);
  }

  // Calculate final PWM value
  float pwm = kp * error + ki * integral + kd * derivative + ff;

  // Limit PWM to valid range
  pwm = clamp(pwm, 0.0f, 999.0f);

  // Set direction based on target RPM sign
  if (targetRPM > 0) {
    // Forward
    pwm_set_chan_level(slice, channel1, (uint16_t)pwm);
    pwm_set_chan_level(slice, channel2, 0);
  } else if (targetRPM < 0) {
    // Reverse
    pwm_set_chan_level(slice, channel1, 0);
    pwm_set_chan_level(slice, channel2, (uint16_t)pwm);
  } else {
    // Stop
    pwm_set_chan_level(slice, channel1, 0);
    pwm_set_chan_level(slice, channel2, 0);
  }

  // Update time for next calculation
  lastPIDTime = currentTime;
}