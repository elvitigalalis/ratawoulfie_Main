/**
 * main.cpp example
 * Example of using the Motor class with quadrature encoders
 */
#include "Motor.h"
#include "pico/stdlib.h"
#include <stdio.h>

int main() {
  stdio_init_all();

  // Initialize LED pin
  gpio_init(PICO_DEFAULT_LED_PIN);
  gpio_set_dir(PICO_DEFAULT_LED_PIN, true);

  // Blink LED to indicate startup
  gpio_put(PICO_DEFAULT_LED_PIN, 1);
  sleep_ms(3000);
  gpio_put(PICO_DEFAULT_LED_PIN, 0);

  // Initialize motors with PIO instances and state machines
  // Left motor uses PIO0, state machine 0
  // Right motor uses PIO0, state machine 1
  Motor leftMotor(19, 18, 20, pio0, 0);
  Motor rightMotor(6, 7, 8, pio0, 1);

  // Set PID constants
  leftMotor.setPIDVariables(LEFTMOTORKP, LEFTMOTORKI);
  rightMotor.setPIDVariables(RIGHTMOTORKP, RIGHTMOTORKI);

  // Set target RPM
  leftMotor.setRPM(200);
  rightMotor.setRPM(200);


  absolute_time_t start_time = get_absolute_time();
  float loop_time = 10000; // in ms

  // Main loop
  while (absolute_time_diff_us(start_time, get_absolute_time()) < loop_time * 1000) {
    // Update motor PWM (which also reads encoders)
    leftMotor.updatePWM();
    rightMotor.updatePWM();

    // Print status
    printf("Left: RPM=%.2f, Pos=%d | Right: RPM=%.2f, Pos=%d\n",
           leftMotor.getRPM(), leftMotor.getPosition(), rightMotor.getRPM(),
           rightMotor.getPosition());

    // Toggle LED to show program is running
    gpio_put(PICO_DEFAULT_LED_PIN, !gpio_get(PICO_DEFAULT_LED_PIN));

    // Delay to avoid flooding serial output
    sleep_ms(100);
  }

  return 0;
}