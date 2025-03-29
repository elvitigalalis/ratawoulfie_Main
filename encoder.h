/**
 * @file encoder.h
 * @brief High-efficiency quadrature encoder implementation using PIO for
 * Raspberry Pi Pico
 */

#ifndef ENCODER_H
#define ENCODER_H

#include "hardware/pio.h"
#include "pico/stdlib.h"
#include <limits.h>
#include <stdint.h>

class Encoder {
public:
  /**
   * @brief Construct a new Encoder object
   *
   * @param pio_instance PIO block to use (pio0 or pio1)
   * @param pin_a GPIO pin number for encoder channel A
   * @param pin_b GPIO pin number for encoder channel B (typically pin_a + 1)
   * @param max_step_rate Maximum expected steps per second (default: 50000)
   * @param reverse_direction Set to true to reverse counting direction
   */
  Encoder(PIO pio_instance, uint pin_a, uint pin_b = UINT32_MAX,
          uint max_step_rate = 50000, bool reverse_direction = false);

  /**
   * @brief Get the current encoder count
   *
   * @return int32_t Current position count
   */
  int32_t getCount();

  /**
   * @brief Reset the encoder count to zero
   */
  void resetCount();

  /**
   * @brief Set the encoder count to a specific value
   *
   * @param value Value to set the count to
   */
  void setCount(int32_t value);

  /**
   * @brief Calculate the current speed in counts per second
   *
   * @return float Speed in counts per second
   */
  float getSpeed();

  /**
   * @brief Calculate the rotational speed in RPM
   *
   * @param counts_per_revolution Number of encoder counts per revolution
   * @return float Speed in RPM
   */
  float getRPM(uint counts_per_revolution);

  /**
   * @brief Update speed calculations - call periodically for accurate speed
   */
  void update();

private:
  PIO pio;            // PIO block (pio0 or pio1)
  uint sm;            // State machine index
  int32_t last_count; // Previous encoder count for delta calculation
  absolute_time_t last_update_time; // Time of last speed update
  float current_speed;              // Current speed in counts per second
  bool direction_reversed;          // Flag to reverse counting direction

  // Configure and load the PIO program
  void initializePIO(uint pin_a, uint pin_b, uint max_step_rate);
};

#endif // ENCODER_H