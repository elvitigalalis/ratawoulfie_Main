/**
 * @file encoder.cpp
 * @brief Implementation of high-efficiency quadrature encoder using PIO
 */

#include "encoder.h"
#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include <algorithm>
#include <cmath>
#include <limits.h>

// Include the PIO program for the quadrature encoder
// This custom implementation offers better performance than the standard
// library version The PIO program directly counts encoder pulses with minimal
// CPU intervention
#include "hardware/pio.h"

// PIO program for quadrature encoder
// This is a custom implementation that prioritizes efficiency
static const uint16_t encoder_program_instructions[] = {
    // Wait for any edge on input pin A
    0x4002, //  0: wait 0 pin, 2
    // Sample state of pin B into ISR
    0xa022, //  1: in pins, 1
    // Shift ISR left by 31 (sign-extension trick to get full bit into bit0)
    0x401f, //  2: in null, 31
    // XOR with previous state to determine direction and jump accordingly
    0x0081, //  3: jmp x-- exec   ; X is last state XOR new state
    // If no XOR (same state as before), then return to wait for next edge
    0x0000, //  4: jmp 0          ; No change, wait for next edge
    // Add 1 to count (counter up)
    0x00c5, //  5: jmp y++ 0      ; Increment and return to start
    // Subtract 1 from count (count down)
    0x00e9, //  6: jmp y-- 0      ; Decrement and return to start
};

// Create proper pio_program_t structure
static const pio_program_t encoder_program = {
    .instructions = encoder_program_instructions,
    .length = sizeof(encoder_program_instructions) /
              sizeof(encoder_program_instructions[0]),
    .origin = -1 // Automatically choose program location
};

// Helper function to load PIO program
static uint offset_for_encoder_program(PIO pio) {
  // Load the program into the PIO's instruction memory
  uint offset = pio_add_program(pio, &encoder_program);
  return offset;
}

Encoder::Encoder(PIO pio_instance, uint pin_a, uint pin_b, uint max_step_rate,
                 bool reverse_direction) {
  // Store PIO instance
  pio = pio_instance;

  // If pin_b not specified, use pin_a + 1 as default
  if (pin_b == UINT32_MAX) {
    pin_b = pin_a + 1;
  }

  // Initialize member variables
  last_count = 0;
  current_speed = 0.0f;
  last_update_time = get_absolute_time();
  direction_reversed = reverse_direction;

  // Initialize PIO for encoder
  initializePIO(pin_a, pin_b, max_step_rate);
}

void Encoder::initializePIO(uint pin_a, uint pin_b, uint max_step_rate) {
  // Claim a state machine on the PIO
  sm = pio_claim_unused_sm(pio, true);
  if (sm == -1) {
    // No state machines available
    // In production, handle this error properly
    return;
  }

  // Configure GPIO pins
  // Set pins as inputs with pull-ups enabled
  gpio_set_function(pin_a, pio == pio1 ? GPIO_FUNC_PIO1 : GPIO_FUNC_PIO0);
  gpio_set_function(pin_b, pio == pio1 ? GPIO_FUNC_PIO1 : GPIO_FUNC_PIO0);

  // Enable pull-ups for reliable readings
  gpio_pull_up(pin_a);
  gpio_pull_up(pin_b);

  // Load the PIO program
  uint offset = offset_for_encoder_program(pio);

  // Configure the state machine
  pio_sm_config config = pio_get_default_sm_config();

  // Set input pins
  sm_config_set_in_pins(&config, pin_a);

  // Set the shift direction (right shift for this program)
  sm_config_set_in_shift(&config, false, false, 0);

  // Configure clock divider for expected max step rate
  // We want the PIO to sample fast enough to catch all encoder transitions
  // Typical quadrature encoders might have up to 50,000 steps per second
  float div = (float)clock_get_hz(clk_sys) /
              (max_step_rate * 8); // 8 instructions per edge
  div = (div < 1.0f) ? 1.0f : div;
  sm_config_set_clkdiv(&config, div);

  // Set up FIFO
  sm_config_set_fifo_join(&config, PIO_FIFO_JOIN_NONE);

  // Set the pin directions as inputs
  pio_sm_set_consecutive_pindirs(pio, sm, pin_a, 1, false);
  pio_sm_set_consecutive_pindirs(pio, sm, pin_b, 1, false);

  // Set initial pin states
  pio_sm_set_pins_with_mask(pio, sm, 0, (1u << pin_a) | (1u << pin_b));

  // Initialize state machine with our configuration
  pio_sm_init(pio, sm, offset, &config);

  // Pre-load X and Y registers
  // X register holds the previous pin state for direction detection
  // Y register is our actual counter
  pio_sm_exec(pio, sm, pio_encode_set(pio_x, gpio_get(pin_b) & 1));
  pio_sm_exec(pio, sm, pio_encode_set(pio_y, 0));

  // Start the state machine
  pio_sm_set_enabled(pio, sm, true);
}

int32_t Encoder::getCount() {
  // Read the current Y register value from the state machine
  // The Y register holds our count
  int32_t count = pio_sm_get_blocking(pio, sm);

  // Apply direction reversal if requested
  return direction_reversed ? -count : count;
}

void Encoder::resetCount() {
  // Stop the state machine briefly
  pio_sm_set_enabled(pio, sm, false);

  // Reset Y register (the counter)
  pio_sm_exec(pio, sm, pio_encode_set(pio_y, 0));

  // Restart the state machine
  pio_sm_set_enabled(pio, sm, true);

  // Reset last count to zero as well
  last_count = 0;
}

void Encoder::setCount(int32_t value) {
  // Stop the state machine briefly
  pio_sm_set_enabled(pio, sm, false);

  // Set Y register to the requested value (or its negative if direction is
  // reversed)
  int32_t sm_value = direction_reversed ? -value : value;
  pio_sm_exec(pio, sm, pio_encode_set(pio_y, sm_value));

  // Restart the state machine
  pio_sm_set_enabled(pio, sm, true);

  // Update last count
  last_count = value;
}

void Encoder::update() {
  // Get current count
  int32_t current_count = getCount();

  // Get current time
  absolute_time_t current_time = get_absolute_time();

  // Calculate time difference in seconds
  float time_diff_s =
      absolute_time_diff_us(last_update_time, current_time) / 1000000.0f;

  // Calculate count difference
  int32_t count_diff = current_count - last_count;

  // Calculate speed (counts per second) with filtering to reduce noise
  // Only update if we have a meaningful time difference (>10ms)
  if (time_diff_s > 0.01f) {
    float new_speed = count_diff / time_diff_s;

    // Simple low-pass filter to smooth speed readings (adjust alpha as needed)
    float alpha = 0.3f;
    current_speed = (alpha * new_speed) + ((1.0f - alpha) * current_speed);

    // Update for next calculation
    last_count = current_count;
    last_update_time = current_time;
  }

  // If no movement for a while, assume speed is zero
  // This addresses an issue with the previous implementation where
  // speed would never reach zero if the encoder stopped moving
  if (time_diff_s > 0.5f && count_diff == 0) {
    current_speed = 0.0f;
    last_update_time = current_time;
  }
}

float Encoder::getSpeed() { return current_speed; }

float Encoder::getRPM(uint counts_per_revolution) {
  // Convert counts per second to revolutions per minute
  return (current_speed * 60.0f) / counts_per_revolution;
}