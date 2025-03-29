#include "encoder.h"

const uint16_t encoder_program_instructions[]{
    0x0000, // 00 => 00: stop
    0x0104, // 00 => 01: reverse
    0x0204, // 00 => 10: forward
    0x0000, // 00 => 11: stop
    0x0204, // 01 => 00: forward
    0x0000, // 01 => 01: stop
    0x0000, // 01 => 10: stop
    0x0104, // 01 => 11: reverse
    0x0104, // 10 => 00: reverse
    0x0000, // 10 => 01: stop
    0x0000, // 10 => 10: stop
    0x0204, // 10 => 11: forward
    0x0000, // 11 => 00: stop
    0x0204, // 11 => 01: forward
    0x6001, // reverse: jmp Y-- stop
    0x80a0, // stop: mov ISR, Y
    0x6020, // push noblock
    0xa022, // out ISR, 2
    0x4042, // in PINS, 2
    0xa0c1, // mov OSR, ISR
    0xa0e1, // mov PC, ISR
    0x01a0, // forward: mov Y, ~Y
    0x6104, // jmp Y--, forward_cont
    0x01a0, // forward_cont: mov Y, ~Y
};

const struct pio_program encoder_program = {
    .instructions = encoder_program_instructions,
    .length = sizeof(encoder_program_instructions) /
              sizeof(encoder_program_instructions[0]),
    .origin = 0};

void quadrature_encoder_init(PIO pio, uint sm, uint pin_base,
                             int max_step_rate) {
  printf("Initializing quadrature encoder on PIO%d, SM%d, pins %d,%d\n",
         pio == pio0 ? 0 : 1, sm, pin_base, pin_base + 1);

  // First clear any previous configuration
  pio_sm_set_enabled(pio, sm, false);

  // Set up the pins as inputs
  pio_sm_set_consecutive_pindirs(pio, sm, pin_base, 2, false);
  pio_gpio_init(pio, pin_base);
  pio_gpio_init(pio, pin_base + 1);
  gpio_pull_up(pin_base);
  gpio_pull_up(pin_base + 1);

  // Check pin values
  printf("Initial pin states: A(%d)=%d, B(%d)=%d\n", pin_base,
         gpio_get(pin_base), pin_base + 1, gpio_get(pin_base + 1));

  // Load the program into the PIO
  uint offset = pio_add_program(pio, &encoder_program);
  printf("Loaded encoder program at offset %d\n", offset);

  // Configure state machine
  pio_sm_config config = pio_get_default_sm_config();

  sm_config_set_in_pins(&config, pin_base);
  sm_config_set_jmp_pin(&config, pin_base);
  sm_config_set_in_shift(&config, false, false, 32);
  sm_config_set_fifo_join(
      &config, PIO_FIFO_JOIN_RX); // Join both FIFOs to expand RX capacity

  // Configure clock divider
  float div = 1.0f;
  if (max_step_rate > 0) {
    div = (float)clock_get_hz(clk_sys) / (10 * max_step_rate);
  }
  printf("Setting PIO clock divider to %.2f\n", div);
  sm_config_set_clkdiv(&config, div);

  // Initialize state machine
  pio_sm_init(pio, sm, offset, &config);

  // Clear FIFOs before enabling
  pio_sm_clear_fifos(pio, sm);

  // Enable the state machine
  pio_sm_set_enabled(pio, sm, true);
  printf("Encoder state machine enabled\n");

  // Force a push to test the FIFO
  pio_sm_exec(pio, sm,
              pio_encode_set(pio_y, 0x1234)); // Set Y register to test value
  pio_sm_exec(pio, sm, pio_encode_mov(pio_isr, pio_y)); // Move Y to ISR
  pio_sm_exec(pio, sm, pio_encode_push(false, false));  // Push to FIFO

  printf("Test value pushed to PIO\n");
  sleep_ms(10);

  // Check if our test push worked
  printf("FIFO level after test push: %d\n", pio_sm_get_rx_fifo_level(pio, sm));
  if (pio_sm_get_rx_fifo_level(pio, sm) > 0) {
    uint32_t test_val = pio_sm_get(pio, sm);
    printf("Test value read back: 0x%08x\n", test_val);
  } else {
    printf("ERROR: Could not read back test value! FIFO not working.\n");

    // Additional diagnostics
    printf("PIO status: ctrl=0x%08x, fstat=0x%08x\n", pio->ctrl, pio->fstat);
    printf("SM%d config: clkdiv=0x%08x, execctrl=0x%08x, shiftctrl=0x%08x\n",
           sm, pio->sm[sm].clkdiv, pio->sm[sm].execctrl, pio->sm[sm].shiftctrl);
  }
}

int32_t quadrature_encoder_get_count(PIO pio, uint sm) {
  // Check if FIFO is empty
  if (pio_sm_get_rx_fifo_level(pio, sm) == 0) {
    printf("FIFO empty for state machine %dn", sm);
    return 0;
  }

  int32_t ret = 0;
  // Read all from FIFO, but we'll use the most recent value
  uint fifo_count = pio_sm_get_rx_fifo_level(pio, sm);
  printf("FIFO count: %d\n", fifo_count);

  for (uint i = 0; i < fifo_count; i++) {
    ret = pio_sm_get(pio, sm);
  }

  return ret;
}