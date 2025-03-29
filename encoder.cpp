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
    .length = sizeof(encoder_program_instructions) / sizeof(encoder_program_instructions[0]),
    .origin = 0
};

void quadrature_encoder_init(PIO pio, uint sm, uint pin_base, int max_step_rate) {
    // Set up the pins as inputs
    pio_sm_set_consecutive_pindirs(pio, sm, pin_base, 2, false);
    pio_gpio_init(pio, pin_base);
    pio_gpio_init(pio, pin_base + 1);
    gpio_pull_up(pin_base);
    gpio_pull_up(pin_base + 1);

    // Load the program into the PIO
    uint offset = pio_add_program(pio, &encoder_program);

    // Configure state machine
    pio_sm_config config = pio_get_default_sm_config();

    sm_config_set_in_pins(&config, pin_base);
    sm_config_set_jmp_pin(&config, pin_base);
    sm_config_set_in_shift(&config, false, false, 32);
    sm_config_set_fifo_join(&config, PIO_FIFO_JOIN_NONE);

    // Configure clock divider
    if (max_step_rate == 0) {
        sm_config_set_clkdiv(&config, 1.0f);
    } else {
        float div = (float) clock_get_hz(clk_sys) / (10 * max_step_rate);
        sm_config_set_clkdiv(&config, div);
    }

    // Initialize state machine
    pio_sm_init(pio, sm, offset, &config);
    pio_sm_set_enabled(pio, sm, true);
}