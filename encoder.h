#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "hardware/gpio.h"

extern const uint16_t encoder_program_instructions[];
extern const struct pio_program encoder_program;

void quadrature_encoder_init(PIO pio, uint sm, uint pin_base, int max_step_rate);

int32_t quadrature_encoder_get_count(PIO pio, uint sm);

