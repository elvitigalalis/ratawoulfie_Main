#include "Motor.h"

const int encoder_pin = 20; // 20 or 8
const int encoder_pin2 = 8;
Motor Motor1 =
    Motor(encoder_pin - 1, encoder_pin - 2, encoder_pin, encoder_pin + 1);
Motor Motor2 =
    Motor(encoder_pin2 - 2, encoder_pin2 - 1, encoder_pin2, encoder_pin2 + 1);

bool controlLoop(repeating_timer_t *timer1) {
  Motor1.updatePWM();
  Motor2.updatePWM();
  return true;
};

void encoderInterrupt(uint gpio, uint32_t event_mask) {
  Motor1.encoderInterruptTime(gpio, event_mask);
  Motor2.encoderInterruptTime(gpio, event_mask);
};

bool clockInterrupt(repeating_timer_t *timer) {
  Motor1.clockInterruptTime();
  Motor2.clockInterruptTime();

  return true;
};

int main(int argc, char *argv[]) {
  stdio_init_all();
  gpio_init(PICO_DEFAULT_LED_PIN);
  gpio_set_dir(PICO_DEFAULT_LED_PIN, true);

  gpio_put(PICO_DEFAULT_LED_PIN, 1);
  sleep_ms(3000);
  gpio_put(PICO_DEFAULT_LED_PIN, 0);

  Motor1.setPIDVariables(RIGHTMOTORKP, RIGHTMOTORKI);
  Motor2.setPIDVariables(RIGHTMOTORKP, RIGHTMOTORKI);

  // Enable the timer interrupts
  struct repeating_timer controlTimer;
  add_repeating_timer_us(1000, controlLoop, NULL, &controlTimer);

  struct repeating_timer encoderTimer;
  add_repeating_timer_us(1000, clockInterrupt, NULL, &encoderTimer);

  // Enable the encoder GPIO interrupts
  gpio_set_irq_enabled(encoder_pin + 1, 0x8, true);
  gpio_set_irq_callback(encoderInterrupt);
  gpio_set_irq_enabled(encoder_pin2 + 1, 0x8, true);
  gpio_set_irq_callback(encoderInterrupt);
  irq_set_enabled(IO_IRQ_BANK0, true);

  // Set motor speed
  Motor1.setRPM(200);
  Motor2.setRPM(200);

  gpio_put(PICO_DEFAULT_LED_PIN, 1);
  while (true) {
    float currentRPM = Motor1.getRPM();
    float targetRPM = Motor1.getTargetRPM();
    float currentRPM2 = Motor2.getRPM();
    float targetRPM2 = Motor2.getTargetRPM();

    printf("Current RPM: %.2f, Target RPM: %.2f\n", currentRPM, targetRPM);
    printf("Current RPM2: %.2f, Target RPM2: %.2f\n", currentRPM2, targetRPM2);

    sleep_ms(500); // Add a small delay to make the output readable
  }
};