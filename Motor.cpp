#include "Motor.h"

Motor::Motor(int motorPin1, int motorPin2, int encoderPinA, PIO pio_instance,
             uint state_machine) {
  H_BRIDGE_PIN_1 = motorPin1;
  H_BRIDGE_PIN_2 = motorPin2;
  ENCODER_PIN_1 = encoderPinA;
  ENCODER_PIN_2 = encoderPinA + 1;
  MOTOR_CEPR = 360; // For RPM calculations
  pio = pio_instance;
  sm = state_machine;
  last_encoder_count = 0;
  lastPIDTime = get_absolute_time();
  last_rpm_time = get_absolute_time();

  setup();
  initializePWM();
  initializeEncoder();
}

void Motor::setup() {
  gpio_set_function(H_BRIDGE_PIN_1, GPIO_FUNC_PWM);
  gpio_set_function(H_BRIDGE_PIN_2, GPIO_FUNC_PWM);
}

void Motor::initializePWM() {
  /*
      RP Pico has multiple PWM slices, each with two channels that
      have the same configuration and wrap values but can have different
      duty cycles.
  */
  channel1 = pwm_gpio_to_channel(H_BRIDGE_PIN_1);
  channel2 = pwm_gpio_to_channel(H_BRIDGE_PIN_2);
  slice = pwm_gpio_to_slice_num(H_BRIDGE_PIN_1);
  pwm_config config = pwm_get_default_config();
  pwm_init(slice, &config, false);
  pwm_set_wrap(slice, 999);
  pwm_set_both_levels(slice, 0, 0);
  pwm_set_enabled(slice, true);
}

void Motor::initializeEncoder() {
  // 50000 steps/second is max rate for encoder
  quadrature_encoder_init(pio, sm, ENCODER_PIN_1, 50000);
}

float Motor::abs(float num) {
  /*
      For calculating absolute values since cmath isn't included
      in the Motor file. This function could probably be deleted with
      #include <cmath> moved to this file.
  */
  if (num < 0) {
    return num * -1;
  } else {
    return num;
  }
}

void Motor::updateEncoder() {
  int32_t new_count = quadrature_encoder_get_count(pio, sm);

  // Change in position
  int32_t delta = new_count - last_encoder_count;
  last_encoder_count = new_count;

  currentPosition = new_count;

  absolute_time_t current_time = get_absolute_time();
  int64_t time_diff_us = absolute_time_diff_us(last_rpm_time, current_time);

  // Update RPM after 50ms minimum difference between last and current time
  if (time_diff_us > 50000) {
    currentRPM = (delta * 60000000.0f) / (MOTOR_CEPR * time_diff_us);
    last_rpm_time = current_time;
  }

  if (time_diff_us > 500000 && delta == 0) {
    currentRPM = 0;
  }
}

void Motor::updatePWM() {
  updateEncoder();

  if (!motorOn) {
    return;
  }

  absolute_time_t pidCurrentTime = get_absolute_time();
  // PI Control Loop for motor speed
  float error = abs(targetRPM) - abs(currentRPM);
  integral += error * (absolute_time_diff_us(lastPIDTime, pidCurrentTime) /
                       60000000.0f);
  // Deadband
  if (integral > 100) {
    integral = 100;
  }

  // Calculate PWM with feedforward value
  float feedForward = 537 + 0.8f * abs(targetRPM); // for vbatt of 7.6V
  float pwm = kp * error + ki * integral + feedForward;

  // Deadband
  if (pwm > 999) {
    pwm = 999;
  }

  // Set direction based on target RPM sign
  if (targetRPM > 0) { // We should find a way to condense these if statements.
    pwm_set_chan_level(slice, channel1, pwm);
    pwm_set_chan_level(slice, channel2, 0);
  } else {
    pwm_set_chan_level(slice, channel1, 0);
    pwm_set_chan_level(slice, channel2, pwm);
  }

  lastPIDTime = pidCurrentTime;
}

// Test program
/*
Motor Motor1 = Motor(19, 18, 21, 20);

bool controlLoop(repeating_timer_t *timer1){
    Motor1.updatePWM();
    return true;
};

void encoderInterrupt(uint gpio, uint32_t event_mask){
    Motor1.encoderInterruptTime(gpio, event_mask);
};

bool clockInterrupt(repeating_timer_t *timer){
    Motor1.clockInterruptTime();
    return true;
};

int main(int argc, char* argv[]) {
    stdio_init_all();
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, true);

    gpio_put(PICO_DEFAULT_LED_PIN, 1);
    sleep_ms(3000);
    gpio_put(PICO_DEFAULT_LED_PIN, 0);

    Motor1.setPIDVariables(RIGHTMOTORKP, RIGHTMOTORKI);

    struct repeating_timer controlTimer;
    add_repeating_timer_us(1000, controlLoop, NULL, &controlTimer);

    struct repeating_timer encoderTimer;
    add_repeating_timer_us(1000, clockInterrupt, NULL, &encoderTimer);


    // gpio_set_irq_enabled(8, 0x8, true);
    gpio_set_irq_enabled(21, 0x8, true);
    gpio_set_irq_callback(encoderInterrupt);
    irq_set_enabled(IO_IRQ_BANK0, true);

    Motor1.setRPM(200);

    gpio_put(PICO_DEFAULT_LED_PIN, 1);
    while(true){
        std::string thing = std::to_string(Motor1.getRPM()) + "\n";
        // std::string thing = std::to_string(Motor1.getRPM()) + " " +
std::to_string(Motor1.getTargetRPM()) + "\n"; printf(thing.c_str());
    }
};
*/
