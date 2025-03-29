#include "Motor.h"

Motor::Motor(int motorPin1, int motorPin2, int encoderPin1, int encoderPin2) {
  H_BRIDGE_PIN_1 = motorPin1;
  H_BRIDGE_PIN_2 = motorPin2;
  ENCODER_PIN_1 = encoderPin1;
  ENCODER_PIN_2 = encoderPin2;
  MOTOR_CEPR = 360;                  // For calculating RPM
  lastPIDTime = get_absolute_time(); // For calculating integral value
  setup();
  initializePWM();
}

void Motor::setup() {
  gpio_set_function(H_BRIDGE_PIN_1, GPIO_FUNC_PWM);
  gpio_set_function(H_BRIDGE_PIN_2, GPIO_FUNC_PWM);

  gpio_set_input_enabled(ENCODER_PIN_1, true);
  gpio_set_input_enabled(ENCODER_PIN_2, true);
  gpio_set_dir(ENCODER_PIN_1, false);
  gpio_set_dir(ENCODER_PIN_2, false);
  gpio_pull_up(ENCODER_PIN_1);
  gpio_pull_up(ENCODER_PIN_2);
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

void Motor::encoderInterruptTime(uint gpio, uint32_t event_mask) {
  /*
      To be used in the GPIO interrupt callback for determining
      the RPM of a motor. The RPM is averaged between the last three readings,
      which is equivalent to calculating the RPM after one full revolution.
  */
  int currentDir = 0;
  currentDir = gpio_get(ENCODER_PIN_2);
  absolute_time_t currentTime = get_absolute_time();
  int64_t dif = absolute_time_diff_us(lastTime, currentTime);
  lastTime = currentTime;
  difs[vindex++] = dif;
  vindex %= 3;

  currentRPM =
      60000000.0 / (MOTOR_CEPR / 4 * (difs[0] + difs[1] + difs[2]) / 3);

  if (currentDir) {
    currentPosition += 1;
  } else {
    currentPosition -= 1;
    currentRPM *= -1;
  }
}

void Motor::clockInterruptTime() {
  /*
      Used in a clock interrupt callback to set the RPM to 0 if the GPIO
      interrupt hasn't been triggered after a set amount of time.
  */
  if (absolute_time_diff_us(lastTime, get_absolute_time()) > 10000) {
    currentRPM = 0;
  }
}

void Motor::updatePWM() {
  /*
      Used in a clock interrupt callback to set RPM values of motors based on a
     PI loop. A feedforward value is used to decrease the time needed for the
     motor to hit the target RPM after turning on.
  */
  if (!motorOn) {
  } else {
    absolute_time_t pidCurrentTime = get_absolute_time();
    float error = abs(targetRPM) - abs(currentRPM);
    integral += error * (absolute_time_diff_us(lastPIDTime, pidCurrentTime) /
                         60000000.0);
    if (integral > 100) {
      integral = 100;
    }
    float feedForward = 537 + 0.8 * abs(targetRPM); // for vbatt of 7.6V
    float pwm = kp * error + ki * integral + feedForward;
    if (pwm > 999) {
      pwm = 999;
    }
    if (targetRPM >
        0) { // We should find a way to condense these if statements.
      pwm_set_chan_level(slice, channel1, pwm);
      pwm_set_chan_level(slice, channel2, 0);
    } else {
      pwm_set_chan_level(slice, channel1, 0);
      pwm_set_chan_level(slice, channel2, pwm);
    }
    lastPIDTime = pidCurrentTime;
  }
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
