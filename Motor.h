#include "encoder.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "hardware/pio.h"
#include "hardware/pwm.h"
#include "pico/stdlib.h"
#include <iostream>
#include <string>

// PID Constants
#define RIGHTMOTORKP 10
#define RIGHTMOTORKI 23 // 20 is better for 5V
#define LEFTMOTORKP 7.75
#define LEFTMOTORKI 20

// Pins
// 6, 7, 8, 9
// 18, 19, 20, 21

class Motor {
private:
  // Motor pins
  int H_BRIDGE_PIN_1;
  int H_BRIDGE_PIN_2;
  int ENCODER_PIN_1;
  int ENCODER_PIN_2;
  float MOTOR_CEPR; // Countable events per revolution, used for RPM
                    // calculations and dependent on motor used

  // See initializePWM()
  uint slice;
  uint channel1;
  uint channel2;

  // Motor control
  volatile float targetRPM = 0;
  volatile int targetPosition;

  // Motor state
  volatile int currentPosition = 0;
  volatile float currentRPM = 0;
  volatile bool motorOn = false;

  // PI variables
  float kp = 0;
  float ki = 0;
  volatile float integral = 0;
  absolute_time_t lastPIDTime;

  // PIO encoder variables
  PIO pio;
  uint sm;
  int32_t last_encoder_count;
  ;
  absolute_time_t last_rpm_time;

  // Setup functions
  void setup();
  void initializePWM();
  void initializeEncoder();
  float abs(float num);

public:
  // Initialization commands
  Motor(int motorPin1, int motorPin2, int encoderPinA, PIO pio_instance = pio0,
        uint state_machine = 0);
  void setPIDVariables(float Kp, float Ki) {
    kp = Kp;
    ki = Ki;
  }
  void start() { motorOn = true; }
  void stop() {
    motorOn = false;
    pwm_set_both_levels(slice, 0, 0);
  }
  void setRPM(float rpm) {
    targetRPM = rpm;
    start();
  }
  void setPosition(int position) { targetPosition = position; }

  // get commands
  float getTargetRPM() { return targetRPM; }
  float getRPM() { return currentRPM; }
  int getTargetPosition() { return targetPosition; }
  int getPosition() { return currentPosition; }

  // Update functions
  void updateEncoder();
  void updatePWM();
};