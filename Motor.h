#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "hardware/irq.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include <string>
#include <iostream>

// PID Constants
#define RIGHTMOTORKP 10
#define RIGHTMOTORKI 23 // 20 is better for 5V
#define LEFTMOTORKP 7.75
#define LEFTMOTORKI 20

// Pins
// 6, 7, 8, 9
// 18, 19, 20, 21

class Motor{
    private:
        // Motor pins
        int H_BRIDGE_PIN_1;
        int H_BRIDGE_PIN_2;
        int ENCODER_PIN_1;
        int ENCODER_PIN_2;
        float MOTOR_CEPR; // Countable events per revolution, used for RPM calculations and dependent on motor used

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

        // PIO variables
        PIO pio;
        uint sm;
        uint offset;
        int last_position;
        absolute_time_t last_rpm_time;

        // Setup functions
        void setup();
        void initializePWM();
        void initializePIO();
        float abs(float num);

    public:
        // Initialization commands
        // call setPIDvariables post-initialization
        Motor(int motorPin1, int motorPin2, int encoderPin1, int encoderPin2);
        void setPIDVariables(float Kp, float Ki){kp = Kp; ki = Ki;}
        void start(){motorOn = true;}
        void stop(){motorOn = false; pwm_set_both_levels(slice, 0, 0);}
        void setRPM(float rpm){targetRPM = rpm; start();}
        void setPosition(int position){targetPosition = position;}

        // get commands
        float getTargetRPM(){return targetRPM;}
        float getRPM(){return currentRPM;}
        int getTargetPosition(){return targetPosition;}
        int getPosition(){return currentPosition;}

        // Interrupt functions
        void encoderInterruptTime(uint gpio, uint32_t event_mask);
        void clockInterruptTime();

        // Control loop
        void updatePWM();
        void updateEncoder();
};