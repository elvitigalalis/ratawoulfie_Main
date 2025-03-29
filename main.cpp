/**
 * @file main.cpp
 * @brief Main program for testing the PIO encoder with motor control
 */

#include "Drivetrain.h"
#include "Motor.h"
#include "encoder.h"
#include "hardware/irq.h"
#include "hardware/pio.h"
#include "pico/stdlib.h"
#include <stdio.h>

/* IMU and LIDAR definitions removed
// Define UART pins and instances
#define UART_IMU uart0
#define UART_RX_PIN 1
#define BAUD_RATE 115200
#define DATA_BITS 8
#define STOP_BITS 1
#define PARITY UART_PARITY_NONE

#define UART_LIDAR uart1
#define LIDAR_TX_PIN 4
#define LIDAR_RX_PIN 5
#define LIDAR_BAUD_RATE 921600
*/

// Define encoder pins
#define LEFT_ENCODER_PIN_A 20
#define RIGHT_ENCODER_PIN_A 8

// Define motor pins
#define LEFT_MOTOR_PIN_1 19
#define LEFT_MOTOR_PIN_2 18
#define RIGHT_MOTOR_PIN_1 6
#define RIGHT_MOTOR_PIN_2 7

// PID constants
#define TURNINGKP 1.0f
#define TURNINGKI 0.1f
#define LEFTMOTORKP 0.5f
#define LEFTMOTORKI 0.2f
#define RIGHTMOTORKP 0.5f
#define RIGHTMOTORKI 0.2f

// Global objects
Encoder *leftEncoder;
Encoder *rightEncoder;
Motor *leftMotor;
Motor *rightMotor;
Drivetrain *drivetrain;

// Function declarations
bool motorControlLoop(repeating_timer_t *timer);

/* IMU and LIDAR function declarations removed
void initIMU();
void initLidar();
void uartIMUInterrupt();
void uartLidarInterrupt();
*/

int main() {
    printf("Starting Test...\n");
  // Initialize standard I/O
  stdio_init_all();

  // Initialize status LED
  gpio_init(PICO_DEFAULT_LED_PIN);
  gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);

  // Signal startup
  gpio_put(PICO_DEFAULT_LED_PIN, 1);
  sleep_ms(500);
  gpio_put(PICO_DEFAULT_LED_PIN, 0);

  // Initialize encoders (use both PIO instances to maximize resources)
  printf("Initializing encoders...\n");
  leftEncoder = new Encoder(pio0, LEFT_ENCODER_PIN_A);
  rightEncoder = new Encoder(pio1, RIGHT_ENCODER_PIN_A);

  // Initialize motors with encoders
  leftMotor = new Motor(LEFT_MOTOR_PIN_1, LEFT_MOTOR_PIN_2, leftEncoder);
  rightMotor = new Motor(RIGHT_MOTOR_PIN_1, RIGHT_MOTOR_PIN_2, rightEncoder);

  // Set PID constants
  leftMotor->setPIDVariables(LEFTMOTORKP, LEFTMOTORKI);
  rightMotor->setPIDVariables(RIGHTMOTORKP, RIGHTMOTORKI);

  // Create drivetrain controller
  drivetrain =
      new Drivetrain(200, TURNINGKP, TURNINGKI, 200, leftMotor, rightMotor);

  /* IMU and LIDAR initialization removed
  // Initialize sensors
  initIMU();
  initLidar();

  // Wait for initialization
  sleep_ms(500);
  drivetrain->initLidar();
  */

  // Wait for initialization
  sleep_ms(500);

  // Set up motor control timer (1ms interval)
  repeating_timer_t controlTimer;
  add_repeating_timer_ms(1, motorControlLoop, NULL, &controlTimer);

  // Ready signal
  for (int i = 0; i < 3; i++) {
    gpio_put(PICO_DEFAULT_LED_PIN, 1);
    sleep_ms(100);
    gpio_put(PICO_DEFAULT_LED_PIN, 0);
    sleep_ms(100);
  }

  printf("PIO Encoder and Motor Control Test\n");
  printf("----------------------------------\n");

  // Basic test sequence
  printf("Starting test sequence...\n\n");

  // 1. Forward motion
  printf("1. Moving forward for 2 seconds\n");
  drivetrain->forward();
  sleep_ms(2000);
  drivetrain->stop();
  sleep_ms(1000);

  // 2. Turn right 90 degrees
  printf("2. Turning right 90 degrees\n");
  drivetrain->turn(90);
  sleep_ms(1000);

  // 3. Forward motion
  printf("3. Moving forward for 2 seconds\n");
  drivetrain->forward();
  sleep_ms(2000);
  drivetrain->stop();
  sleep_ms(1000);

  // 4. Turn left 90 degrees
  printf("4. Turning left 90 degrees\n");
  drivetrain->turn(-90);
  sleep_ms(1000);

  /* Wall detection test removed
  // 5. Wall detection
  printf("5. Wall detection test\n");
  bool frontWall = drivetrain->checkFrontWall();
  bool rightWall = drivetrain->checkRightWall();
  bool leftWall = drivetrain->checkLeftWall();
  float frontDistance = drivetrain->checkFrontWallDistance();

  printf("   Front wall: %s\n", frontWall ? "Detected" : "Not detected");
  printf("   Right wall: %s\n", rightWall ? "Detected" : "Not detected");
  printf("   Left wall: %s\n", leftWall ? "Detected" : "Not detected");
  printf("   Front wall distance: %.1f mm\n", frontDistance);
  */

  printf("\nTest complete!\n");

  // Continuous monitoring loop
  printf("\nEntering monitoring mode. Press Ctrl+C to exit.\n");
  while (true) {
    // Print motor status every second
    sleep_ms(1000);

    printf("Left motor: Position=%ld, RPM=%.2f | Right motor: Position=%ld, "
           "RPM=%.2f\n",
           leftMotor->getPosition(), leftMotor->getRPM(),
           rightMotor->getPosition(), rightMotor->getRPM());

    // Toggle LED to indicate program is running
    gpio_put(PICO_DEFAULT_LED_PIN, !gpio_get(PICO_DEFAULT_LED_PIN));
  }

  return 0;
}

// Timer callback for motor control
bool motorControlLoop(repeating_timer_t *timer) {
  // Update motor PWM values based on PID control
  leftMotor->updatePWM();
  rightMotor->updatePWM();

  // Run drivetrain control logic
  drivetrain->controlLoop();

  return true; // Keep timer running
}

/* IMU and LIDAR initialization functions removed
// Initialize IMU communication
void initIMU() {
    // Initialize UART for IMU
    uart_init(UART_IMU, BAUD_RATE);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);

    uart_set_hw_flow(UART_IMU, false, false);
    uart_set_format(UART_IMU, DATA_BITS, STOP_BITS, PARITY);
    uart_set_fifo_enabled(UART_IMU, true);

    // Set up interrupt for IMU data
    irq_set_exclusive_handler(UART0_IRQ, uartIMUInterrupt);
    irq_set_enabled(UART0_IRQ, true);

    // Enable UART receiver timeout interrupt
    uart_set_irq_enables(UART_IMU, true, false);
}

// Initialize LIDAR communication
void initLidar() {
    // Initialize UART for LIDAR
    uart_init(UART_LIDAR, LIDAR_BAUD_RATE);
    gpio_set_function(LIDAR_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(LIDAR_RX_PIN, GPIO_FUNC_UART);

    uart_set_hw_flow(UART_LIDAR, false, false);
    uart_set_format(UART_LIDAR, DATA_BITS, STOP_BITS, PARITY);
    uart_set_fifo_enabled(UART_LIDAR, false);

    // Set up interrupt for LIDAR data
    irq_set_exclusive_handler(UART1_IRQ, uartLidarInterrupt);
    irq_set_enabled(UART1_IRQ, true);

    // Enable UART receiver interrupt
    uart_set_irq_enables(UART_LIDAR, true, false);
}

// UART interrupt handler for IMU
void uartIMUInterrupt() {
    if (drivetrain != NULL) {
        drivetrain->setYaw();
    }
}

// UART interrupt handler for LIDAR
void uartLidarInterrupt() {
    if (drivetrain != NULL) {
        drivetrain->uart_lidar_irq();
    }
}
*/