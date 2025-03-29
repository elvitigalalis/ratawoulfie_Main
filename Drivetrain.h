#include "Motor.h"
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "hardware/uart.h"
#include <cmath>

#define TURNINGKP 0.01
#define TURNINGKI 0

#define SQUAREENCODERCOUNTS                                                    \
  134                 // Nummber of encoder counts to traverse one square
#define WALLCUTOFF 60 // Distance from wall where mouse stops

#define LEFTADD                                                                \
  0.08 // Determines how much mouse should turn when driving straight if its
       // angle is off
#define RIGHTADD 0.08
#define ANGLEERROR                                                             \
  300 // Mouse stops turning when it's within 3 degrees of the target angle

#define UART_IMU uart0
#define UART_LIDAR uart1
#define BAUD_RATE 115200
#define DATA_BITS 8
#define STOP_BITS 1
#define PARITY UART_PARITY_NONE

#define UART_TX_PIN 0
#define UART_RX_PIN 1

// Note: Figure out how to handle invalid LIDAR readings

class Drivetrain {
private:
  volatile float targetrpm;
  float kp; // PI control for turning
  float ki;
  float maxrpm; // Maximum RPM during a turn

  volatile int16_t yaw = 0;
  volatile int targetYaw = 0;
  volatile bool turnLoop = false;
  volatile float turningIntegral;
  absolute_time_t lastTime; // Used to calculate turningIntegral

  volatile uint8_t readingbuff[19]; // Buffer for IMU readings
  volatile int chars_rxed = 0;

  volatile bool distance = false; // Variables for moving by encoder counts
  volatile int targetDis = 0;

  Motor *leftMotor;
  Motor *rightMotor;

  volatile char buffer[600]; // Buffer for LIDAR readings
  volatile int bufferIndex = 0;
  int16_t lidarPoints[320]; // Stores LIDAR data for processing
  volatile int lidarPointIndex;
  float frontWallDist = 10000; // Distance wall is from mouse

  float xcor[320]; // Arrays for processed LIDAR data
  float ycor[320];

  int positiveMod(int a,
                  int b); // Returns a number [0,b) that's equivalent to a mod b
  void cartesianConvert(); // Turning LIDAR values to x, y coordinates
  float checkFrontWallDistance();

public:
  Drivetrain(float forwardRPM, float KP, float KI, float maxTurning,
             Motor *motorOne, Motor *motorTwo);
  int16_t getYaw() { return yaw; }
  int16_t getTargetYaw() { return targetYaw; }

  // Movement
  void forward();
  void stop();
  void forwardDistance(int dis);
  void turn(int deg);

  // Timer interrupt callbacks
  void turningLoop();
  void driveLoop();
  void controlLoop();

  // UART interrupt callbacks
  void setYaw();
  void updateYaw();

  void initLidar();
  void uart_lidar_irq();

  // Higher level functions to be called in API
  void driveForwardsOneSquare() { forwardDistance(SQUAREENCODERCOUNTS); }
  void turnLeft() { turn(-9000); }
  void turnRight() { turn(9000); }
  bool checkFrontWall();
  bool checkRightWall();
  bool checkLeftWall();
};