#include "Drivetrain.h"

int Drivetrain::positiveMod(int a, int b){
    if (a > b){
        return positiveMod(a - b, b);
    } 
    else if (a < 0){
        return positiveMod(a + b, b);
    }
    else {
        return a;
    }
}

void Drivetrain::cartesianConvert(){
    /*
        Takes 320 LIDAR points and converts them to 
        x, y coordinates.
    */
    for (int i = 0; i < 80; i++){
        int r = lidarPoints[i];
        if (r == 0){
            xcor[i] = 0;
            ycor[i] = 0;
        }
        else{
            float theta = 2 * 3.1415 * (52.5 / 79 * i - 52.5)/ 360;
            xcor[i] = r * cos(theta);
            ycor[i] = r * sin(theta);
        }
    }   
    for (int i = 80; i < 160; i++){
        int r = lidarPoints[i];
        if (r == 0){
            xcor[i] = 0;
            ycor[i] = 0;
        }
        else{
            float theta = 2 * 3.1415 * (52.5 / 79 * (i - 80))/ 360;
            xcor[i] = r * cos(theta);
            ycor[i] = r * sin(theta);
        }
    }
    for (int i = 160; i < 240; i++){
        int r = lidarPoints[i];
        if (r == 0){
            xcor[i] = 0;
            ycor[i] = 0;
        } else {
            float theta = 2 * 3.1415 * (180 - 52.5 / 79 * (239 - i))/ 360;
            xcor[i] = r * cos(theta);
            ycor[i] = r * sin(theta);
        }
    }
    for (int i = 240; i < 320; i++){
        int r = lidarPoints[i];
        if (r == 0){
            xcor[i] = 0;
            ycor[i] = 0;
        } else {
            float theta = 2 * 3.1415 * (180 + 52.5 / 79 * (i - 240))/ 360;
            xcor[i] = r * cos(theta);
            ycor[i] = r * sin(theta);
        }
    }
}

float Drivetrain::checkFrontWallDistance(){
    /*
        Takes LIDAR points 150-170 (front points) and averages the 
        y distance.
    */
    int pointOne = 150; // first point corresponding to front wall (position in array)
    int pointTwo = 170; // last point corresponding to front wall
    int sum = 0;
    int zeroCount = 0;
    cartesianConvert();
    for (int i = pointOne; i < pointTwo + 1; i++){
        if (ycor[i] == 0){
            zeroCount += 1;
        } else {sum += ycor[i];}
    }
    return sum / (float(pointTwo - pointOne + 1) - zeroCount);
}


Drivetrain::Drivetrain(float forwardRPM, float KP, float KI, float maxTurning, Motor *motorOne, Motor *motorTwo){
    targetrpm = forwardRPM;
    kp = KP;
    ki = KI;
    maxrpm = maxTurning;
    lastTime = get_absolute_time();

    leftMotor = motorOne;
    rightMotor = motorTwo;
}

void Drivetrain::forward(){
    leftMotor->setRPM(targetrpm);
    rightMotor->setRPM(targetrpm);
}

void Drivetrain::stop(){
        leftMotor->stop();
        rightMotor->stop();
}

void Drivetrain::forwardDistance(int dis){
    updateYaw();
    targetDis = leftMotor->getPosition()+dis;
    distance = true;
    while (distance){
        frontWallDist = checkFrontWallDistance();
    }
}

void Drivetrain::turn(int deg){ // positive is right, negative is left
    stop(); // Moving forward commands do not stop the motors
    updateYaw();
    targetYaw = positiveMod((deg + targetYaw + 17999), 36000) - 17999; // adds the degrees to targetYaw and maps it to [18000, -17999]
    turnLoop = true;
    while (turnLoop){
    }
    turningIntegral = 0;
    stop();
}

void Drivetrain::turningLoop(){
    /*
        Called from controlLoop() after a turning variable is set.
    */
    updateYaw();
    absolute_time_t currentTime = get_absolute_time();
    
    // Determines direction of turn
    bool right = false;
    if (yaw - targetYaw > 0){right = false;} else {right = true;}
    if (abs(yaw-targetYaw) > 18000){right = !right;}

    // Determines turning RPM
    float error = abs(yaw - targetYaw);
    if (error > 18000){error = 36000 - error;}
    turningIntegral += error * (absolute_time_diff_us(lastTime, currentTime)/60000000.0);
    float turningrpm = kp * error + ki * turningIntegral;
    turningrpm = std::min(turningrpm, maxrpm);
    if (abs(error)  < ANGLEERROR){ 
        // Stops turning when target angle is reached
        turningrpm = 0;
        turnLoop = false;
    }

    // Ilan really hated these if statements, and I do too
    if (right){
        leftMotor->setRPM(turningrpm);
        rightMotor->setRPM(turningrpm * -1);
    } else {
        leftMotor->setRPM(turningrpm * -1);
        rightMotor->setRPM(turningrpm);
    }
    lastTime = currentTime;
}

void Drivetrain::driveLoop(){
    /*
        Called from controlLoop() after a variable is set.
    */
    if ((leftMotor->getPosition() > targetDis - 10) || (frontWallDist < WALLCUTOFF)){ 
        distance = false;
    } else {
        // Adjusts RPM of either motor if angle is off
        float error = abs(yaw - targetYaw);
        if (error > 18000){error = 36000 - error;}
        bool right = false;
        if (yaw - targetYaw > 0){right = false;} else {right = true;}
        if (abs(yaw-targetYaw) > 18000){right = !right;}
        if (right) {error *= -1;}
        rightMotor->setRPM(std::min(targetrpm + RIGHTADD * error, (double) (targetrpm + 100)));
        leftMotor->setRPM(std::min(targetrpm - LEFTADD * error, (double) (targetrpm + 100)));
    }
}

void Drivetrain::controlLoop(){
    /*
        Determines what the mouse does depending on what's called from the main function.
        To be placed in an interrupt callback.
    */
    if (turnLoop){
        turningLoop();
    } else if (distance){
        driveLoop();
    } else {}
}

void Drivetrain::setYaw(){
    /*
        UART interrupt callback for receiving data from the IMU.
    */
    while (uart_is_readable(UART_IMU)) { // Do we need this?
        uint8_t ch = uart_getc(UART_IMU);
        readingbuff[chars_rxed]=ch;
        chars_rxed++;
        if (chars_rxed > 18){chars_rxed = 0;}
    }
}

void Drivetrain::updateYaw(){
    /*
        Used for processing data from the IMU.
    */
    uint8_t csum_raw, csum_calc;
    gpio_xor_mask(0x04);
    if (chars_rxed == 3){}
    yaw    = (readingbuff[4]<<8)|readingbuff[3];
    csum_raw = readingbuff[18];
    csum_calc= readingbuff[2]+readingbuff[3]+readingbuff[4]+readingbuff[5]+readingbuff[6]+readingbuff[7]+readingbuff[8]+readingbuff[9]+readingbuff[10]+readingbuff[11]+readingbuff[12]+readingbuff[13]+readingbuff[14];
    if (chars_rxed!=19) {}
    if (csum_raw != csum_calc){}
    chars_rxed = 0;
}

void Drivetrain::initLidar(){
    /*
        Part 2 of LIDAR initialization. Sends a test command, verifies the response, and sends a command to start scanning.
    */
    uint8_t command[9] = {0xA5, 0xA5, 0xA5, 0xA5, 0x00, 0x60, 0x00, 0x00, 0x60};
    uart_write_blocking(UART_LIDAR, command, 9);

    bool check = true;
    // Waits for a full response
    while (buffer[8] != 0x62){}
    // Checks if response is good
    for (int i = 0; i < 8; i++){
        if (i == 4){
            if (buffer[i] != 0x01){check = false;}
        } else {
            if (buffer[i] != command[i]){check = false;}
        }
    }
    // Changes command array to send command to start scanning
    command[5] = 0x63;
    command[8] = 0x63;
    bufferIndex = 0;
    // Sends command
    uart_write_blocking(UART_LIDAR, command, 9);
    while (buffer[8] != 0x65){}
    sleep_ms(10);
    // uart_set_fifo_enabled(UART_LIDAR, true);
    bufferIndex = 0;
    // Resets buffer
    gpio_put(PICO_DEFAULT_LED_PIN, 1);
    for(int i=0; i<331;i++) buffer[i]=0;
}

void Drivetrain::uart_lidar_irq(){
    /*
        UART interrupt callback for receiving data.
    */
    while( uart_is_readable(UART_LIDAR)){
        buffer[bufferIndex] = uart_getc(UART_LIDAR);
        if (bufferIndex == 330){
            bufferIndex = 0;
            for (int i = 10; i < 330; i += 2){
                int distance = (buffer[i] | ((buffer[i + 1] << 8))) & 0x01ff;
                lidarPointIndex = (i - 8) / 2;
            if (lidarPointIndex < 81){
                lidarPointIndex=80-lidarPointIndex;
            } else {
                lidarPointIndex =-lidarPointIndex+240;
            }
            int address = buffer[4];
            if(address == 0x1) lidarPoints[159-(lidarPointIndex-1)]=distance;
            if(address == 0x2) lidarPoints[320-(lidarPointIndex)]=distance;
            }
        } else {bufferIndex += 1;}
    }
}

bool Drivetrain::checkFrontWall(){
    /*
        Counts number of points within a space in front of the mouse to determine if there's a wall.
    */
    cartesianConvert();
    int count = 0;
    for (int i = 0; i < 320; i++){
        if (ycor[i] < 70 && ycor[i] > 20){
            if (xcor[i] < 65 && xcor[i] > -65){
                count += 1;
            }
        }
    }
    return count > 45;
}

bool Drivetrain::checkRightWall(){
    cartesianConvert();
    int count = 0;
    for (int i = 0; i < 320; i++){
        if (ycor[i] < 130 && ycor[i] > -60){
            if (xcor[i] < 90 && xcor[i] > 30){
                count += 1;
            }
        }
    }
    return count > 50;
}

bool Drivetrain::checkLeftWall(){
   cartesianConvert();
    int count = 0;
    for (int i = 0; i < 320; i++){
        if (ycor[i] < 130 && ycor[i] > -60){
            if (xcor[i] < -30 && xcor[i] > -90){
                count += 1;
            }
        }
    }
    return count > 50;
}

// Test program
/*
Motor LeftMotor = Motor(19, 18, 21, 20);
Motor RightMotor = Motor(6, 7, 8, 9);
Drivetrain drivetrain = Drivetrain(200, TURNINGKP, TURNINGKI, 200, &LeftMotor, &RightMotor);

void uartInterrupt(){
    drivetrain.setYaw();
    drivetrain.updateYaw();
    drivetrain.controlLoop();
}

void lidarInterrupt(){
    drivetrain.uart_lidar_irq();
}

bool controlLoop(repeating_timer_t *timer1){
    LeftMotor.updatePWM();
    RightMotor.updatePWM();
    return true;
};

void encoderInterrupt(uint gpio, uint32_t event_mask){
    if (gpio == 21){LeftMotor.encoderInterruptTime(gpio, event_mask);}
    else {RightMotor.encoderInterruptTime(gpio, event_mask);}
};

bool clockInterrupt(repeating_timer_t *timer){
    LeftMotor.clockInterruptTime();
    RightMotor.clockInterruptTime();
    return true;
};

void initIMU(){
    //Initialize UART
    uart_init(UART_IMU, 115200);

    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);

    int __unused actual = uart_set_baudrate(UART_IMU, BAUD_RATE);

    uart_set_hw_flow(UART_IMU, false, false);
    uart_set_format(UART_IMU, DATA_BITS, STOP_BITS, PARITY);
    uart_set_fifo_enabled(UART_IMU, true);

    irq_set_enabled(UART0_IRQ, true);
    irq_set_exclusive_handler(UART0_IRQ, &uartInterrupt);

    uart_get_hw(UART_IMU)->imsc = (bool_to_bit(true) << UART_UARTIMSC_RTIM_LSB);
}

void initLidar(){
    uart_init(UART_LIDAR, 921600);
    gpio_set_function(4, GPIO_FUNC_UART);
    gpio_set_function(5, GPIO_FUNC_UART);
    uart_set_hw_flow(UART_LIDAR, false, false);
    uart_set_format(UART_LIDAR, DATA_BITS, STOP_BITS, PARITY);
    uart_set_fifo_enabled(UART_LIDAR, false);
    // Sets up interrupt for reading information from lidar
    irq_set_exclusive_handler(UART1_IRQ, &lidarInterrupt);
    irq_set_enabled(UART1_IRQ, true);
    uart_set_irq_enables(UART_LIDAR, true, false);
}

int main(int argc, char* argv[]) {
    stdio_init_all();
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, true);

    gpio_put(PICO_DEFAULT_LED_PIN, 1);
    sleep_ms(3000);
    gpio_put(PICO_DEFAULT_LED_PIN, 0); 

    LeftMotor.setPIDVariables(LEFTMOTORKP, LEFTMOTORKI);
    RightMotor.setPIDVariables(RIGHTMOTORKP, RIGHTMOTORKI);

    struct repeating_timer controlTimer;
    add_repeating_timer_us(1000, &controlLoop, NULL, &controlTimer);

    struct repeating_timer encoderTimer;
    add_repeating_timer_us(1000, &clockInterrupt, NULL, &encoderTimer);
    
    gpio_set_irq_enabled(8, 0x8, true);
    gpio_set_irq_enabled(21, 0x8, true);
    gpio_set_irq_callback(encoderInterrupt);
    irq_set_enabled(IO_IRQ_BANK0, true);

    initIMU();
    sleep_ms(500);
    initLidar();
    sleep_ms(500);
    drivetrain.initLidar();
    sleep_ms(500);

    while(true){
    }
};
*/
