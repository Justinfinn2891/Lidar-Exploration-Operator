#ifndef MOTOR_H
#define MOTOR_H


// Libraries used for the step motor 
#include <gpiod.h>
#include <iostream>
#include <thread> 
#include <chrono> 

#define CHIP_NAME "gpiochip0" // GPIO controller on the PI
/* DIR tells the driver which way to rotate.*/
#define DIR_LINE 20 // GPIO 20

/* Each pulse here makes the motor move one step*/
#define STEP_LINE 21 // GPIO21

class Motor{

    private:
    gpiod_chip* chip;
    gpiod_line* dir;
    gpiod_line* step;
    int stepsPerRev;

public:
    Motor(const char* chipName, int dirPin, int stepPin, int stepsPerRev = 200);


    ~Motor();

    void setDirection(bool forward);

    void stepOnce(int delay_us);

    void rotateSteps(int steps, int delay_us);

    void rotateDegrees(float degrees, int delay_us);
};



#endif // MOTOR_H
