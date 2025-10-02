#ifndef MOTOR_H
#define MOTOR_H


// Libraries used for the step motor 
#include "wiringPi.h" // USING FOR PINMODE() and digitalWrite()
#include <unistd.h> // Need for sleep functions 
#include <iostream>

const int NUM_X = 8;
const int NUM_Y = 4;
class Motor{

    private:
        int steps[NUM_X][NUM_Y] = {
        {1, 0, 0, 0},
        {1, 1, 0, 0},
        {0,  1, 0, 0},
        {0, 1, 1, 0},
        {0, 0, 1, 0},
        {0, 0, 1, 1},
        {0, 0, 0, 1},
        {1, 0, 0, 1}
    };
        

    public:
  
        Motor();

        int Pin1, Pin2, Pin3, Pin4;
        // FUNCTIONS
        void stepMotor(const int &step);
        void forward(const int &stepCount, const int &delayMs = 2);
        void backward(const int &stepCount, const int &delayMs = 2);
        void Activate();
        void Deactivate();
};

#endif // MOTOR_H