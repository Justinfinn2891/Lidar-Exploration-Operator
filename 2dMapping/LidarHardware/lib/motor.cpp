#include "../include/motor.h"

// USE LIBRARY TO CONTROL MOTOR
// CREATE FUNCTIONS HERE, label in .h and then implemnent in lidar.cpp


    Motor::Motor(){
        Pin1 = 0; // GPIO17
        Pin2 = 1; // 18
        Pin3 = 2; // 27
        Pin4 = 3; // 22

    // rows represent a step and the 4 numbers represent which coil is energized for ON (1) and OFF (0)

}

// This function will be used for HIGH or LOW setting for the motor pins 
void Motor::stepMotor(const int &current_step){

    digitalWrite(Pin1, steps[current_step][0]);
    digitalWrite(Pin2, steps[current_step][1]);
    digitalWrite(Pin3, steps[current_step][2]);
    digitalWrite(Pin4, steps[current_step][3]);

}

void Motor::forward(const int &stepCount, const int &delayMs){
    for(int i = 0; i < stepCount; i++){
        stepMotor(i % 8);
        delay(delayMs);
    }
}

void Motor::backward(const int &stepCount, const int &delayMs){
    for(int i = 0; i < stepCount; i++){
        stepMotor(7 - (i % 8));
        delay(delayMs); // small delay in between steps 
    }
}

void Motor::Activate(){

    pinMode(motor.Pin1, OUTPUT); // Assign output to control them with high / low signals 
    pinMode(motor.Pin2, OUTPUT);
    pinMode(motor.Pin3, OUTPUT);
    pinMode(motor.Pin4, OUTPUT);

}

void Motor:: Deactivate(){

    digitalWrite(motor.Pin1, 0);
    digitalWrite(motor.Pin2, 0);
    digitalWrite(motor.Pin3, 0);
    digitalWrite(motor.Pin4, 0);
}
