// motor.cpp
#include <wiringPi.h>
#include <unistd.h>  // For usleep
#include "../include/motor.h"

Motor::Motor(int stepPin, int dirPin, int enaPin) {
    STEP_PIN = stepPin;
    DIR_PIN = dirPin;
    ENA_PIN = enaPin;
}

void Motor::Activate() {
    pinMode(STEP_PIN, OUTPUT);
    pinMode(DIR_PIN, OUTPUT);
    if (ENA_PIN != -1) {
        pinMode(ENA_PIN, OUTPUT);
        digitalWrite(ENA_PIN, LOW);
    }
}

void Motor::Deactivate() {
    if (ENA_PIN != -1) {
        digitalWrite(ENA_PIN, HIGH); 
    }
}

void Motor::forward(int steps, int delayMs) {
    digitalWrite(DIR_PIN, HIGH);
    for (int i = 0; i < steps; ++i) {
        digitalWrite(STEP_PIN, HIGH);
        delayMicroseconds(delayMs);
        digitalWrite(STEP_PIN, LOW);
        delayMicroseconds(delayMs);
    }
}

void Motor::backward(int steps, int delayMs) {
    digitalWrite(DIR_PIN, LOW);
    for (int i = 0; i < steps; ++i) {
        digitalWrite(STEP_PIN, HIGH);
        delayMicroseconds(delayMs);
        digitalWrite(STEP_PIN, LOW);
        delayMicroseconds(delayMs);
    }
}
