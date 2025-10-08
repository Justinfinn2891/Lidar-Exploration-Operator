#include "../include/motor.h"


Motor::Motor(const char* chipName, int dirPin, int stepPin, int stepsPerRev)
    : chip(nullptr), dir(nullptr), step(nullptr), stepsPerRev(stepsPerRev)
{
    chip = gpiod_chip_open_by_name(chipName);
    if (!chip) throw std::runtime_error("Failed to open GPIO chip");

    dir  = gpiod_chip_get_line(chip, dirPin);
    step = gpiod_chip_get_line(chip, stepPin);

    if (!dir || !step)
        throw std::runtime_error("Failed to get GPIO lines");

    if (gpiod_line_request_output(dir, "motor", 0) < 0 ||
        gpiod_line_request_output(step, "motor", 0) < 0)
        throw std::runtime_error("Failed to request lines as outputs");
}

Motor::~Motor() {
    if (step) gpiod_line_release(step);
    if (dir) gpiod_line_release(dir);
    if (chip) gpiod_chip_close(chip);
}

void Motor::setDirection(bool forward) {
    gpiod_line_set_value(dir, forward ? 1 : 0);
}

void Motor::stepOnce(int delay_us) {
    gpiod_line_set_value(step, 1);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us));
    gpiod_line_set_value(step, 0);
    std::this_thread::sleep_for(std::chrono::microseconds(delay_us));
}

void Motor::rotateSteps(int steps, int delay_us) {
    for (int i = 0; i < steps; ++i)
        stepOnce(delay_us);
}

void Motor::rotateDegrees(float degrees, int delay_us) {
    int steps = static_cast<int>((degrees / 360.0f) * stepsPerRev);
    rotateSteps(steps, delay_us);
}

