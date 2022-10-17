#include "sensors/ValorDebounceSensor.h"

ValorDebounceSensor::ValorDebounceSensor()
{
}

void ValorDebounceSensor::reset()
{
    prevState = false;
    currState = false;
    isSpiked = false;
}

bool ValorDebounceSensor::spiked()
{
    return isSpiked;
}

void ValorDebounceSensor::calculate()
{
    currState = getSensor();
    isSpiked = currState != prevState;
    prevState = currState;
}
