#include "sensors/ValorDebounceSensor.h"

ValorDebounceSensor::ValorDebounceSensor(frc::TimedRobot *_robot) : ValorSensor(_robot)
{
    reset();
}

void ValorDebounceSensor::reset()
{
    prevState = false;
    currState = false;
}

void ValorDebounceSensor::setEdgeCallback(std::function<void()> _lambda)
{
    edge = _lambda;
}

void ValorDebounceSensor::setRisingEdgeCallback(std::function<void()> _lambda)
{
    risingEdge = _lambda;
}

void ValorDebounceSensor::setFallingEdgeCallback(std::function<void()> _lambda)
{
    fallingEdge = _lambda;
}

void ValorDebounceSensor::calculate()
{
    prevState = currState;
    currState = getSensor();
    if (currState != prevState)
        edge();
    if (currState && !prevState)
        risingEdge();
    if (!currState && prevState)
        fallingEdge();
}
