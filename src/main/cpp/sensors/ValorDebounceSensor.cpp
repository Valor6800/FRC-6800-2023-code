#include "sensors/ValorDebounceSensor.h"

ValorDebounceSensor::ValorDebounceSensor(frc::TimedRobot *_robot, const char* _name) : ValorSensor(_robot, _name)
{
    reset();

    wpi::SendableRegistry::AddLW(this, "ValorDebounceSensor", sensorName);
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
    if (currState != prevState && edge)
        edge();
    if (currState && !prevState && risingEdge)
        risingEdge();
    if (!currState && prevState && fallingEdge)
        fallingEdge();
}


void ValorDebounceSensor::InitSendable(wpi::SendableBuilder& builder)
{
    builder.SetSmartDashboardType("Susbsystem");
    builder.AddBooleanProperty(
        "currentState", [this] { return currState; }, nullptr);
    builder.AddBooleanProperty(
        "previousState", [this] { return prevState; }, nullptr);
}
