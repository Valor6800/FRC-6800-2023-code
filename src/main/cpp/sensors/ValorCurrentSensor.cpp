#include "sensors/ValorCurrentSensor.h"

#define DEFAULT_CACHE_SIZE 20
#define DEFAULT_SPIKE_VALUE 22

ValorCurrentSensor::ValorCurrentSensor(frc::TimedRobot *_robot, const char *_name) :
    ValorSensor(_robot, _name),
    spikedSetpoint(DEFAULT_SPIKE_VALUE),
    cacheSize(DEFAULT_CACHE_SIZE)
{
    wpi::SendableRegistry::AddLW(this, "ValorCurrentSensor", sensorName);
    reset();
}

void ValorCurrentSensor::setSpikeSetpoint(double _setpoint)
{
    spikedSetpoint = _setpoint;
}

void ValorCurrentSensor::setCacheSize(int _size)
{
    cacheSize = _size;
    reset();
}

void ValorCurrentSensor::setSpikeCallback(std::function<void()> _lambda)
{
    spikeCallback = _lambda;
}

void ValorCurrentSensor::reset() {
    cache.clear();
    for (int i = 0; i < cacheSize; i++) {
        cache.push_back(0);
    }
    prevState = 0;
    currState = 0;
}

void ValorCurrentSensor::calculate() {
    prevState = currState;
    cache.pop_front();
    cache.push_back(getSensor());

    // Calculate average current over the cache size, or circular buffer window
    double sum = 0;
    for (int i = 0; i < cacheSize; i++) {
        sum += cache.at(i);
    }

    currState = sum / cacheSize;
    if (currState > spikedSetpoint)
        spikeCallback();
}

void ValorCurrentSensor::InitSendable(wpi::SendableBuilder& builder)
{
    builder.SetSmartDashboardType("Subsystem");
    builder.AddDoubleProperty(
        "Previous State", 
        [this] { return prevState; },
        nullptr);
    builder.AddDoubleProperty(
        "Current State", 
        [this] { return currState; },
        nullptr);
    builder.AddDoubleProperty(
        "Spiked Setpoint", 
        [this] { return spikedSetpoint; },
        nullptr);
    builder.AddDoubleProperty(
        "Cache Size", 
        [this] { return cacheSize; },
        nullptr);
    builder.AddDoubleProperty(
        "Raw Sensor Value", 
        [this] { return getSensor(); },
        nullptr);
}