#include "sensors/ValorCurrentSensor.h"

#define CACHE_SIZE 20
#define DEFAULT_SPIKE_VALUE 22

ValorCurrentSensor::ValorCurrentSensor(frc::TimedRobot *_robot, const char* _name) :
    ValorSensor(_robot, _name),
    spikedSetpoint(DEFAULT_SPIKE_VALUE),
    cacheSize(CACHE_SIZE)
{
    reset();

    wpi::SendableRegistry::AddLW(this, "ValorCurrentSensor", sensorName);
}

void ValorCurrentSensor::setCacheSize(double _threshold)
{
    cacheSize = _threshold;
}

void ValorCurrentSensor::setSpikeSetpoint(double _setpoint)
{
    spikedSetpoint = _setpoint;
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
    if (currState > spikedSetpoint) {
        if (spikeCallback) {
            spikeCallback();
        }
    }
}

void ValorCurrentSensor::InitSendable(wpi::SendableBuilder& builder)
{
    builder.SetSmartDashboardType("Susbsystem");
    builder.AddDoubleProperty(
        "Current State", 
        [this] { return currState; },
        nullptr);
    builder.AddDoubleProperty(
        "Previous State",
        [this] { return prevState; },
        nullptr);
    builder.AddDoubleProperty(
        "Spiked Setpoint",
        [this] { return spikedSetpoint; },
        nullptr/*[this](double spikedSetpointSet) { setSpikeSetpoint(spikedSetpointSet); }*/);
    builder.AddDoubleProperty(
        "Cache Size",
        [this] { return cacheSize; },
        nullptr/*[this](double cacheSizeSet) { cacheSize = cacheSizeSet; }*/);
    builder.AddBooleanProperty(
        "Spiked",
        [this] { return (currState > spikedSetpoint); },
        nullptr);
}