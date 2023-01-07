#include "sensors/ValorCurrentSensor.h"

#define CACHE_SIZE 20
#define DEFAULT_SPIKE_VALUE 22

ValorCurrentSensor::ValorCurrentSensor(frc::TimedRobot *_robot) :
    ValorSensor(_robot),
    spikedSetpoint(DEFAULT_SPIKE_VALUE)
{
    reset();
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
    for (int i = 0; i < CACHE_SIZE; i++) {
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
    for (int i = 0; i < CACHE_SIZE; i++) {
        sum += cache.at(i);
    }

    currState = sum / CACHE_SIZE;
    if (currState > spikedSetpoint)
        spikeCallback();
}