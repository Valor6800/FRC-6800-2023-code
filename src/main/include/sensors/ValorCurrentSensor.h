/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include "sensors/ValorSensor.h"
#include <frc/TimedRobot.h>
#include <deque>

/**
 * @brief Sensor - identifying changes in current from a motor
 * 
 * This sensor is centered around detecting changes in output current of a motor.
 * It uses a circular buffer to maintain a rolling-average of previous current values.
 * When the average current in the window spikes above a desired amount, let the subsystem
 * know via the @link spiked @endlink function.
 * 
 * Let's say we want to monitor the last 100ms worth of current data from an intake motor.
 * When the average current over that last 100ms rises above a certain value (which we
 * specify with @link setSpikeSetpoint @endlink), then we should stop the intake to prevent
 * the intake from burning out, or let the LEDs know that we have a ball in the robot.
 * 
 * <b>What is a circular buffer?</b> A circular buffer is simply an array to store data.
 * We individually push data into the array and when the array fills up, we start to overwrite
 * data from the beginning. This allows us to keep track of the last X amount of data and see
 * the history of the array.
 */
class ValorCurrentSensor : public ValorSensor<double>
{
public:

    /**
     * @brief Construct a new Valor Current Sensor object
     * 
     * @param _robot Pass in the Robot reference so the calculate method can be auto-scheduled
     */
    ValorCurrentSensor(frc::TimedRobot *_robot, const char *_name);

    void reset();

    /**
     * @brief Setup a lambda function to identify when the current has risen above a certain threshold
     * 
     * @param _lambda Function to run when a current spike has been detected
     */
    void setSpikeCallback(std::function<void()> _lambda);

    /**
     * @brief Set the threshold to identify when current is spiked or not
     * 
     * @param threshold The average current value that will trigger the spiked function
     */
    void setSpikeSetpoint(double _setpoint);

     /**
     * @brief Set the cache size to be averaged for determining spiked value
     * 
     * @param size The size of the cache that stores current values
     */
    void setCacheSize(int _size);

    void InitSendable(wpi::SendableBuilder& builder) override;

private:
    std::function<void()> spikeCallback;

    void calculate();

    std::deque<double> cache;

    double spikedSetpoint;
    int cacheSize;
};