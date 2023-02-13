/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include "sensors/ValorSensor.h"
#include <frc/TimedRobot.h>
#include <functional>

/**
 * @brief Sensor - debouncing and identifying rising/falling edges of boolean inputs
 * 
 * Class centered around debouncing boolean inputs.
 * 
 * <b>What is a rising/falling edge?</b> When a boolean input changes, that is called
 * an "edge". When a boolean input goes from false to true, that is called a rising
 * edge. When a boolean input goes from true to false, that is called a falling edge.
 * 
 * <b>What is debouncing?</b> Debouncing is the process of filtering out edges and making
 * sure that if an edge triggers multiple times in a very quick amount of time, it only
 * counts as a single rising or falling edge.
 */
class ValorDebounceSensor : public ValorSensor<bool>
{
public:

    /**
     * @brief Construct a new Valor Debounce Sensor object
     * 
     * @param _robot Pass in the Robot reference so the calculate method can be auto-scheduled
     */
    ValorDebounceSensor(frc::TimedRobot *_robot, const char* name);
    
    void reset();

    /**
     * @brief Setup a lambda function to detect a rising or falling edge of the sensor
     * 
     * @param _lambda Function to run when an edge has been detected
     */
    void setEdgeCallback(std::function<void()> _lambda);

    /**
     * @brief Setup a lambda function to detect a rising edge of the sensor
     * 
     * @param _lambda Function to run when a rising edge has been detected
     */
    void setRisingEdgeCallback(std::function<void()> _lambda);

    /**
     * @brief Setup a lambda function to detect a falling edge of the sensor
     * 
     * @param _lambda Function to run when a falling edge has been detected
     */
    void setFallingEdgeCallback(std::function<void()> _lambda);

    void InitSendable(wpi::SendableBuilder& builder) override;

private:

    void calculate();

    std::function<void()> edge;
    std::function<void()> fallingEdge;
    std::function<void()> risingEdge;
};