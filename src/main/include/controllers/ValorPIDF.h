/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

/**
 * @brief Container to hold PID and feed forward values for the motor controller
 */
struct ValorPIDF
{
    /// Proportion control of the feedback term
    double P = 0.1;
    /// Integral control of the feedback term
    double I = 0.0;
    /// Derivative control of the feedback term
    double D = 0.0;
    /// Feedforward term
    double F = 0.000244;
    /// Max velocity: revolutions per 1s
    double velocity = 1500;
    /// Max acceleration: revolutions per 1ms^2
    double acceleration = 15000;
    /// Minimum error threshold
    double error = 0.5; 
};