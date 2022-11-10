/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <wpi/sendable/Sendable.h>
#include <wpi/sendable/SendableBuilder.h>
#include <wpi/sendable/SendableHelper.h>

/**
 * @brief Container to hold PID and feed forward values for the motor controller
 */
class ValorPIDF : public wpi::Sendable, public wpi::SendableHelper<ValorPIDF>
{
public:
    ValorPIDF();/*(int _motorId, int _slot) : motorId(_motorId), slot(_slot) {}*/

    void InitSendable(wpi::SendableBuilder& builder) override;
    /// Proportion control of the feedback term
    double P = 0.1;
    /// Integral control of the feedback term
    double I = 0.0;
    /// Derivative control of the feedback term
    double D = 0.0;
    /// Feedforward term
    double F = 0.05;
    /// Max velocity: revolutions per 100ms
    double velocity = 15000;
    /// Max acceleration: revolutions per 100ms^2
    double acceleration = 150000;
    /// Minimum error threshold
    double error = 0.5; 
    
    //protected:
        // int motorId;

        // int slot;
};