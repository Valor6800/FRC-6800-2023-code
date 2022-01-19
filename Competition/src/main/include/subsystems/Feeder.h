/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include "ValorSubsystem.h"
#include "Constants.h"

#include <frc/XboxController.h>
#include <rev/CANSparkMax.h>
#include <frc/DigitalInput.h>

#ifndef FEEDER_H
#define FEEDER_H

class Feeder : public ValorSubsystem
{
public:
    Feeder();

    void init();
    void setControllers(frc::XboxController *controllerO, frc::XboxController *controllerD);

    void assessInputs();
    void analyzeDashboard();
    void assignOutputs();

    void resetState();

    enum FeederState {
        FEEDER_DISABLE,
        FEEDER_REVERSE,
        FEEDER_INTAKE1,
        FEEDER_INTAKE2
    };
    
    struct x
    {
        bool driver_bButtonPressed;
        bool driver_aButtonPressed;

        bool operator_bButtonPressed;
        bool operator_aButtonPressed;

        bool driver_leftBumperPressed;
        bool operator_leftBumperPressed;

        bool upperBannerTripped;
        bool lowerBannerTripped;

        FeederState feederState;
    } state;



private:
    frc::XboxController *driverController;
    frc::XboxController *operatorController;

    rev::CANSparkMax motor_intake;
    rev::CANSparkMax motor_stage1;
    rev::CANSparkMax motor_stage2;

    frc::DigitalInput banner_lower;
    frc::DigitalInput banner_upper;
};

#endif