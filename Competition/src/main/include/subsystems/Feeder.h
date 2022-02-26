/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include "ValorSubsystem.h"
#include "Constants.h"
#include <deque>

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
        FEEDER_SHOOT,
        FEEDER_INTAKE,
        FEEDER_AUTO
    };
    
    struct x
    {
        bool driver_rightBumperPressed;

        bool operator_bButtonPressed;
        bool operator_aButtonPressed;

        bool driver_leftBumperPressed;
        bool operator_leftBumperPressed;

        bool bannerTripped;
        bool previousBanner;
        bool currentBanner;

        bool reversed;

        bool spiked;
        
        double intakeForwardSpeed;
        double intakeReverseSpeed;

        double feederForwardSpeedDefault;
        double feederForwardSpeedShoot;
        double feederReverseSpeed;
        
        //int current_cache_index;
        //std::vector<double> current_cache;
        std::deque<double> current_cache;

        double instCurrent;

        FeederState feederState;
    } state;



private:
    frc::XboxController *driverController;
    frc::XboxController *operatorController;

    rev::CANSparkMax motor_intake;
    rev::CANSparkMax motor_stage;

    frc::DigitalInput banner;

    void calcCurrent();
    
    void resetDeque();
};

#endif