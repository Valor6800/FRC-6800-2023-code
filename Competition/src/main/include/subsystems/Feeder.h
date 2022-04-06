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
#include <ctre/Phoenix.h>
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
        FEEDER_CURRENT_INTAKE,
        FEEDER_REGULAR_INTAKE
    };
    
    struct x
    {
        bool driver_rightBumperPressed;

        bool operator_bButtonPressed;
        bool operator_aButtonPressed;

        bool driver_leftBumperPressed;
        bool operator_leftBumperPressed;

        bool driver_rightTriggerPressed;
        bool driver_leftTriggerPressed;

        bool bannerTripped;
        bool previousBanner;
        bool currentBanner;

        bool reversed;

        bool spiked;
        
        double intakeForwardSpeed;
        double intakeReverseSpeed;
        double spikeCurrent;

        double feederForwardSpeedDefault;
        double feederForwardSpeedShoot;
        double feederReverseSpeed;
        
        //int current_cache_index;
        //std::vector<double> current_cache;
        std::deque<double> current_cache;

        double instCurrent;

        FeederState feederState;
    } state;

void resetDeque();

private:
    frc::XboxController *driverController;
    frc::XboxController *operatorController;

    WPI_TalonFX motor_intake;
    WPI_TalonFX motor_stage;

    frc::DigitalInput banner;

    void calcCurrent();
    
    

};

#endif