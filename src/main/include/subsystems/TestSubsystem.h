/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include "ValorSubsystem.h"
#include "Constants.h"
#include "ValorGamepad.h"

#include "sensors/ValorCurrentSensor.h"
#include "sensors/ValorDebounceSensor.h"

#include "controllers/ValorFalconController.h"
#include "controllers/ValorNeoController.h"

class TestSubsystem : public ValorSubsystem
{
public:
    TestSubsystem(frc::TimedRobot *_robot);

    void init();
    void setControllers(ValorGamepad *controllerO, ValorGamepad *controllerD);

    void assessInputs();
    void analyzeDashboard();
    void assignOutputs();

    void resetState();

    enum TestSubsystemState {
        DISABLED,
        TOPOSITION,
        TOSPEED,
        TOPOWER
    };
    
    struct x
    {
        TestSubsystemState testSubsystemState;

        int testPositionTarget;
        int testSpeedTarget;
        int testPowerTarget;

    } state;

private:
    ValorGamepad *driverController;
    ValorGamepad *operatorController;

    ValorFalconController testMotorController;
};