/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc/XboxController.h>

#ifndef VALORGAMEPAD_H
#define VALORGAMEPAD_H

class ValorGamepad : public frc::XboxController
{
public:

    ValorGamepad(int);

    void setDeadbandX(double);
    void setDeadbandY(double);

    double leftStickX(int polynomial=1);
    bool leftStickXActive(int polynomial=1);
    double leftStickY(int polynomial=1);
    bool leftStickYActive(int polynomial=1);

    double rightStickX(int polynomial=1);
    bool rightStickXActive(int polynomial=1);
    double rightStickY(int polynomial=1);
    bool rightStickYActive(int polynomial=1);

    double rightTrigger();
    bool rightTriggerActive();
    double leftTrigger();
    bool leftTriggerActive();

    bool DPadUp();
    bool DPadDown();
    bool DPadLeft();
    bool DPadRight();

private:
    double deadband(double, double, int);

    double deadbandX;
    double deadbandY;
};

#endif