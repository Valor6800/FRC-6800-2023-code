/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include "controllers/ValorController.h"

#include <ctre/Phoenix.h>
#include <ctre/phoenix/motorcontrol/NeutralMode.h>

#ifndef VALORFALCONCONTROLLER_H
#define VALORFALCONCONTROLLER_H

class ValorFalconController : public ValorController<WPI_TalonFX, NeutralMode>
{
public:
    ValorFalconController(int, NeutralMode, bool, const std::__cxx11::string &canbus = "");

    void init();
    void reset();

    double getPosition();
    double getSpeed();
    void setPosition(double);
    void setSpeed(double);
    void setPower(double);

    void setupFollower(int);
    
    void setPIDF(int slot, PIDF pidf);
    void setLimits(int reverse, int forward);
    void setRange(int slot, double min, double max);
    
    void setConversion(double);

    void setProfile(int slot);

    void preventBackwards();
private:
    double conversion;
};

#endif