/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include "controllers/ValorController.h"

#include <rev/CANSparkMax.h>
#include <rev/CANEncoder.h>
#include <string>

class ValorNeoController : public ValorController<rev::CANSparkMax>
{
public:
    ValorNeoController(int, rev::CANSparkMax::IdleMode, bool, std::string canbus = "");

    void init();
    void reset();

    double getCurrent();
    double getPosition();
    double getSpeed();
    void setPosition(double);
    void setSpeed(double);
    void setPower(double);
    
    void setupFollower(int);
    
    void setPIDF(ValorPIDF pidf, int slot);
    void setLimits(int reverse, int forward);
    void setForwardLimit(int forward);
    void setReverseLimit(int reverse);
    void setRange(int slot, double min, double max);

    void setConversion(double);

    void setProfile(int slot);
    
private:
    rev::SparkMaxPIDController pidController;
    rev::SparkMaxRelativeEncoder encoder;

    bool inverted;
    rev::CANSparkMax::IdleMode mode;

    int currentPidSlot;
};