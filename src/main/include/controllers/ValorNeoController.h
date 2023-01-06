/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include "controllers/ValorController.h"

#include <iostream>

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

    void setEncoderPosition(double position, int slot);

    void setPosition(double);
    void setSpeed(double);
    void setPower(double);
    
    void setupFollower(int, bool = false);
    
    void setPIDF(ValorPIDF pidf, int slot);
    void setForwardLimit(double forward);
    void setReverseLimit(double reverse);
    void setRange(int slot, double min, double max);

    void setConversion(double);

    void setProfile(int slot);
    
private:
    rev::SparkMaxPIDController pidController;
    rev::SparkMaxRelativeEncoder encoder;

    rev::CANSparkMax::IdleMode mode;
    bool inverted;

    int currentPidSlot;
};