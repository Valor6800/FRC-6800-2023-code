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
#include <rev/SparkMaxAbsoluteEncoder.h>
#include <string>

class ValorNeoController : public ValorController<rev::CANSparkMax>
{
public:
    ValorNeoController(int, ValorNeutralMode, bool, std::string canbus = "");

    void init();
    void reset();

    double getCurrent();
    double getPosition();
    double getSpeed();

    int getProfile();

    void setEncoderPosition(double position);

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
    void setNeutralMode(ValorNeutralMode nmode);

    double getAbsEncoderPosition() override;
    
    void InitSendable(wpi::SendableBuilder& builder) override;
    
private:
    rev::SparkMaxPIDController pidController;
    rev::SparkMaxRelativeEncoder encoder;
    rev::SparkMaxAbsoluteEncoder extEncoder;

    int currentPidSlot;
};