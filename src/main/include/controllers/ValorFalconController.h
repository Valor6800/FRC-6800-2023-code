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
#include <string>

class ValorFalconController : public ValorController<WPI_TalonFX>
{
public:
    ValorFalconController(int _canID, ValorNeutralMode _mode, bool _inverted, std::string _canbus = "");

    void init();
    void reset();

    double getPosition();
    double getSpeed();
    double getCurrent();

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
    double getAbsEncoderPosition();

    /**
     * @brief Prevent the motor from traveling backwards
     * 
     * Restrict the motor from going backwards
     */
    void preventBackwards();

    void setNeutralMode(ValorNeutralMode mode);

    void InitSendable(wpi::SendableBuilder& builder);
private:
    ValorPIDF pidf;
};