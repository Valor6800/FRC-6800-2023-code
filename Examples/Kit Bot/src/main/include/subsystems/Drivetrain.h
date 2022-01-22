/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include "ValorSubsystem.h"
#include "Constants.h"
#include <vector>

#include <frc/XboxController.h>
#include "rev/CANSparkMax.h"
#include <frc/drive/DifferentialDrive.h>


#ifndef DRIVETRAIN_H
#define DRIVETRAIN_H

class Drivetrain : public ValorSubsystem
{
public:
     Drivetrain();
     ~Drivetrain();

     void init();
     void setController(frc::XboxController *controller);

     void assessInputs();
     void analyzeDashboard();
     void assignOutputs();

     void resetState();

     struct x
     {
          double leftStickX;
          double leftStickY;
          double rightStickX;
          double rightStickY;

          bool backButtonPressed;
          bool startButtonPressed;

          bool bButtonPressed;
          bool aButtonPressed;
          bool xButtonPressed;
          bool yButtonPressed;

          bool dPadUpPressed;
          bool dPadDownPressed;
     } state;

     frc::XboxController *driverController;

     rev::CANSparkMax m_leftBackMotor;
     rev::CANSparkMax m_leftFrontMotor;
     rev::CANSparkMax m_rightBackMotor;
     rev::CANSparkMax m_rightFrontMotor;

     frc::DifferentialDrive m_drive{m_leftFrontMotor, m_rightFrontMotor};

     std::shared_ptr<nt::NetworkTable> limeTable;
};

#endif