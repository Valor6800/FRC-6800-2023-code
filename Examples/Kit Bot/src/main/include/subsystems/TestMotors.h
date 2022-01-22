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
#include <string>

#include <frc/XboxController.h>
#include "rev/CANSparkMax.h"
#include <frc/DigitalInput.h>

#ifndef TESTMOTORS_H
#define TESTMOTORS_H

class TestMotors : public ValorSubsystem
{
public:
     TestMotors();
     ~TestMotors();

     void init();
     void setControllers(frc::XboxController *controllerO, frc::XboxController *controllerD);

     void assessInputs();
     void analyzeDashboard();
     void assignOutputs();

     void resetState();

     struct x
     {
          double driver_leftStickX;
          double driver_leftStickY;     //Reserved For Drivetrain
          double driver_rightStickX;    //Reserved For Drivetrain
          double driver_rightStickY;

          bool driver_bButtonPressed;
          bool driver_aButtonPressed;
          bool driver_xButtonPressed;
          bool driver_yButtonPressed;

          bool driver_dPadUpPressed;
          bool driver_dPadDownPressed;
          bool driver_dPadLeftPressed;
          bool driver_dPadRightPressed;

          bool driver_leftTriggerPressed;
          bool driver_rightTriggerPressed;

          bool driver_startButtonPressed;
          bool driver_backButtonPressed;

          bool driver_leftBumperPressed;
          bool driver_rightBumperPressed;

          double operator_leftStickX;
          double operator_leftStickY;
          double operator_rightStickX;
          double operator_rightStickY;

          bool operator_bButtonPressed;
          bool operator_aButtonPressed;
          bool operator_xButtonPressed;
          bool operator_yButtonPressed;

          bool operator_dPadUpPressed;
          bool operator_dPadDownPressed;
          bool operator_dPadLeftPressed;
          bool operator_dPadRightPressed;

          bool operator_leftTriggerPressed;
          bool operator_rightTriggerPressed;

          bool operator_startButtonPressed;
          bool operator_backButtonPressed;

          bool operator_leftBumperPressed;
          bool operator_rightBumperPressed;

          bool photoelecReading;

          double testMotorVal[10];

          bool testMotorReversed[10];

          std::string controllers[2];
          std::string buttons[18];

          std::string motorAttachedTo[10];

          rev::CANSparkMax *motors[10];
     } state;
   
     frc::XboxController *driverController;
     frc::XboxController *operatorController;

     rev::CANSparkMax m_testMotor1;
     rev::CANSparkMax m_testMotor2;
     rev::CANSparkMax m_testMotor3;
     rev::CANSparkMax m_testMotor4;
     rev::CANSparkMax m_testMotor5;
     rev::CANSparkMax m_testMotor6;
     rev::CANSparkMax m_testMotor7;
     rev::CANSparkMax m_testMotor8;
     rev::CANSparkMax m_testMotor9;
     rev::CANSparkMax m_testMotor10;

     std::shared_ptr<nt::NetworkTable> limeTable;

     frc::DigitalInput testPhotoelec;
};

#endif