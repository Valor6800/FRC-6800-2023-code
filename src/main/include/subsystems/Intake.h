/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include "ValorSubsystem.h"
#include "Constants.h"
#include "controllers/ValorFalconController.h"
#include "controllers/ValorNeoController.h"
#include "sensors/ValorCurrentSensor.h"

#include <frc/smartdashboard/SendableChooser.h>
#include <frc/smartdashboard/SmartDashboard.h>


/**
 * @brief Subsystem - Intake
 */
class Intake : public ValorSubsystem
{
public:
     /**
      * @brief Construct a new Intake object
      * 
      * @param robot Top level robot object to parse out smart dashboard and table information
      */
     Intake(frc::TimedRobot *robot);

     /**
      * @brief Destroy the Intake object
      * 
      * Intake objects have member objects on the heap - need a destructor to take care of memory on destruction
      */
     ~Intake();

     /**
      * @brief Initialize the Intake
      * 
      */
     void init();

     void assessInputs();
     void analyzeDashboard();
     void assignOutputs();

     void resetState();

     void InitSendable(wpi::SendableBuilder& builder);

     //this state will come from elevarm, elevarm not currently connected to dev
     enum IntakePieceState {
        ELEVARM_CONE,
        ELEVARM_CUBE
     };

     enum IntakeStates {
        DISABLED,
        SPIKED,
        OUTTAKE_CONE,
        OUTTAKE_CUBE,
        OUTTAKE,
        INTAKE
     };

     struct x
     {
          IntakePieceState pieceState;
          IntakeStates intakeState;

          double intakeSpeed;
          double outtakeSpeed;
          double outtakeConeSpeed;
          double outtakeCubeSpeed;
          double holdSpeed;

          double stallCurrent;

     }state;
    
private:

     ValorNeoController intakeMotor;
     ValorCurrentSensor currySensor;
};