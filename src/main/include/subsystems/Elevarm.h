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

#include <frc/smartdashboard/SendableChooser.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/trajectory/Trajectory.h>
#include <frc/geometry/Pose3d.h>
#include <frc/geometry/Rotation3d.h>

/**
 * @brief Subsystem - Elevarm
 */
class Elevarm : public ValorSubsystem
{
public:

    
     
     /**
      * @brief Construct a new Elevarm object
      * 
      * @param robot Top level robot object to parse out smart dashboard and table information
      */
     Elevarm(frc::TimedRobot *robot);

     /**
      * @brief Destroy the Elevarm object
      * 
      * Elevarm objects have member objects on the heap - need a destructor to take care of memory on destruction
      */
     ~Elevarm();

     /**
      * @brief Initialize the Elevarm
      * 
      * Includes:
      * * Resetting the test Bench state
      */
     void init();

     void assessInputs();
     void analyzeDashboard();
     void assignOutputs();

     void resetState();


     enum ElevarmPieceState {
        ELEVARM_CONE,
        ELEVARM_CUBE
    };

    enum ElevarmDirectionState {
        ELEVARM_FRONT,
        ELEVARM_BACK
    };

    enum ElevarmPositionState {
        ELEVARM_STOW,
        ELEVARM_GROUND,
        ELEVARM_PLAYER,
        ELEVARM_MID,
        ELEVARM_HIGH
    };

    enum ElevarmSolutions {
        ELEVARM_A,
        ELEVARM_B,
        ELEVARM_C,
        ELEVARM_D,
        
    };

     struct x
     {
          ElevarmPieceState pieceState;
          ElevarmDirectionState directionState;
          ElevarmPositionState positionState;
          
     } futureState, previousState;

       
     ElevarmSolutions solutionsEnum;

     // First in the pair returns the height in meters and second returns the rotation in degrees
     std::pair<double, double> reverseKinematics(frc::Pose3d pose, ElevarmSolutions); 


     
private:
     ValorNeoController carriageMotors;
     ValorNeoController armRotateMotor;
     std::map<ElevarmPieceState, std::map<ElevarmDirectionState, std::map<ElevarmPositionState, frc::Pose3d>>> posMap;
     frc::Pose3d stowPos;


};