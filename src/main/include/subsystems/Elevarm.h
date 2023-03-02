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
#include "subsystems/Intake.h"

#include <frc/smartdashboard/SendableChooser.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/trajectory/Trajectory.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>

#include <frc2/command/FunctionalCommand.h>
#include <unordered_map>

#include <ctre/phoenix/sensors/WPI_CANCoder.h>

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
     Elevarm(frc::TimedRobot *robot , Intake *_intake);

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

     void InitSendable(wpi::SendableBuilder& builder);

    struct Positions {
        Positions() {
            Positions(0,0,0);
        }
        Positions(double _h, double _theta, double _wrist) {
            h = _h;
            theta = _theta;
            wrist = _wrist;
        }
        double h;
        double theta;
        double wrist;
    };

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
        ELEVARM_GROUND_SCORE,
        ELEVARM_PLAYER,
        ELEVARM_MID,
        ELEVARM_HIGH,
        ELEVARM_MANUAL,
        ELEVARM_SNAKE
    };

    enum ElevarmSolutions {
        ELEVARM_ARMS,
        ELEVARM_LEGS
    };

    struct x
    {
        ElevarmPieceState pieceState;
        ElevarmDirectionState directionState;
        ElevarmPositionState positionState;

        double manualCarriage;
        double manualArm;

        Positions targetPose;
        frc::Pose2d resultKinematics;

        bool deadManEnabled;
        bool pitModeEnabled;

        double frontMinAngle;
        double backMinAngle;

    } futureState, previousState;

    double heightDeadband, rotationDeadband;

    frc2::FunctionalCommand * getAutoCommand(std::string, std::string, std::string, bool);

    frc2::FunctionalCommand * getRotatePIDSetterCommand(bool);

    std::unordered_map<std::string, ElevarmPieceState> stringToPieceMap = {
        {"cone", ElevarmPieceState::ELEVARM_CONE},
        {"cube", ElevarmPieceState::ELEVARM_CUBE}
    };
    ElevarmPieceState stringToPieceState(std::string name){
        return stringToPieceMap[name];
    }

    std::unordered_map<std::string, ElevarmDirectionState> stringToDirectionMap = {
        {"front", ElevarmDirectionState::ELEVARM_FRONT},
        {"back", ElevarmDirectionState::ELEVARM_BACK}
    };
    ElevarmDirectionState stringToDirectionState(std::string name){
        return stringToDirectionMap[name];
    }

    std::unordered_map<std::string, ElevarmPositionState> stringToPositionMap = {
        {"stow", ElevarmPositionState::ELEVARM_STOW},
        {"ground", ElevarmPositionState::ELEVARM_GROUND},
        {"player", ElevarmPositionState::ELEVARM_PLAYER},
        {"mid", ElevarmPositionState::ELEVARM_MID},
        {"high", ElevarmPositionState::ELEVARM_HIGH},
        {"ground", ElevarmPositionState::ELEVARM_MANUAL},
        {"ground_score", ElevarmPositionState::ELEVARM_GROUND_SCORE},
        {"snake", ElevarmPositionState::ELEVARM_SNAKE}
    };
    ElevarmPositionState stringToPositionState(std::string name){
        if (!stringToPositionMap.contains(name))
            return ElevarmPositionState::ELEVARM_GROUND;
        return stringToPositionMap.at(name);
    }

    void setArmPIDF(bool);

private:

    /**
     * @brief Find the minimum angle required before moving the carriage
     * 
     * Dynamically calculate the angle (in degrees) in which the arm needs to be at before moving
     * the carriage in the -Z (or downward) direction. Prevents collision of the arm with the chassis.
     * 
     * Does not consider the ground - assumption is that the arm is at enough of an angle to not get stuck
     * if the arm does hit the ground while the carriage is moving downward.
     * 
     * @return double Minimum angle in degrees
     */
    double minAngle(bool);
    bool minFloorAngle();

     ValorNeoController carriageMotors;
     ValorFalconController armRotateMotor;

    ctre::phoenix::sensors::WPI_CANCoder armCANcoder;

     ValorFalconController wristMotor;

     std::map<ElevarmPieceState, std::map<ElevarmDirectionState, std::map<ElevarmPositionState, frc::Pose2d>>> posMap;
     frc::Pose2d stowPos;

    Positions reverseKinematics(frc::Pose2d pose, ElevarmSolutions, ElevarmDirectionState); 
    frc::Pose2d forwardKinematics(Positions positions);
    Positions detectionBoxManual(double, double);

    Intake *intake;

    ValorPIDF carriagePID;
    ValorPIDF rotatePID;
    ValorPIDF autoRotatePID;
    ValorPIDF wristPID;
     
     double manualMaxCarriageSpeed;
     double manualMaxArmSpeed;
     double carriageStallPower;
};