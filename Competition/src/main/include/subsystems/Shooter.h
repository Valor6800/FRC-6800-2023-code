/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include "ValorSubsystem.h"
#include "Constants.h"
#include "ValorSwerve.h"
#include "Drivetrain.h"
#include "ValorGamepad.h"
#include "controllers/ValorFalconController.h"
#include "controllers/ValorNeoController.h"

#include <vector>

#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/shuffleboard/ShuffleboardLayout.h>
#include <frc/shuffleboard/ShuffleboardTab.h>
#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableInstance.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include "networktables/NetworkTable.h"
#include <frc/livewindow/LiveWindow.h>

#ifndef SHOOTER_H
#define SHOOTER_H

class Shooter : public ValorSubsystem
{
public:
    Shooter();

    void init();
    void setControllers(ValorGamepad *controllerO, ValorGamepad *controllerD);
    void setDrivetrain(Drivetrain * dt);

    void assessInputs();
    void analyzeDashboard();
    void assignOutputs();

    void resetState();

     double getTargetTics(double, double, double, double, double, double, double);
     double convertTargetTics(double, double);

     void setLimelight(int pipeline);

     void assignTurret(double tg);

    enum TurretState{
         TURRET_DISABLE, // Not moving
         TURRET_MANUAL, // Manual control from operator
         TURRET_HOME_MID, // In process of moving to home
         TURRET_HOME_LEFT,
         TURRET_HOME_RIGHT,
         TURRET_TRACK // Tracking via limelight
    };

    enum HoodState{
         HOOD_DOWN, // Down position
         HOOD_TRACK, // Tracking using limelight
         HOOD_POOP //Low goal shot
     };

     enum FlywheelState{
          FLYWHEEL_DISABLE, // Not moving
          FLYWHEEL_DEFAULT, // Low speed
          FLYWHEEL_TRACK, // Dynamic calculations
          FLYWHEEL_POOP //Low goal shot
     };

    struct x
    {
          FlywheelState flywheelState;
          HoodState hoodState;
          TurretState turretState;
          TurretState lastTurretState;

          bool driverLastLeftTrigger;

          double limelightDistance;

          double turretOutput; //%
          double turretTarget; //pos
          double turretDesired;

          double flywheelTarget; //vel
          double hoodTarget; //pos

          double flywheelLow; // Low setpoint
          double flywheelHigh; // High setpoint

          double hoodLow; // Low position
          double hoodHigh; // High position

          bool trackCorner;

          double distanceToHub;

          int currentBall;

          int pipeline;
          int LoBFZoom;

          double powerC_1x;
          double hoodC_1x;
          double powerC_2x;
          double hoodC_2x;

          double tv;
          double tx;

    } state;

     void limelightTrack(bool track);

     ValorFalconController shooterController;
     ValorNeoController turretController;
     ValorNeoController hoodController;

     ValorGamepad *operatorController;
     ValorGamepad *driverController;

     std::shared_ptr<nt::NetworkTable> limeTable;
     std::shared_ptr<nt::NetworkTable> liftTable;

     Drivetrain *odom;

     frc::SendableChooser<int> m_chooserLimelight;
     frc::SendableChooser<int> m_chooserPID;
};

#endif