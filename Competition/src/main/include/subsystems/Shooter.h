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
#include <vector>

#include <rev/CANSparkMax.h>
#include <rev/CANEncoder.h>

#include <frc/XboxController.h>

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
    void setController(frc::XboxController *controller);
    void setDrivetrain(Drivetrain * dt);

    void assessInputs();
    void analyzeDashboard();
    void assignOutputs();

    void resetState();
    void resetEncoder();



     double getTargetTics(double, double, double, double, double, double, double);
     double convertTargetTics(double, double);

    enum TurretState{
         TURRET_DISABLE, // Not moving
         TURRET_MANUAL, // Manual control from operator
         TURRET_HOME_MID, // In process of moving to home
         TURRET_HOME_LEFT,
         TURRET_HOME_RIGHT,
         TURRET_TRACK, // Tracking via limelight
         TURRET_AUTO // Using odometry to hub
    };

    enum HoodState{
         HOOD_DOWN, // Down position
         HOOD_UP, // Up position
         HOOD_TRACK, // Tracking using limelight
         HOOD_AUTO //Auto for the furthest shot
     };

     enum FlywheelState{
          FLYWHEEL_DISABLE, // Not moving
          FLYWHEEL_DEFAULT, // Low speed
          FLYWHEEL_PRIME, // Higher speed
          FLYWHEEL_TRACK, // Dynamic calculations
          FLYWHEEL_AUTO // Auto value for the furthest shot
     };

    struct x
    {
          FlywheelState flywheelState;
          HoodState hoodState;
          TurretState turretState;
          TurretState lastTurretState;

          double leftStickX;
     
          bool backButton;
          bool startButton;
          bool rightBumper;
          bool aButton;
          bool yButton;
          bool xButton;
          bool bButton;

          double limelightDistance;

          double turretOutput; //%
          double turretTarget; //pos

          double flywheelTarget; //vel
          int hoodTarget; //pos

          double flywheelLow; // Low setpoint
          double flywheelHigh; // High setpoint

          double hoodLow; // Low position
          double hoodHigh; // High position

          bool trackCorner;

          double distanceToHub;

    } state;



private:
     void limelightTrack(bool track);

     WPI_TalonFX flywheel_lead;

     rev::CANSparkMax turret;
     rev::SparkMaxRelativeEncoder turretEncoder = turret.GetEncoder();
     rev::SparkMaxPIDController turretPidController = turret.GetPIDController();

     rev::CANSparkMax hood;
     rev::SparkMaxRelativeEncoder hoodEncoder = hood.GetEncoder();
     rev::SparkMaxPIDController hoodPidController = hood.GetPIDController();

     frc::XboxController *operatorController;
     std::shared_ptr<nt::NetworkTable> limeTable;

     Drivetrain *odom;
};

#endif