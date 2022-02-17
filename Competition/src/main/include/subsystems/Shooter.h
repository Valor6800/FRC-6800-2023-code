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
     void resetHood();

    enum TurretState{
         TURRET_DISABLE,
         TURRET_MANUAL,
         TURRET_HOME,
         TURRET_DEFAULT,
         TURRET_PRIME
    };

    enum HoodState{
         HOOD_DISABLE,
         HOOD_PRIME,
         HOOD_RESET
     };

     enum FlywheelState{
          FLYWHEEL_DISABLE,
          FLYWHEEL_DEFAULT,
          FLYWHEEL_PRIME
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

          double limelightDistance;

          double turretOutput; //%
          int turretTarget; //pos

          double flywheelTarget; //vel
          int hoodTarget; //pos

          double flywheelLow;
          double flywheelHigh;

          double hoodLow;
          double hoodHigh;

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