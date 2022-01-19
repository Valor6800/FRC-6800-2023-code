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
    Shooter(Drivetrain * dt);

    void init();
    void setController(frc::XboxController *controller);

    void setDefaultState();
    void assessInputs();
    void analyzeDashboard();
    void assignOutputs();

    void resetState();
    void resetEncoder();

    double getTargetTics(double, double, double, double, double, double, double);
    double convertTargetTics(double, double, double);

    enum TurretState{
         DISABLED_TURRET,
         MANUAL_TURRET,
         HOME_TURRET,
         DEFAULT_TURRET,
         PRIMED_TURRET
    };

    enum HoodState{
         DISABLED_HOOD,
         PRIMED_HOOD
     };

     enum FlywheelState{
          DISABLED_FLYWHEEL,
          DEFAULT_FLYWHEEL,
          PRIMED_FLYWHEEL
     };

    struct x
    {
          bool shooterState;
          FlywheelState flywheelState;
          HoodState hoodState;
          TurretState turretState;


          double leftStickX;
     
          bool backButton;
          bool startButton;



          double error;
          double manualPow;
          double flywheelOffesetPow;
          double limelightDistance;

          double turretVelocityTarget;
          double turretPositionSetpoint;
          int flywheelTarget;
          double hoodTarget;


    } state;



private:
     void limelightTrack(bool track);

     rev::CANSparkMax flywheel_lead;
     rev::CANSparkMax flywheel_follow;

     rev::CANSparkMax turret;
     rev::SparkMaxRelativeEncoder turretEncoder = turret.GetEncoder();
     rev::SparkMaxPIDController turretPidController = turret.GetPIDController();


     rev::CANSparkMax hood;
     rev::SparkMaxRelativeEncoder hoodEncoder = hood.GetEncoder();

     rev::SparkMaxPIDController flywheelPidController = flywheel_lead.GetPIDController();
     rev::SparkMaxRelativeEncoder flywheelEncoder = flywheel_lead.GetEncoder();

     frc::XboxController *operatorController;
     std::shared_ptr<nt::NetworkTable> limeTable;

     Drivetrain *odom;
};

#endif