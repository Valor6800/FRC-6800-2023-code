/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include "ValorSubsystem.h" 
#include "Constants.h"

#include <frc/XboxController.h>
#include <frc/PWMVictorSPX.h>
#include <rev/CANSparkMax.h>
#include <rev/CANEncoder.h>
#include <ctre/Phoenix.h>
#include <adi/ADIS16448_IMU.h>

#include <frc/SpeedControllerGroup.h>
#include <frc/livewindow/LiveWindow.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>

#include <frc/kinematics/DifferentialDriveOdometry.h>
#include <frc/geometry/Pose2d.h>
#include <units/units.h>
#include <frc/geometry/Rotation2d.h>

#ifndef DRIVETRAIN_H
#define DRIVETRAIN_H

class Drivetrain : public ValorSubsystem {
    public:
        Drivetrain();

        void init();
        void setController(frc::XboxController* controller);

        void setDefaultState();
        void assessInputs();
        void analyzeDashboard();
        void assignOutputs();
        void resetState();

        enum DriveModeState {
            SWERVE,
            ARCADE,
            ROCKET_LEAGUE
        };

        struct x {
            DriveModeState driveModeState;
        } state;
    
    private:
        frc::XboxController* driverController;
};

#endif