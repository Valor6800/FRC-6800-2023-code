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
#include <vector>

#include <AHRS.h>
#include <frc/XboxController.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <frc/DutyCycleEncoder.h>

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

        bool tracking;

    } state;

    /**
         * Drive the robot with given x, y and rotational velocities using open loop velocity control
         * @param vx_mps the desired x velocity component in meters per second
         * @param vy_mps the desired y velocity component in meters per second
         * @param omega_radps the desired rotational velocity component in radians per second
         * @param isFOC true if driving field oriented
         */
    void drive(units::meters_per_second_t vx_mps, units::meters_per_second_t vy_mps, units::radians_per_second_t omega_radps, bool isFOC);

    /**
         * Move the robot with given x, y and rotational velocities using closed loop velocity control
         * @param vx_mps the desired x velocity component in meters per second
         * @param vy_mps the desired y velocity component in meters per second
         * @param omega_radps the desired rotational velocity component in radians per second
         * @param isFOC true if driving field oriented
         */
    void move(double vx_mps, double vy_mps, double omega_radps, bool isFOC);

    /**
         * Directly set the swerve modules to the specified states
         * @param desiredStates the desired swerve module states
         */
    void setModuleStates(wpi::array<frc::SwerveModuleState, 4> desiredStates);

    /**
         * Reset the gyro to a heading of zero
         */
    void resetGyro();

    /**
         * Reset the drive encoders to currently read a position of 0
         */
    void resetDriveEncoders();

    /**
         * Reset the robot's position on the field. Any accumulted gyro drift will be noted and
         *   accounted for in subsequent calls to getPoseMeters()
         * @param pose The robot's actual position on the field
         */
    void resetOdometry(frc::Pose2d pose);

    /**
         * Get the configured swerve modules
         * @return vector of swerve modules
         */
    std::vector<ValorSwerve *> getSwerveModules();

    /**
         * Set the current gyro offset applied to the IMU angle during field oriented driving
         * Defaults to 0
         */
    void setGyroOffset(frc::Rotation2d offset);

    /**
         * Get the current gyro offset applied to the IMU angle during field oriented driving
         * @return offset that is applied to the gyro
         */
    frc::Rotation2d getGyroOffset();

    /**
         * Returns the rate of rotation of the gyro
         * The rate is based on the most recent reading of the gyro analog value.
         * The rate is expected to be positive as the gyro turns clockwise
         * @return rate of rotation of the gyro
         */
    double getGyroRate();

    /**
         * Returns the current gyro heading of the robot
         * This will be affected by any gyro drift that may have accumulated since last recalibration
         * The angle is continuous from 360 to 361 degrees
         * This allows algorithms that wouldn't want to see a discontinuity in the gyro as it sweeps 
         *   past from 360 to 0 on the second time around.
         * The angle is expected to increase as the gyro turns clockwise
         */
    frc::Rotation2d getHeading();

     //returns angle within the range [-180, 180]
    double angleWrap(double degrees);

    /**
         * Returns the position of the robot on the field in meters
         * @return the pose of the robot (x and y are in meters)
         */
    frc::Pose2d getPose_m();

    /**
         * Returns the kinematics object in use by the swerve drive
         * @return kinematics object
         */
    frc::SwerveDriveKinematics<4> getKinematics();

private:
    wpi::array<frc::SwerveModuleState, 4> getModuleStates(units::meters_per_second_t,
                                                          units::meters_per_second_t,
                                                          units::radians_per_second_t,
                                                          bool);

    void configSwerveModule(int);

    frc::XboxController *driverController;

    std::vector<ValorSwerve *> swerveModules;
    std::vector<WPI_TalonFX *> azimuthMotors;
    std::vector<WPI_TalonFX *> driveMotors;
        std::vector<frc::DutyCycleEncoder*> magEncoders;
    std::vector<frc::Translation2d> motorLocations{frc::Translation2d{SwerveConstants::SWERVE_MODULE_DIFF_X * DriveConstants::MODULE_DIFF_XS[0],
                                                                      SwerveConstants::SWERVE_MODULE_DIFF_Y *DriveConstants::MODULE_DIFF_YS[0]},
                                                   frc::Translation2d{SwerveConstants::SWERVE_MODULE_DIFF_X * DriveConstants::MODULE_DIFF_XS[1],
                                                                      SwerveConstants::SWERVE_MODULE_DIFF_Y *DriveConstants::MODULE_DIFF_YS[1]},
                                                   frc::Translation2d{SwerveConstants::SWERVE_MODULE_DIFF_X * DriveConstants::MODULE_DIFF_XS[2],
                                                                      SwerveConstants::SWERVE_MODULE_DIFF_Y *DriveConstants::MODULE_DIFF_YS[2]},
                                                   frc::Translation2d{SwerveConstants::SWERVE_MODULE_DIFF_X * DriveConstants::MODULE_DIFF_XS[3],
                                                                      SwerveConstants::SWERVE_MODULE_DIFF_Y *DriveConstants::MODULE_DIFF_YS[3]}};

    AHRS navX;
    bool hasGyroOffset;
    frc::Rotation2d gyroOffset;

    frc::SwerveDriveKinematics<4> kinematics;
    frc::SwerveDriveOdometry<4> odometry;
    
     std::shared_ptr<nt::NetworkTable> limeTable;
};

#endif