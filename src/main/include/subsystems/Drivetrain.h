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
#include "controllers/ValorFalconController.h"
#include "controllers/ValorNeoController.h"

#include "AHRS.h"
#include "ctre/phoenix/sensors/WPI_Pigeon2.h"
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/estimator/SwerveDrivePoseEstimator.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/DutyCycleEncoder.h>
#include <frc/Timer.h>

#include <frc/geometry/Translation2d.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/kinematics/DifferentialDriveWheelSpeeds.h>
#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc/kinematics/DifferentialDriveKinematics.h>
#include <frc/trajectory/constraint/DifferentialDriveVoltageConstraint.h>
#include <frc/trajectory/Trajectory.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include <frc2/command/SwerveControllerCommand.h>
#include <frc2/command/Command.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/WaitCommand.h>
#include <frc/TimedRobot.h>

#include <ctre/phoenix/motorcontrol/NeutralMode.h>

#define MODULE_DIFF 0.248f
#define SWERVE_COUNT 4

/**
 * @brief Subsystem - Drivetrain
 * 
 * Subsystem responsible for driving the robot chassis, and housing all the logic to control the
 * 4 swerve modules on the robot.
 */
class Drivetrain : public ValorSubsystem
{
public:

     /**
      * @brief Quick way to select the drive motor controller
      * To change what motor controller runs the drive motor, change this to either:
      * * ValorFalconController
      * * ValorNeoController
      */
     typedef ValorFalconController SwerveDriveMotor;

     /**
      * @brief Quick way to select the azimuth motor controller
      * To change what motor controller runs the azimuth motor, change this to either:
      * * ValorFalconController
      * * ValorNeoController
      */
     typedef ValorFalconController SwerveAzimuthMotor;

     /**
      * @brief Construct a new Drivetrain object
      * 
      * @param robot Top level robot object to parse out smart dashboard and table information
      */
     Drivetrain(frc::TimedRobot *robot);

     /**
      * @brief Destroy the Drivetrain object
      * 
      * Drivetrain objects have member objects on the heap - need a destructor to take care of memory on destruction
      */
     ~Drivetrain();

     /**
      * @brief Initialize the drivetrain
      * 
      * Includes:
      * * Calibrating the pigeon
      * * Configuring each swerve module (including controllers for azimuth and drive motors)
      * * Setting the PID values for the Azimuth controller
      * * Resetting the drivetrain state
      */
     void init();

     void assessInputs();
     void analyzeDashboard();
     void assignOutputs();

     void resetState();

     struct x
     {
          double xSpeed;
          double ySpeed;
          double rot;

          double slowDown;
          bool startButton;
          
          bool saveToFileDebouncer;

          bool limecentering;
     } state;

     /**
      * Drive the robot with given x, y and rotational velocities using open loop velocity control
      * @param vx_mps the desired x velocity component in meters per second
      * @param vy_mps the desired y velocity component in meters per second
      * @param omega_radps the desired rotational velocity component in radians per second
      * @param isFOC true if driving field oriented
      */
     void drive(units::velocity::meters_per_second_t vx_mps, units::velocity::meters_per_second_t vy_mps, units::angular_velocity::radians_per_second_t omega_radps, bool isFOC);

     /**
      * Directly set the swerve modules to the specified states
      * @param desiredStates the desired swerve module states
      */
     void setModuleStates(wpi::array<frc::SwerveModuleState, SWERVE_COUNT> desiredStates);

     void resetGyro();

     /**
      * Reset the drive encoders to currently read a position of 0
      */
     void resetDriveEncoders();

     void pullSwerveModuleZeroReference();


     frc::Rotation2d getPigeon();

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
     std::vector<ValorSwerve<SwerveAzimuthMotor, SwerveDriveMotor> *> getSwerveModules();


     /**
      * Returns the current gyro heading of the robot
      * This will be affected by any gyro drift that may have accumulated since last recalibration
      * The angle is continuous from 360 to 361 degrees
      * This allows algorithms that wouldn't want to see a discontinuity in the gyro as it sweeps 
      *   past from 360 to 0 on the second time around.
      * The angle is expected to increase as the gyro turns clockwise
      */
     frc::Rotation2d getHeading(bool);

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
     frc::SwerveDriveKinematics<SWERVE_COUNT>& getKinematics();

     void cancelCmdGoToTag();
     void setDriveMotorModeTo(NeutralMode);

     int trackingID;
private:
     double driveMaxSpeed;
     double rotMaxSpeed;
     double autoMaxSpeed;
     double autoMaxAccel;

     wpi::array<frc::SwerveModuleState, SWERVE_COUNT> getModuleStates(units::velocity::meters_per_second_t,
                                                           units::velocity::meters_per_second_t,
                                                           units::angular_velocity::radians_per_second_t,
                                                           bool);

     void configSwerveModule(int);

     std::vector<ValorSwerve<SwerveAzimuthMotor, SwerveDriveMotor> *> swerveModules;
     std::vector<SwerveAzimuthMotor *> azimuthControllers;
     std::vector<SwerveDriveMotor *> driveControllers;

     std::vector<frc::DutyCycleEncoder*> magEncoders;
     units::meter_t swerveModuleDiff = units::meter_t(MODULE_DIFF);

     std::vector<frc::Translation2d> motorLocations{frc::Translation2d{swerveModuleDiff * DriveConstants::MODULE_DIFF_XS[0],
                                                                       swerveModuleDiff * DriveConstants::MODULE_DIFF_YS[0]},
                                                    frc::Translation2d{swerveModuleDiff * DriveConstants::MODULE_DIFF_XS[1],
                                                                       swerveModuleDiff * DriveConstants::MODULE_DIFF_YS[1]},
                                                    frc::Translation2d{swerveModuleDiff * DriveConstants::MODULE_DIFF_XS[2],
                                                                       swerveModuleDiff * DriveConstants::MODULE_DIFF_YS[2]},
                                                    frc::Translation2d{swerveModuleDiff * DriveConstants::MODULE_DIFF_XS[3],
                                                                       swerveModuleDiff * DriveConstants::MODULE_DIFF_YS[3]}};
     wpi::array<frc::SwerveModulePosition, SWERVE_COUNT> initPositions{frc::SwerveModulePosition{0_m, frc::Rotation2d(0_rad)},
                                                            frc::SwerveModulePosition{0_m, frc::Rotation2d(0_rad)},
                                                            frc::SwerveModulePosition{0_m, frc::Rotation2d(0_rad)},
                                                            frc::SwerveModulePosition{0_m, frc::Rotation2d(0_rad)}};

     WPI_Pigeon2 pigeon;

     frc::SwerveDriveKinematics<SWERVE_COUNT> kinematics;
     frc::SwerveDrivePoseEstimator<SWERVE_COUNT> estimator;

     frc::TrajectoryConfig config;
     frc::TrajectoryConfig reverseConfig;

     frc::ProfiledPIDController<units::radians> thetaController;

     frc::Pose2d translatePoseToCorner(frc::Pose2d);
     std::map<int, frc::Pose2d> tags = {{5, frc::Pose2d{0.36195_m, 6.749796_m, 0_deg}}, {8, frc::Pose2d{1.02743_m, 1.071626_m, 0_deg}}};

     frc2::SwerveControllerCommand<4> *cmdGoToTag;

     std::shared_ptr<nt::NetworkTable> limeTable;
};
