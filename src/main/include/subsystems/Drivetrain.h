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
#include "controllers/ValorPIDF.h"

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
#include <frc2/command/FunctionalCommand.h>

#include <ctre/phoenix/motorcontrol/NeutralMode.h>
#include <rev/CANSparkMax.h>

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
     typedef ValorNeoController SwerveDriveMotor;

     /**
      * @brief Quick way to select the azimuth motor controller
      * To change what motor controller runs the azimuth motor, change this to either:
      * * ValorFalconController
      * * ValorNeoController
      */
     typedef ValorNeoController SwerveAzimuthMotor;

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

     void InitSendable(wpi::SendableBuilder& builder);

     enum LimelightPipes{
          APRIL_TAGS,
          TAPE_MID,
          TAPE_HIGH
     };

     struct x
     {
          double xSpeed;
          double ySpeed;
          double rot;
          
          bool adas;
          bool lock;

          LimelightPipes limeLocation;
          
          bool xPose;

          bool isLeveled;
          bool abovePitchThreshold;

          bool topTape;
          bool bottomTape;

          int stage;

          int trackingID;
          double visionOdomDiff;

          double matchStart;

          frc::Pose2d visionPose;
          frc::Pose2d prevVisionPose;

          units::velocity::meters_per_second_t xSpeedMPS;
          units::velocity::meters_per_second_t ySpeedMPS;
          units::angular_velocity::radians_per_second_t rotRPS;

          frc::Pose2d prevPose;

          units::second_t startTimestamp; // generic
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
     units::degree_t getGlobalPitch();

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
     frc::Pose2d getVisionPose();
     void addVisionMeasurement(frc::Pose2d visionPose, double doubt);

     /**
      * Returns the kinematics object in use by the swerve drive
      * @return kinematics object
      */
     frc::SwerveDriveKinematics<SWERVE_COUNT>* getKinematics();

     void limelightHoming(LimelightPipes pipe);
     void angleLock();
     void adas(LimelightPipes pipe);
     void setLimelightPipeline(LimelightPipes pipe);

     frc2::FunctionalCommand* getResetOdom();
     
     frc2::FunctionalCommand* getAutoLevel();
     frc2::FunctionalCommand* getVisionAutoLevel();
     frc2::FunctionalCommand* getAutoLevelReversed();
     frc2::FunctionalCommand* getAutoClimbOver();

     frc2::FunctionalCommand* getOLDAutoLevel();
     frc2::FunctionalCommand* getOLDAutoLevelReversed();

     double getDriveMaxSpeed();
     double getAutoMaxSpeed();
     double getAutoMaxAcceleration();
     double getRotationMaxSpeed();
     double getRotationMaxAcceleration();

     void setAutoMaxAcceleration(double acceleration, double multiplier);

     ValorPIDF getXPIDF();
     ValorPIDF getYPIDF();
     ValorPIDF getThetaPIDF();

     frc::TrajectoryConfig & getTrajectoryConfig();

     void setXMode();

     frc2::InstantCommand* getSetXMode();

     void setDriveMotorNeutralMode(ValorNeutralMode mode);

private:
     double driveMaxSpeed;
     double rotMaxSpeed;
     double autoMaxSpeed;
     double autoMaxAccel;
     double rotMaxAccel;

     wpi::array<frc::SwerveModuleState, SWERVE_COUNT> getModuleStates(units::velocity::meters_per_second_t,
                                                           units::velocity::meters_per_second_t,
                                                           units::angular_velocity::radians_per_second_t,
                                                           bool);



     void configSwerveModule(int);

     std::vector<ValorSwerve<SwerveAzimuthMotor, SwerveDriveMotor> *> swerveModules;
     std::vector<SwerveAzimuthMotor *> azimuthControllers;
     std::vector<SwerveDriveMotor *> driveControllers;

     units::meter_t swerveModuleDiff;

     wpi::array<frc::Translation2d, SWERVE_COUNT> motorLocations;

     wpi::array<frc::SwerveModulePosition, SWERVE_COUNT> initPositions;

     WPI_Pigeon2 pigeon;

     frc::SwerveDriveKinematics<SWERVE_COUNT> * kinematics;
     frc::SwerveDrivePoseEstimator<SWERVE_COUNT> * estimator;

     frc::TrajectoryConfig * config;

     ValorPIDF xPIDF;
     ValorPIDF yPIDF;
     ValorPIDF thetaPIDF;

     std::shared_ptr<nt::NetworkTable> limeTable;

     bool swerveNoError;
};
