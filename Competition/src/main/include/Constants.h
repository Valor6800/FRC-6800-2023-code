/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc/kinematics/DifferentialDriveKinematics.h>
#include <frc/trajectory/constraint/DifferentialDriveVoltageConstraint.h>
#include <cmath>

/**
 * The Constants header provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants.  This should not be used for any other
 * purpose.
 *
 * It is generally a good idea to place constants into subsystem- or
 * command-specific namespaces within this header, which can then be used where
 * they are needed.
 */

#ifndef CONSTANTS_H
#define CONSTANTS_H

//CONTROLLER 0 is operator
//CONTROLLER 1 is driver

namespace OIConstants {
    constexpr static int GAMEPAD_BASE_LOCATION = 1;
    constexpr static int GAMEPAD_OPERATOR_LOCATION = 0;

    constexpr static int dpadUp = 0;
    constexpr static int dpadRight = 90;
    constexpr static int dpadDown = 180;
    constexpr static int dpadLeft = 270;

    constexpr static double kDeadbandX = 0.05;
    constexpr static double kDeadbandY = 0.05;

    constexpr static double kDeadBandTrigger = 0.05;
}

namespace LimelightConstants {
    constexpr static int LED_MODE_ON = 3;
    constexpr static int LED_MODE_OFF = 1;
    constexpr static int TRACK_MODE_ON = 0;
    constexpr static int TRACK_MODE_OFF = 1;
}

namespace DriveConstants {
    constexpr static int DRIVE_CANS[4] = {1, 3, 5, 7};
    constexpr static int AZIMUTH_CANS[4] = {2, 4, 6, 8};
    constexpr static int PIGEON_CAN = 61;
    constexpr static int MAG_ENCODER_PORTS[4] = {1, 2, 3, 4};
    constexpr static int MODULE_DIFF_XS[4] = {1, 1, -1, -1}; //{1, 1, -1, -1};
    constexpr static int MODULE_DIFF_YS[4] = {1, -1, 1, -1}; //{-1, 1, -1, 1};

    constexpr static double TURN_KP = 3.8 * M_PI / 180.0;
    constexpr static double LIMELIGHT_KP = .02;

    constexpr static double KPX = .5; //.75
    constexpr static double KIX = 0.0; //0
    constexpr static double KDX = 0.0; //.1

    constexpr static double KPY = .5; //.75
    constexpr static double KIY = 0.0; //0
    constexpr static double KDY = 0.0; //.1

    constexpr static double KPT = 4; //2.5
    constexpr static double KIT = 0.0; //0
    constexpr static double KDT = 0.0; //.1
}

namespace SwerveConstants {
    constexpr static double module_diff = 0.248;
    constexpr static auto SWERVE_MODULE_DIFF_X = units::meter_t(module_diff);
    constexpr static auto SWERVE_MODULE_DIFF_Y = units::meter_t(module_diff);

    constexpr static double AZIMUTH_COUNTS_PER_REV = 2048;
    constexpr static double DRIVE_COUNTS_PER_REV = 2048;
    constexpr static double MAG_COUNTS_PER_REV = 4096;

    constexpr static double DRIVE_GEAR_RATIO = 1.0 / 5.12; // 1/8.14
    constexpr static double AZIMUTH_GEAR_RATIO = (15.0 / 32.0) * (10.0 / 60.0); // 0.078125

    constexpr static double WHEEL_DIAMETER_M = 0.1016; //.091 with black tread // original number with blue tread 0.1016;
    constexpr static double WHEEL_CIRCUMFERENCE_M = WHEEL_DIAMETER_M * M_PI;

    constexpr static double MOTOR_FREE_SPEED = 6380.0;

    constexpr static units::meters_per_second_t DRIVE_DEADBAND_MPS = units::meters_per_second_t{0.05};

    constexpr static double DRIVE_MAX_SPEED_MPS = MOTOR_FREE_SPEED / 60.0 * DRIVE_GEAR_RATIO * WHEEL_DIAMETER_M * M_PI;
    constexpr static double AUTO_MAX_SPEED_MPS = DRIVE_MAX_SPEED_MPS;
    constexpr static double AUTO_MAX_ACCEL_MPSS = AUTO_MAX_SPEED_MPS * .6; // * 1

    constexpr static double ROTATION_MAX_SPEED_RPS = 2*M_PI;// DRIVE_MAX_SPEED_MPS / std::hypot(module_diff / 2, module_diff / 2);
    constexpr static double AUTO_MAX_ROTATION_RPS = 4 * M_PI;
    constexpr static double AUTO_MAX_ROTATION_ACCEL_RPSS = AUTO_MAX_ROTATION_RPS * .5; // * 1

    constexpr static double MOTION_CRUISE_VELOCITY = 17000; // 21.9k is max. We chose 80%ish of that number
    constexpr static double MOTION_ACCELERATION = MOTION_CRUISE_VELOCITY * 20; // Acceleration matching cruise velocity means 1 second to hit cruise
    constexpr static double KF = 0.05; // Calculated via kF math in the Phoenix documentation
    
    constexpr static double KP = 0.2; //.2
    constexpr static double KI = 0.0; //0
    constexpr static double KD = 0.1; //.1
}

namespace ShooterConstants{
    constexpr static int CAN_ID_FLYWHEEL_FOLLOW = 13;
    constexpr static int CAN_ID_FLYWHEEL_LEAD = 14;
    constexpr static int CAN_ID_TURRET = 12;
    constexpr static int CAN_ID_HOOD = 15;    

    constexpr static double aPower = 0.12;
    constexpr static double bPower = 0.26; //.215
    constexpr static double aHood = 22.6;
    constexpr static double bHood = -25;
    
    constexpr static double limelightTurnKP = (.3 / 25.445) * 1.25;
    constexpr static double limelightAngle = 50;
    constexpr static double hubHeight = 2.64;
    constexpr static double limelightHeight = .6075;
    
    constexpr static double flywheelKP1 = 0.1;
    constexpr static double flywheelKI1 = 0;
    constexpr static double flywheelKD1 = 0;
    constexpr static double flywheelKIZ1 = 0;
    constexpr static double flywheelKFF1 = 0.04;

    constexpr static double flywheelKP0 = 0.1  ; //.25
    constexpr static double flywheelKI0 = 0;
    constexpr static double flywheelKD0 = 0;
    constexpr static double flywheelKIZ0 = 0;
    constexpr static double flywheelKFF0 = 0.04;

    constexpr static double MaxRPM = 6380;

    constexpr static double flywheelCruiseVelo = 20000;
    constexpr static double flywheelMinV = 0;
    constexpr static double flywheelMaxAccel = flywheelCruiseVelo * 1;
    constexpr static double flywheelAllowedError = 0;

    constexpr static double flywheelPrimedValue = 0.46;
    constexpr static double flywheelAutoValue = 0.405; //can change to .4
    constexpr static double flywheelDefaultValue = 0.42; //.375
    constexpr static double flywheelPoopValue = 0.25;
    constexpr static double flywheelLaunchpadValue = .455;    
    
    constexpr static double flywheelSpeeds[] = {.372, .38125, .372}; //.387, .39125
    constexpr static double hoodAngles[] = {5, 9, 5};

    constexpr static double turretKP = 1e-5;
    constexpr static double turretKI = 0;
    constexpr static double turretKD = 0;
    constexpr static double turretKIZ = 0;
    constexpr static double turretKFF = 0.0001;

    constexpr static double turretMaxV = 10000;
    constexpr static double turretMinV = 0;
    constexpr static double turretMaxAccel = turretMaxV * 2;
    constexpr static double turretAllowedError = 0.75;

    constexpr static double hoodKP = 5e-5;
    constexpr static double hoodKI = 0;
    constexpr static double hoodKD = 0;
    constexpr static double hoodKIZ = 0;
    constexpr static double hoodKFF = 0.000156 * .5;

    constexpr static double hoodMaxV = 8000;
    constexpr static double hoodMinV = 0;
    constexpr static double hoodMaxAccel = hoodMaxV * 1;
    constexpr static double hoodAllowedError = 0;

    constexpr static double hoodTop = 5;
   // constexpr static double hoodAuto = 6;
    constexpr static double hoodBottom = 0;
    constexpr static double hoodPoop = 0;
    constexpr static double hoodLaunchpad = 17.6;

    constexpr static double hoodLimitTop = 22;
    constexpr static double hoodLimitBottom = 0;

    constexpr static double hoodGearRatio = 1 / 454.17;

    constexpr static double kDeadband = .08;

    constexpr static double pDeadband = .08;
    constexpr static double TURRET_SPEED_MULTIPLIER = .75;
    constexpr static double pSoftDeadband = 0.1;

    constexpr static double falconMaxRPM = 6380;
    constexpr static double falconGearRatio = 1;

    constexpr static double turretGearRatio = 1.0 / 60;

    // Encoder ticks off of center
    // 192 (gear ration) * angle ratio (ex. 1/2 for 180 deg)

    constexpr static double homePositionMid = 90;
    constexpr static double homePositionLeft = 180;
    constexpr static double homePositionRight = 0;
    constexpr static double turretLimitLeft = 180;
    constexpr static double turretLimitRight = 0;

    constexpr static double turretRotateLiftThreshold = 20000; // lowered from 64500
    constexpr static double hubX = 0;
    constexpr static double hubY = 0;

    constexpr static double cornerX = 0;
    constexpr static double cornerY = 0;

    constexpr static double ticsPerRev = 2048;
}

namespace FeederConstants{
    constexpr static int MOTOR_INTAKE_CAN_ID = 9;
    constexpr static int MOTOR_STAGE_CAN_ID = 10;

    constexpr static int BANNER_DIO_PORT = 5;

    constexpr static double DEFAULT_INTAKE_SPEED_FORWARD = 0.9;
    constexpr static double DEFAULT_INTAKE_SPEED_REVERSE = -0.9;

    constexpr static double DEFAULT_FEEDER_SPEED_FORWARD_DEFAULT = 0.5;
    constexpr static double DEFAULT_FEEDER_SPEED_FORWARD_SHOOT = 0.98;
    constexpr static double DEFAULT_FEEDER_SPEED_REVERSE = -1.0;

    constexpr static int CACHE_SIZE = 25;
    constexpr static double JAM_CURRENT = 35;
}

namespace MathConstants{
    constexpr static double toRadians = M_PI / 180.0;
    constexpr static double toDegrees = 180.0 / M_PI;

    constexpr static double ticksToRads = 2.0 * M_PI  / SwerveConstants::AZIMUTH_COUNTS_PER_REV * SwerveConstants::AZIMUTH_GEAR_RATIO;
    constexpr static double radsToTicks = 1 / ticksToRads;
}

namespace LiftConstants{
    constexpr static int MAIN_CAN_ID = 16;
    constexpr static int MAIN_FOLLOW_CAN_ID = 17;
    constexpr static int ROTATE_CAN_ID = 18;

    constexpr static int MAIN_FIRST_POSITION = 62000;
    constexpr static int MAIN_SECOND_POSITION = 78500;
    constexpr static double MAIN_DOWN_POSITION = 125;

    constexpr static int ROTATE_FIRST_POSITION = 40;

    constexpr static double rotateForwardLimit = 40;
    constexpr static double rotateReverseLimit = 0;

    constexpr static double extendForwardLimit = 103000;
    constexpr static double extendReverseLimit = 125;

    constexpr static double pivotGearRatio = 1 / 95.67;

    constexpr static double DEFAULT_MAIN_EXTEND_SPD = 0.65;
    constexpr static double DEFAULT_MAIN_RETRACT_SPD = 0.65;

    constexpr static double kDeadBandTrigger = 0.9;

    constexpr static double rotateNoLowerThreshold = 77500;

    constexpr static double rotate_kP = 5e-5;
    constexpr static double rotate_kI = 0;
    constexpr static double rotate_kD = 0;
    constexpr static double rotate_kIz = 0;
    constexpr static double rotate_kFF = 0.000156 / 2;
    constexpr static double rotate_kMaxOutput = 0.2;   //1
    constexpr static double rotate_kMinOutput = -0.2;  //-1

    constexpr static double rotate_kMaxVel = 5000;
    constexpr static double rotate_kMinVel = 0; 
    constexpr static double rotate_kMaxAcc = rotate_kMaxVel * 1; 
    constexpr static double rotate_kAllErr = 0;

    constexpr static double main_KF = 0.05;
    constexpr static double main_KD = 0.0;
    constexpr static double main_KI = 0.0;
    constexpr static double main_KP = 0.1;

    constexpr static double MAIN_MOTION_CRUISE_VELOCITY = 15000;
    constexpr static double MAIN_MOTION_ACCELERATION = MAIN_MOTION_CRUISE_VELOCITY * 7;

    constexpr static double DEFAULT_EXTEND_SPD = 0.2;
    constexpr static double DEFAULT_RETRACT_SPD = -0.2;

    constexpr static double DEFAULT_ROTATE_SPD = 0.1;
}

#endif
