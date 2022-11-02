/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <cmath>
#include <iostream>
// When trying to compile against other targets for simulation, cmath doesn't include M_PI
//   Therefore if not defined, define M_PI for use on other targets
#ifndef M_PI
#define M_PI (3.14159265358979323846264338327950288)
#endif

#include <frc/kinematics/DifferentialDriveKinematics.h>
#include <frc/trajectory/constraint/DifferentialDriveVoltageConstraint.h>


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
    constexpr static double kDeadbandY = 0.1;

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
    constexpr static double DRIVE_SLOW_SPEED_MPS = 1;

    constexpr static double ROTATION_MAX_SPEED_RPS = 2*M_PI;// DRIVE_MAX_SPEED_MPS / std::hypot(module_diff / 2, module_diff / 2);
    constexpr static double AUTO_MAX_ROTATION_RPS = 4 * M_PI;
    constexpr static double AUTO_MAX_ROTATION_ACCEL_RPSS = AUTO_MAX_ROTATION_RPS * .5; // * 1
    constexpr static double ROTATION_SLOW_SPEED_RPS = 1*M_PI;

    constexpr static double MOTION_CRUISE_VELOCITY = 17000; // 21.9k is max. We chose 80%ish of that number
    constexpr static double MOTION_ACCELERATION = MOTION_CRUISE_VELOCITY * 20; // Acceleration matching cruise velocity means 1 second to hit cruise
    constexpr static double KF = 0.05; // Calculated via kF math in the Phoenix documentation
    
    constexpr static double KP = 0.2; //.2
    constexpr static double KI = 0.0; //0
    constexpr static double KD = 0.1; //.1
}

namespace DIOPorts {
    constexpr static int BANNER = 5;
}

namespace MathConstants{
    constexpr static double toRadians = M_PI / 180.0;
    constexpr static double toDegrees = 180.0 / M_PI;

    constexpr static double ticksToRads = 2.0 * M_PI  / SwerveConstants::AZIMUTH_COUNTS_PER_REV * SwerveConstants::AZIMUTH_GEAR_RATIO;
    constexpr static double radsToTicks = 1 / ticksToRads;
}

namespace CANIDs {
    constexpr static int INTAKE = 9;
    constexpr static int STAGE = 10;

    constexpr static int TURRET = 12;
    constexpr static int FLYWHEEL_LEAD = 14;
    constexpr static int HOOD = 15;

    constexpr static int LIFT_EXTEND = 16;
    constexpr static int LIFT_FOLLOW = 17;
    constexpr static int LIFT_ROTATE = 18;
}

#endif
