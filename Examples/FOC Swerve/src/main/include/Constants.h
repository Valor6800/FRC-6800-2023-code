/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

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
}

namespace DriveConstants {
    constexpr static int DRIVE_CANS[4] = {0, 2, 4, 6};
    constexpr static int AZIMUTH_CANS[4] = {1, 3, 5, 7};
    constexpr static int MODULE_DIFF_XS[4] = {1, 1, -1, -1};
    constexpr static int MODULE_DIFF_YS[4] = {1, -1, 1, -1};
    constexpr static auto SWERVE_MODULE_DIFF_X = 0.29051_m;
    constexpr static auto SWERVE_MODULE_DIFF_Y = 0.29051_m;

    constexpr static double kDeadbandX = 0.05;
    constexpr static double kDeadbandY = 0.05;
}

namespace FalconSwerveConstants {
    constexpr static double AZIMUTH_COUNTS_PER_REV = 2048;
    constexpr static double DRIVE_COUNTS_PER_REV = 2048;
    constexpr static double DRIVE_GEAR_RATIO = 12.8;
    constexpr static double WHEEL_CIRCUMFERENCE_M = 4 * M_PI;
    constexpr static units::meters_per_second_t DRIVE_DEADBAND_MPS = units::meters_per_second_t{0.2};
    constexpr static units::meters_per_second_t DRIVE_MAX_SPEED_MPS = units::meters_per_second_t{14};
}

#endif
