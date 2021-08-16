/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

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

    // Default Gateway 10.68.0.1
    // Rio 10.68.0.2
    // Limelight 10.68.0.11:5801
    // mask 24
/**
 * Limelight tuning values
 * Input TAB
 *  Exposure: 2
 *  Black level offset: 5
 *  Red Balance: 1500
 *  Blue Balance: 1920
 * Threshholding
 *  Hue: 0 - 179
 *  Saturation: 118 - 255
 *  Value: 152 - 255
 * Contour Filtering
 *  Area: 0.0534 - 100.000
 *  Fullness: 0.2 - 100.0
 *  W/H Ratio: 0.5120 - 0.8201
 *  Direction Filter: none
 *  Target Grouping: Single Target
 */
}

namespace DriveConstants {
    constexpr static int CAN_ID_LEFT_A = 1;
    constexpr static int CAN_ID_LEFT_B = 3;
    constexpr static int CAN_ID_RIGHT_A = 2;
    constexpr static int CAN_ID_RIGHT_B = 4;

    constexpr static double kDeadbandX = 0.05;
    constexpr static double kDeadbandY = 0.1;
    constexpr static double kArcTurnMultipler = 0.5;
    constexpr static double kNoBoost = 0.5;
    constexpr static double kBoost = 1;
    constexpr static double kDriveMultiplierX = 0.60;
    constexpr static double kDriveMultiplierY = 1;
}

namespace LimelightConstants {
    constexpr static int LED_MODE_ON = 3;
    constexpr static int LED_MODE_OFF = 1;
    constexpr static int TRACK_MODE_ON = 0;
    constexpr static int TRACK_MODE_OFF = 1;

    constexpr static double TrACKING_OFFSET = 2; 
}

#endif
