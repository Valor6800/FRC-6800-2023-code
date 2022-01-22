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

    constexpr static int dpadUp = 0;
    constexpr static int dpadRight = 90;
    constexpr static int dpadDown = 180;
    constexpr static int dpadLeft = 270;

    constexpr static double kTriggerDeadband = 0.02;
}

namespace LimelightConstants {
    constexpr static int LED_MODE_ON = 3;
    constexpr static int LED_MODE_OFF = 1;
    constexpr static int TRACK_MODE_ON = 0;
    constexpr static int TRACK_MODE_OFF = 1;
}

namespace DriveConstants {
    constexpr static int FRONT_LEFT_DRIVE_CAN_ID = 2;
    constexpr static int FRONT_RIGHT_DRIVE_CAN_ID = 1;
    constexpr static int BACK_LEFT_DRIVE_CAN_ID = 4;
    constexpr static int BACK_RIGHT_DRIVE_CAN_ID = 3;

    constexpr static double kDeadbandX = 0.04;
    constexpr static double kDeadbandY = 0.04;
}

namespace TestMotorsConstants {
    constexpr static int TEST_MOTOR_CAN_ID_1 = 5;
    constexpr static int TEST_MOTOR_CAN_ID_2 = 6;
    constexpr static int TEST_MOTOR_CAN_ID_3 = 7;
    constexpr static int TEST_MOTOR_CAN_ID_4 = 8;
    constexpr static int TEST_MOTOR_CAN_ID_5 = 9;
    constexpr static int TEST_MOTOR_CAN_ID_6 = 10;
    constexpr static int TEST_MOTOR_CAN_ID_7 = 11;
    constexpr static int TEST_MOTOR_CAN_ID_8 = 12;
    constexpr static int TEST_MOTOR_CAN_ID_9 = 13;
    constexpr static int TEST_MOTOR_CAN_ID_10 = 14;

    constexpr static int TEST_PHOTO_ELEC_DIO_PORT = 1;
}

#endif
