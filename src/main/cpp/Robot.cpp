/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Robot.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/CommandScheduler.h>

Robot::Robot() : drivetrain(this), testSubsystem(this)
{
    frc::TimedRobot();
}

void Robot::RobotInit() {
    drivetrain.setController(&gamepadDriver);
    drivetrain.resetState();

    testSubsystem.setControllers(&gamepadDriver, &gamepadOperator);
}

/**
 * This function is called every robot packet, no matter the mode. Use
 * this for items like diagnostics that you want to run during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() { frc2::CommandScheduler::GetInstance().Run(); }

/**
 * This function is called once each time the robot enters Disabled mode. You
 * can use it to reset any subsystem information you want to clear when the
 * robot is disabled.
 */
void Robot::DisabledInit() {
    drivetrain.resetState();
    drivetrain.setMotorMode(false);

    testSubsystem.resetState();
}

void Robot::DisabledPeriodic()
{

}

/**
 * This autonomous runs the autonomous command selected by your {@link
 * RobotContainer} class.
 */
void Robot::AutonomousInit() {
    drivetrain.setMotorMode(true);
    drivetrain.pullSwerveModuleZeroReference();

    if (autoCommand != nullptr) {
        autoCommand->Schedule();
    }
}

void Robot::AutonomousPeriodic()
{

}

void Robot::TeleopInit() {
    drivetrain.setMotorMode(false);

    if (autoCommand != nullptr) {
        autoCommand->Cancel();
        autoCommand = nullptr;
    }

}

/**
 * This function is called periodically during operator control.
 */
void Robot::TeleopPeriodic() {}

/**
 * This function is called periodically during test mode.
 */
void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
