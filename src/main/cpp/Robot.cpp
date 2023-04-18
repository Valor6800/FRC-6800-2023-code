/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "Robot.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/CommandScheduler.h>

#include <ctime>

Robot::Robot() : drivetrain(this), intake(this), elevarm(this, &intake), leds(this, &elevarm, &intake, &drivetrain), autonomous(&drivetrain, &intake, &elevarm)
{
    frc::TimedRobot();
}

void Robot::RobotInit() {
    drivetrain.setGamepads(&gamepadOperator, &gamepadDriver);
    elevarm.setGamepads(&gamepadOperator, &gamepadDriver);
    intake.setGamepads(&gamepadOperator, &gamepadDriver);

    drivetrain.resetState();
    autonomous.fillAutoList();

    frc::LiveWindow::EnableAllTelemetry();
    frc::DataLogManager::Start();
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
void Robot::DisabledInit() { }

void Robot::DisabledPeriodic() { }

/**
 * This autonomous runs the autonomous command selected by your {@link
 * RobotContainer} class.
 */
void Robot::AutonomousInit() {
    intake.setConeHoldSpeed(true);
    drivetrain.resetState();
    elevarm.resetState();
    leds.resetState();
    drivetrain.setDriveMotorNeutralMode(ValorNeutralMode::Brake);
    drivetrain.pullSwerveModuleZeroReference();

    drivetrain.state.matchStart = frc::Timer::GetFPGATimestamp().to<double>();

    elevarm.futureState.highStow = false;

    //intake.state.intakeState = Intake::SPIKED;

    autoCommand = autonomous.getCurrentAuto();

    if (autoCommand != nullptr) {
        autoCommand->Schedule();
    }

    outfile.open("/home/lvuser/poseLog" + std::to_string(time(0)) + ".csv");

    drivetrain.setLimelightPipeline(Drivetrain::LimelightPipes::APRIL_TAGS);
}

void Robot::AutonomousExit() {
    outfile.close();
    drivetrain.state.xPose = true;
    intake.setConeHoldSpeed(false);
    elevarm.futureState.highStow = true;

}

std::string makePoseLog(frc::Pose2d pose){
    return std::to_string(frc::Timer::GetFPGATimestamp().to<double>()) + "," + std::to_string(pose.X().to<double>()) + "," + std::to_string(pose.Y().to<double>()) + "," + std::to_string(pose.Rotation().Degrees().to<double>()) + "\n";
}

void Robot::AutonomousPeriodic(){
    outfile << makePoseLog(drivetrain.getPose_m());
}

void Robot::TeleopInit() {
    drivetrain.pullSwerveModuleZeroReference();
    drivetrain.setDriveMotorNeutralMode(ValorNeutralMode::Coast);

    elevarm.teleopStart = frc::Timer::GetFPGATimestamp().to<double>();
    elevarm.setArmPIDF(false);

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
