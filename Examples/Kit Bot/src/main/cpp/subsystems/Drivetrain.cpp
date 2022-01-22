/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "subsystems/Drivetrain.h"
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/shuffleboard/Shuffleboard.h>

Drivetrain::Drivetrain() : ValorSubsystem(),
                           m_leftFrontMotor{DriveConstants::FRONT_LEFT_DRIVE_CAN_ID , rev::CANSparkMax::MotorType::kBrushed},
                           m_rightFrontMotor{DriveConstants::FRONT_RIGHT_DRIVE_CAN_ID , rev::CANSparkMax::MotorType::kBrushed},
                           m_leftBackMotor{DriveConstants::BACK_LEFT_DRIVE_CAN_ID , rev::CANSparkMax::MotorType::kBrushed},
                           m_rightBackMotor{DriveConstants::BACK_RIGHT_DRIVE_CAN_ID , rev::CANSparkMax::MotorType::kBrushed},
                           driverController(NULL)
{
    frc2::CommandScheduler::GetInstance().RegisterSubsystem(this);
    init();
}

Drivetrain::~Drivetrain()
{

}


void Drivetrain::init()
{
    limeTable = nt::NetworkTableInstance::GetDefault().GetTable("limelight");
    initTable("Drivetrain");

    m_leftFrontMotor.SetInverted(true);
    m_leftBackMotor.SetInverted(true);

    m_leftBackMotor.Follow(m_leftFrontMotor, false);
    m_rightBackMotor.Follow(m_rightFrontMotor, false);

}

void Drivetrain::setController(frc::XboxController *controller)
{
    driverController = controller;
}

void Drivetrain::assessInputs()
{
    if (!driverController)
    {
        return;
    }

    // driver inputs
    state.leftStickX = driverController->GetLeftX();
    state.leftStickY = driverController->GetLeftY();
    state.rightStickX = driverController->GetRightX();
    state.rightStickY = driverController->GetRightY();

    state.bButtonPressed = driverController->GetBButton();
    state.aButtonPressed = driverController->GetAButton();
    state.xButtonPressed = driverController->GetXButton();
    state.yButtonPressed = driverController->GetYButton();
}

void Drivetrain::analyzeDashboard()
{

}

void Drivetrain::assignOutputs()
{
    std::abs(state.leftStickY) < DriveConstants::kDeadbandY ? state.leftStickY = 0 : state.leftStickY;
    std::abs(state.rightStickX) < DriveConstants::kDeadbandX ? state.rightStickX = 0 : state.rightStickX;

    //m_leftFrontMotor.Set(state.leftStickY);

    m_drive.ArcadeDrive(-1 * state.leftStickY,state.rightStickX);
}

void Drivetrain::resetState()
{

}