/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

//start button pulls swerve 0 positions from file
//back button pushes current swerve positions to file


#include "subsystems/Feeder.h"

Feeder::Feeder() : ValorSubsystem(),
                           driverController(NULL),
                           operatorController(NULL),
                           motor_intake(FeederConstants::MOTOR_INTAKE_CAN_ID, rev::CANSparkMax::MotorType::kBrushless),
                           motor_stage1(FeederConstants::MOTOR_STAGE1_CAN_ID, rev::CANSparkMax::MotorType::kBrushless),
                           motor_stage2(FeederConstants::MOTOR_STAGE2_CAN_ID, rev::CANSparkMax::MotorType::kBrushless),
                           banner_lower(FeederConstants::BANNER_LOWER_DIO_PORT),
                           banner_upper(FeederConstants::BANNER_UPPER_DIO_PORT)

{
    frc2::CommandScheduler::GetInstance().RegisterSubsystem(this);
    init();
}

Feeder::~Feeder()
{
    
}

void Feeder::init()
{
    initTable("Feeder");
}

void Feeder::setControllers(frc::XboxController *controllerO, frc::XboxController *controllerD)
{
    driverController = controllerD;
    operatorController = controllerO;
}

void Feeder::assessInputs()
{
    if (!driverController)
    {
        return;
    }

    // driver inputs
    state.driver_bButtonPressed = driverController->GetBButton();
    state.driver_aButtonPressed = driverController->GetAButton();

    state.driver_leftBumperPressed = driverController->GetLeftBumperPressed();

    // operator inputs
    state.operator_bButtonPressed = operatorController->GetBButton();
    state.operator_aButtonPressed = operatorController->GetAButton();

    state.operator_leftBumperPressed = operatorController->GetLeftBumperPressed();

    state.lowerBannerTripped = banner_upper.Get();
    state.upperBannerTripped = banner_upper.Get();

    if (state.driver_leftBumperPressed || state.operator_leftBumperPressed) {
        state.feederState = FeederState::INTAKE2;
    }
    else if (state.operator_bButtonPressed || state.driver_bButtonPressed) {
        state.feederState = FeederState::REVERSE;
    }
    else if (state.operator_aButtonPressed || state.driver_aButtonPressed) {
        if (state.lowerBannerTripped) {
            if (state.upperBannerTripped) {
                state.feederState = FeederState::DISABLED;
            }
            else {
                state.feederState = FeederState::INTAKE2;
            }
        } 
        else if (state.upperBannerTripped) {
            state.feederState = FeederState::INTAKE2;
        }
        else {
            state.feederState = FeederState::INTAKE1;
        }
    }
    else {
        state.feederState = FeederState::DISABLED;
    }
}

void Feeder::analyzeDashboard()
{
    
}

void Feeder::assignOutputs()
{
    if (state.feederState == FeederState::DISABLED) {
        motor_intake.Set(0);
        motor_stage1.Set(0);
        motor_stage2.Set(0);
    }
    else if (state.feederState == FeederState::INTAKE2) {
        motor_intake.Set(1);
        motor_stage1.Set(1);
        motor_stage2.Set(1);
    }
    else if (state.feederState == Feeder::REVERSE) {
        motor_intake.Set(-1);
        motor_stage1.Set(-1);
        motor_stage2.Set(-1);
    }
    else if (state.feederState == FeederState::INTAKE1) {
        motor_intake.Set(1);
        motor_stage1.Set(1);
        motor_stage2.Set(0);
    }
    else {
        motor_intake.Set(0);
        motor_stage1.Set(0);
        motor_stage2.Set(0);
    }
}

void Feeder::resetState()
{
    
}