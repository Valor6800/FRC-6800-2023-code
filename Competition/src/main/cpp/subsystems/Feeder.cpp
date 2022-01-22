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

void Feeder::init()
{
    initTable("Feeder");

    table->PutBoolean("Reverse Feeder?", false);
    table->PutNumber("Intake Reverse Speed", FeederConstants::DEFUALT_INTAKE_SPEED_REVERSE);
    table->PutNumber("Feeder Reverse Speed", FeederConstants::DEFUALT_FEEDER_SPEED_REVERSE);
    table->PutNumber("Intake Forward Speed", FeederConstants::DEFUALT_INTAKE_SPEED_FORWARD);
    table->PutNumber("Feeder Forward Speed", FeederConstants::DEFUALT_FEEDER_SPEED_FORWARD);
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
        state.feederState = FeederState::FEEDER_INTAKE2;
    }
    else if (state.operator_bButtonPressed || state.driver_bButtonPressed) {
        state.feederState = FeederState::FEEDER_REVERSE;
    }
    else if (state.operator_aButtonPressed || state.driver_aButtonPressed) {
        if (state.lowerBannerTripped) {
            state.feederState = state.upperBannerTripped ? FeederState::FEEDER_DISABLE : FeederState::FEEDER_INTAKE2;
        } 
        else if (state.upperBannerTripped) {
            state.feederState = FeederState::FEEDER_INTAKE2;
        }
        else {
            state.feederState = FeederState::FEEDER_INTAKE1;
        }
    }
    else {
        state.feederState = FeederState::FEEDER_DISABLE;
    }
}

void Feeder::analyzeDashboard()
{
    state.reversed = table->PutBoolean("Reverse Feeder?", false);
    state.intakeReverseSpeed = table->PutNumber("Intake Reverse Speed", FeederConstants::DEFUALT_INTAKE_SPEED_REVERSE);
    state.feederReverseSpeed = table->PutNumber("Feeder Reverse Speed", FeederConstants::DEFUALT_FEEDER_SPEED_REVERSE);
    state.intakeForwardSpeed = table->PutNumber("Intake Forward Speed", FeederConstants::DEFUALT_INTAKE_SPEED_FORWARD);
    state.feederForwardSpeed = table->PutNumber("Feeder Forward Speed", FeederConstants::DEFUALT_FEEDER_SPEED_FORWARD);
}

void Feeder::assignOutputs()
{
    if (state.feederState == FeederState::FEEDER_DISABLE) {
        motor_intake.Set(0);
        motor_stage1.Set(0);
        motor_stage2.Set(0);
    }
    else if (state.feederState == FeederState::FEEDER_INTAKE2) {
        motor_intake.Set(state.intakeForwardSpeed);
        motor_stage1.Set(state.feederForwardSpeed);
        motor_stage2.Set(state.feederForwardSpeed);
    }
    else if (state.feederState == Feeder::FEEDER_REVERSE) {
        motor_intake.Set(state.intakeForwardSpeed);
        motor_stage1.Set(state.feederReverseSpeed);
        motor_stage2.Set(state.feederReverseSpeed);
    }
    else if (state.feederState == FeederState::FEEDER_INTAKE1) {
        motor_intake.Set(state.intakeForwardSpeed);
        motor_stage1.Set(state.feederForwardSpeed);
        motor_stage2.Set(0);
    }
    else {
        motor_intake.Set(0);
        motor_stage1.Set(0);
        motor_stage2.Set(0);
    }

    // @TODO use the variables to set the speeds
}

void Feeder::resetState()
{
    state.feederState = FeederState::FEEDER_DISABLE;
}