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
                           motor_stage(FeederConstants::MOTOR_STAGE_CAN_ID, rev::CANSparkMax::MotorType::kBrushless),
                           banner(FeederConstants::BANNER_DIO_PORT)

{
    frc2::CommandScheduler::GetInstance().RegisterSubsystem(this);
    init();
}

void Feeder::init()
{
    initTable("Feeder");
    motor_intake.RestoreFactoryDefaults();
    motor_intake.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
    motor_intake.SetInverted(false);

    motor_stage.RestoreFactoryDefaults();
    motor_stage.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
    motor_stage.SetInverted(false);

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

    state.driver_leftBumperPressed = driverController->GetLeftBumper();
    state.driver_rightBumperPressed = driverController->GetRightBumper();

    // operator inputs
    state.operator_bButtonPressed = operatorController->GetBButton();
    state.operator_aButtonPressed = operatorController->GetAButton();

    state.operator_leftBumperPressed = operatorController->GetLeftBumper();
    

    state.bannerTripped = !banner.Get();

    if (state.driver_rightBumperPressed || state.operator_leftBumperPressed) {
        state.feederState = FeederState::FEEDER_INTAKE2;
    }
    else if (state.operator_bButtonPressed || state.driver_leftBumperPressed) {
        state.feederState = FeederState::FEEDER_REVERSE;
    }
    else if (state.operator_aButtonPressed) {
        if (state.bannerTripped) {
            state.feederState = FeederState::FEEDER_INTAKE1;
        }
        else {
            state.feederState = FeederState::FEEDER_INTAKE2;
        }
    }
    else {
        state.feederState = FeederState::FEEDER_DISABLE;
    }
}

void Feeder::analyzeDashboard()
{
    state.reversed = table->GetBoolean("Reverse Feeder?", false);
    state.intakeReverseSpeed = table->GetNumber("Intake Reverse Speed", FeederConstants::DEFUALT_INTAKE_SPEED_REVERSE);
    state.feederReverseSpeed = table->GetNumber("Feeder Reverse Speed", FeederConstants::DEFUALT_FEEDER_SPEED_REVERSE);
    state.intakeForwardSpeed = table->GetNumber("Intake Forward Speed", FeederConstants::DEFUALT_INTAKE_SPEED_FORWARD);
    state.feederForwardSpeed = table->GetNumber("Feeder Forward Speed", FeederConstants::DEFUALT_FEEDER_SPEED_FORWARD);
}

void Feeder::assignOutputs()
{
    if (state.feederState == FeederState::FEEDER_DISABLE) {
        motor_intake.Set(0);
        motor_stage.Set(0);
    }
    else if (state.feederState == FeederState::FEEDER_INTAKE2) {
        motor_intake.Set(state.intakeForwardSpeed);
        motor_stage.Set(state.feederForwardSpeed);
    }
    else if (state.feederState == Feeder::FEEDER_REVERSE) {
        motor_intake.Set(state.intakeReverseSpeed);
        motor_stage.Set(state.feederReverseSpeed);
    }
    else if (state.feederState == FeederState::FEEDER_INTAKE1) {
        motor_intake.Set(state.intakeForwardSpeed);
        motor_stage.Set(0);
    }
    else {
        motor_intake.Set(0);
        motor_stage.Set(0);
    }
}

void Feeder::resetState()
{
    state.feederState = FeederState::FEEDER_DISABLE;
}