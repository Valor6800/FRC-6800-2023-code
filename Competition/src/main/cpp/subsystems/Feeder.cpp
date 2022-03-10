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
    motor_stage.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    motor_stage.SetInverted(true);

    table->PutBoolean("Reverse Feeder?", false);
    table->PutNumber("Intake Reverse Speed", FeederConstants::DEFAULT_INTAKE_SPEED_REVERSE);
    table->PutNumber("Feeder Reverse Speed", FeederConstants::DEFAULT_FEEDER_SPEED_REVERSE);
    table->PutNumber("Intake Forward Speed", FeederConstants::DEFAULT_INTAKE_SPEED_FORWARD);
    table->PutNumber("Feeder Forward Speed Default", FeederConstants::DEFAULT_FEEDER_SPEED_FORWARD_DEFAULT);
    table->PutNumber("Feeder Forward Speed Shoot", FeederConstants::DEFAULT_FEEDER_SPEED_FORWARD_SHOOT);
    
    resetState();
    
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

    state.driver_rightTriggerPressed = driverController->GetRightTriggerAxis() > OIConstants::kDeadBandTrigger;

    // operator inputs

    state.operator_leftBumperPressed = operatorController->GetLeftBumper();
        
    if (state.operator_leftBumperPressed) {
        state.feederState = FeederState::FEEDER_SHOOT;
        state.spiked = false;
    }
    else if (state.driver_leftBumperPressed) {
        state.feederState = FeederState::FEEDER_REVERSE;
        state.spiked = false;
    }
    else if (state.driver_rightBumperPressed) {
        state.feederState = FeederState::FEEDER_AUTO;
    }
    else if (state.driver_rightTriggerPressed) {
        state.feederState = FeederState::FEEDER_INTAKE;
    }
    else {
        state.feederState = FeederState::FEEDER_DISABLE;
    }
    
    // Calculate instantaneous current
    calcCurrent();
}

void Feeder::analyzeDashboard()
{
    state.reversed = table->GetBoolean("Reverse Feeder?", false);
    state.intakeReverseSpeed = table->GetNumber("Intake Reverse Speed", FeederConstants::DEFAULT_INTAKE_SPEED_REVERSE);
    state.feederReverseSpeed = table->GetNumber("Feeder Reverse Speed", FeederConstants::DEFAULT_FEEDER_SPEED_REVERSE);
    state.intakeForwardSpeed = table->GetNumber("Intake Forward Speed", FeederConstants::DEFAULT_INTAKE_SPEED_FORWARD);
    state.feederForwardSpeedDefault = table->GetNumber("Feeder Forward Speed Default", FeederConstants::DEFAULT_FEEDER_SPEED_FORWARD_DEFAULT);
    state.feederForwardSpeedShoot = table->GetNumber("Feeder Forward Speed Shoot", FeederConstants::DEFAULT_FEEDER_SPEED_FORWARD_SHOOT);
    table->PutNumber("Average Amps", state.instCurrent);
    table->PutBoolean("Spiked: ", state.spiked);
    table->PutBoolean("Banner: ", state.bannerTripped);
}

void Feeder::assignOutputs()
{
    state.bannerTripped = !banner.Get();
    state.currentBanner = state.bannerTripped;

    if (state.feederState == FeederState::FEEDER_DISABLE) {
        motor_intake.Set(0);
        motor_stage.Set(0);
    }
    else if (state.feederState == FeederState::FEEDER_SHOOT) {
        motor_intake.Set(state.intakeForwardSpeed);
        motor_stage.Set(state.feederForwardSpeedShoot);
    }
    else if (state.feederState == Feeder::FEEDER_REVERSE) {
        motor_intake.Set(state.intakeReverseSpeed);
        motor_stage.Set(state.feederReverseSpeed);
    }
    else if (state.feederState == Feeder::FEEDER_AUTO){
        motor_intake.Set(state.intakeForwardSpeed);
        motor_stage.Set(0);
    }
    else if (state.feederState == FeederState::FEEDER_INTAKE) {
        if (state.bannerTripped) {
            if (state.currentBanner && !state.previousBanner) {
                resetDeque();
                state.spiked = false;
            }
            if (state.spiked) {
                motor_intake.Set(0);
                motor_stage.Set(0);
            }
            else {
                if (state.instCurrent > FeederConstants::JAM_CURRENT && state.bannerTripped) {
                    motor_intake.Set(0);
                    motor_stage.Set(0);
                   state.spiked = true;
                }
                else {
                    motor_intake.Set(state.intakeForwardSpeed);
                    motor_stage.Set(0);
                }
            }
        }
        else {
            motor_intake.Set(state.intakeForwardSpeed);
            motor_stage.Set(state.feederForwardSpeedDefault);
        }
    }
    else {
        motor_intake.Set(0);
        motor_stage.Set(0);
    }
    state.previousBanner = state.currentBanner;
}

void Feeder::calcCurrent() {
    state.current_cache.pop_front();
    state.current_cache.push_back(motor_intake.GetOutputCurrent());

    // Calculate average current over the cache size, or circular buffer window
    double sum = 0;
    for (int i = 0; i < FeederConstants::CACHE_SIZE; i++) {
        sum += state.current_cache.at(i);
    }
    state.instCurrent = sum / FeederConstants::CACHE_SIZE;
}

void Feeder::resetDeque() {
    state.current_cache.clear();
    for (int i = 0; i < FeederConstants::CACHE_SIZE; i++) {
        state.current_cache.push_back(0);
    }
}


void Feeder::resetState()
{
    state.feederState = FeederState::FEEDER_DISABLE;

    state.spiked = false;
    state.previousBanner = false;

    resetDeque();
}