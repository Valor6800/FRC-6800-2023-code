/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

//start button pulls swerve 0 positions from file
//back button pushes current swerve positions to file


#include "subsystems/Shooter.h"
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <iostream>

Shooter::Shooter() : ValorSubsystem(),
                     flywheel_lead{ShooterConstants::CAN_ID_FLYWHEEL_LEAD, rev::CANSparkMax::MotorType::kBrushless},
                     flywheel_follow{ShooterConstants::CAN_ID_FLYWHEEL_FOLLOW, rev::CANSparkMax::MotorType::kBrushless},
                     turret{ShooterConstants::CAN_ID_TURRET, rev::CANSparkMax::MotorType::kBrushless},
                     hood{ShooterConstants::CAN_ID_HOOD, rev::CANSparkMax::MotorType::kBrushless},
                     operatorController(NULL)
                 
{
    frc2::CommandScheduler::GetInstance().RegisterSubsystem(this);
    init();
}

void Shooter::init()
{
    limeTable = nt::NetworkTableInstance::GetDefault().GetTable("limelight");
    initTable("Shooter");
    table->PutBoolean("Home Turret", false);
    
    flywheel_follow.RestoreFactoryDefaults();
    flywheel_lead.RestoreFactoryDefaults();
    turret.RestoreFactoryDefaults();

    flywheel_follow.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
    flywheel_lead.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);
    turret.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);

    turret.SetInverted(false);

    flywheel_lead.SetInverted(true);

    flywheel_lead.Follow(rev::CANSparkMax::kFollowerDisabled, false);
    flywheel_follow.Follow(flywheel_lead, true);

    turret.EnableSoftLimit(rev::CANSparkMax::SoftLimitDirection::kForward, true);
    turret.SetSoftLimit(rev::CANSparkMax::SoftLimitDirection::kForward, ShooterConstants::limitLeft);

    turret.EnableSoftLimit(rev::CANSparkMax::SoftLimitDirection::kReverse, true);
    turret.SetSoftLimit(rev::CANSparkMax::SoftLimitDirection::kReverse, ShooterConstants::limitRight);

    pidController.SetP(ShooterConstants::shooterKP);
    pidController.SetI(ShooterConstants::shooterKI);
    pidController.SetD(ShooterConstants::shooterKD);
    pidController.SetIZone(ShooterConstants::shooterKFF);
    pidController.SetFF(ShooterConstants::shooterKIZ);
    pidController.SetOutputRange(0, 1);

    resetState(); //reset shooter/encoder state
}

void Shooter::resetState(){
    resetEncoder();
    state.turretState = TurretState::TURRET_DEFAULT;
    state.hoodState = HoodState::HOOD_DISABLE;
    state.flywheelState = FlywheelState::FLYWHEEL_DEFAULT;
}

void Shooter::resetEncoder(){
    turretEncoder.SetPosition(0);
    hoodEncoder.SetPosition(0);
}

void Shooter::assessInputs()
{
    if (!operatorController)
    {
        return;
    }

    // driver inputs
    state.leftStickX = operatorController->GetLeftX();
    state.startButton = operatorController->GetStartButton();
    state.backButton = operatorController->GetBackButton(); 
    //state.dPadDownPressed = driverController->GetPOV(frc::GenericHID::)
    
    //Turret
    if (std::abs(state.leftStickX) > ShooterConstants::kDeadband) {
        state.turretState = TurretState::TURRET_MANUAL;
    }
    else if(state.startButton){
       state.turretState = TurretState::TURRET_PRIME;
    }
    else if(state.backButton){
        state.turretState = TurretState::TURRET_DEFAULT;
    }

    //Hood
    if(state.startButton){
        state.hoodState = HoodState::HOOD_PRIME;
    }
    else if(state.backButton){
        state.hoodState = HoodState::HOOD_DISABLE;
    }

    //Flywheel
    if(state.startButton){
        state.flywheelState = FlywheelState::FLYWHEEL_PRIME;
    }
    else if (state.backButton){
        state.flywheelState = FlywheelState::FLYWHEEL_DEFAULT;
    }
}

void Shooter::analyzeDashboard()
{
    if(table->GetBoolean("Home Turret", false)){
        state.turretState = TurretState::TURRET_HOME;
    }
}

void Shooter::assignOutputs()
{   
    state.turretTarget = 0;

    //DISABLED
    if(state.turretState == TurretState::TURRET_DISABLE){
        state.turretTarget = 0;
    }//HOME
    else if(state.turretState == TurretState::TURRET_HOME){
        // @TODO Motion Profile to turret home position
        state.turretTarget = 0;
    } //MANUAL
     else if (state.turretState == TurretState::TURRET_MANUAL) {
        int sign = state.leftStickX >= 0 ? 1 : -1;
        state.turretTarget = sign * std::pow(state.leftStickX, 2) * ShooterConstants::TURRET_SPEED_MULTIPLIER;

        // Minimum power deadband
        if (std::abs(state.turretTarget) < ShooterConstants::pDeadband) {
            state.turretTarget = 0;
        }
        // Stop deadband
        else if (std::abs(state.turretTarget) < ShooterConstants::pSoftDeadband) {
            int direction = 1;
            if (state.turretTarget < 0) direction = -1;
            state.turretTarget = ShooterConstants::pSoftDeadband * direction;
        }
    }//DEFAULT
    else if (state.turretState == TurretState::TURRET_DEFAULT){
        //Odometry tracking
        // @TODO Motion Profile to target turret position
        state.turretTarget = 0;
    }

    turret.Set(state.turretTarget);

    if(state.flywheelState == FlywheelState::FLYWHEEL_DISABLE){
        state.flywheelTarget = 0;
    }
    else if(state.flywheelTarget == FlywheelState::FLYWHEEL_PRIME){
        //math conditions, leaving 1 for now
        // @TODO use dashboard value
        state.flywheelTarget = 1;
    }
    else if (state.flywheelTarget == FlywheelState::FLYWHEEL_DEFAULT){
        // @TODO use dashboard value
        state.flywheelTarget = 0.5;
    }
    
    flywheel_lead.Set(state.flywheelTarget);

    if(state.hoodState == HoodState::HOOD_DISABLE){
        state.hoodTarget = 0;
    }
    else if(state.hoodState == HoodState::HOOD_PRIME){
        // @TODO Motion profile to target hood position
        state.hoodTarget = 0.1;
    }

    hood.Set(state.hoodTarget);
}