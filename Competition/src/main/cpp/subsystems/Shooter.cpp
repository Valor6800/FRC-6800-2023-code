/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "subsystems/Shooter.h"
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <iostream>

Shooter::Shooter() : ValorSubsystem(),
                    flywheel_lead{ShooterConstants::CAN_ID_FLYWHEEL_LEAD},
                    turret{ShooterConstants::CAN_ID_TURRET, rev::CANSparkMax::MotorType::kBrushless},
                    hood{ShooterConstants::CAN_ID_HOOD, rev::CANSparkMax::MotorType::kBrushless},
                    operatorController(NULL),
                    driverController(NULL)
{
    frc2::CommandScheduler::GetInstance().RegisterSubsystem(this);
    init();
}

void Shooter::setDrivetrain(Drivetrain *dt){
    odom = dt;
}

void Shooter::init()
{
    limeTable = nt::NetworkTableInstance::GetDefault().GetTable("limelight");
    liftTable = nt::NetworkTableInstance::GetDefault().GetTable("Lift");
    initTable("Shooter");
    
    table->PutBoolean("Zero Turret", false);

    table->PutBoolean("Zero Hood", false);
    table->PutBoolean("Pit Disable", false);
    table->PutNumber("Flywheel Primed Value", ShooterConstants::flywheelPrimedValue);
    table->PutNumber("Flywheel Default Value", ShooterConstants::flywheelDefaultValue);
    table->PutNumber("Hood Top Position", ShooterConstants::hoodTop);
    table->PutNumber("Hood Bottom Position", ShooterConstants::hoodBottom);
    
    table->PutNumber("Hood Y Int 1X", ShooterConstants::cHood_1x);
    table->PutNumber("Power Y Int 1X", ShooterConstants::cPower_1x);

    // table->PutNumber("Hood Y Int 2X", ShooterConstants::cHood_2x);
    // table->PutNumber("Power Y Int 2X", ShooterConstants::cPower_2x);

    flywheel_lead.ConfigFactoryDefault();
    flywheel_lead.ConfigAllowableClosedloopError(0, 0);
    flywheel_lead.Config_IntegralZone(0, 0);
    flywheel_lead.Config_kF(0, ShooterConstants::flywheelKFF0);
    flywheel_lead.Config_kD(0, ShooterConstants::flywheelKD0);
    flywheel_lead.Config_kI(0, ShooterConstants::flywheelKI0);
    flywheel_lead.Config_kP(0, ShooterConstants::flywheelKP0);
     
    flywheel_lead.ConfigAllowableClosedloopError(1, 0);
    flywheel_lead.Config_IntegralZone(1, 0);
    flywheel_lead.Config_kF(1, ShooterConstants::flywheelKFF1);
    flywheel_lead.Config_kD(1, ShooterConstants::flywheelKD1);
    flywheel_lead.Config_kI(1, ShooterConstants::flywheelKI1);
    flywheel_lead.Config_kP(1, ShooterConstants::flywheelKP1);

    flywheel_lead.ConfigMotionAcceleration(ShooterConstants::flywheelMaxAccel);
    flywheel_lead.ConfigMotionCruiseVelocity(ShooterConstants::flywheelCruiseVelo);
    flywheel_lead.SetNeutralMode(NeutralMode::Coast);
    flywheel_lead.ConfigPeakOutputReverse(0);

    flywheel_lead.SetInverted(false);
    flywheel_lead.SelectProfileSlot(0, 0);

    turret.RestoreFactoryDefaults();
    turret.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    turret.SetInverted(true);

    turret.EnableSoftLimit(rev::CANSparkMax::SoftLimitDirection::kForward, true);
    turret.SetSoftLimit(rev::CANSparkMax::SoftLimitDirection::kForward, ShooterConstants::turretLimitLeft);

    turret.EnableSoftLimit(rev::CANSparkMax::SoftLimitDirection::kReverse, true);
    turret.SetSoftLimit(rev::CANSparkMax::SoftLimitDirection::kReverse, ShooterConstants::turretLimitRight);

    turretPidController.SetP(ShooterConstants::turretKP);
    turretPidController.SetI(ShooterConstants::turretKI);
    turretPidController.SetD(ShooterConstants::turretKD);
    turretPidController.SetIZone(ShooterConstants::turretKIZ);
    turretPidController.SetFF(ShooterConstants::turretKFF);
    turretPidController.SetOutputRange(-1, 1);

    turretPidController.SetSmartMotionMaxVelocity(ShooterConstants::turretMaxV);
    turretPidController.SetSmartMotionMinOutputVelocity(ShooterConstants::turretMinV);
    turretPidController.SetSmartMotionMaxAccel(ShooterConstants::turretMaxAccel);
    turretPidController.SetSmartMotionAllowedClosedLoopError(ShooterConstants::turretAllowedError);

    turretEncoder.SetPositionConversionFactor(360.0 * ShooterConstants::turretGearRatio);
    
    hood.RestoreFactoryDefaults();
    hood.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    hood.SetInverted(true);

    hood.EnableSoftLimit(rev::CANSparkMax::SoftLimitDirection::kForward, true);
    hood.SetSoftLimit(rev::CANSparkMax::SoftLimitDirection::kForward, ShooterConstants::hoodLimitTop);

    hood.EnableSoftLimit(rev::CANSparkMax::SoftLimitDirection::kReverse, true);
    hood.SetSoftLimit(rev::CANSparkMax::SoftLimitDirection::kReverse, ShooterConstants::hoodLimitBottom);

    hoodEncoder.SetPositionConversionFactor(360 * ShooterConstants::hoodGearRatio);

    hoodPidController.SetP(ShooterConstants::hoodKP);
    hoodPidController.SetI(ShooterConstants::hoodKI);
    hoodPidController.SetD(ShooterConstants::hoodKD);
    hoodPidController.SetIZone(ShooterConstants::hoodKIZ);
    hoodPidController.SetFF(ShooterConstants::hoodKFF);
    hoodPidController.SetOutputRange(-1, 1);

    hoodPidController.SetSmartMotionMaxVelocity(ShooterConstants::hoodMaxV);
    hoodPidController.SetSmartMotionMinOutputVelocity(ShooterConstants::hoodMinV);
    hoodPidController.SetSmartMotionMaxAccel(ShooterConstants::hoodMaxAccel);
    hoodPidController.SetSmartMotionAllowedClosedLoopError(ShooterConstants::hoodAllowedError);

    state.pipeline = 0;
    state.LoBFZoom = 1;
    
    resetState();
    resetEncoder();

    limelightTrack(true);

    setPIDProfile(0);
}

void Shooter::resetState(){
    state.turretState = TurretState::TURRET_TRACK;
    //hack solution
    state.lastTurretState = TurretState::TURRET_DISABLE;
    state.hoodState = HoodState::HOOD_TRACK;
    state.flywheelState = FlywheelState::FLYWHEEL_TRACK;
    state.trackCorner = false;

    state.turretOutput = 0;
    state.turretTarget = 0;

    state.flywheelTarget = 0;
    state.hoodTarget = 0;
    state.distanceToHub = 0.5;//change to 0?
    state.currentBall = 0;
}

void Shooter::resetEncoder(){
    turretEncoder.SetPosition(0);
    hoodEncoder.SetPosition(0);
}

void Shooter::setControllers(frc::XboxController *controllerO, frc::XboxController *controllerD)
{
    operatorController = controllerO;
    driverController = controllerD;
}

void Shooter::limelightTrack(bool track){
    limeTable->PutNumber("ledMode", track ? LimelightConstants::LED_MODE_ON : LimelightConstants::LED_MODE_OFF);
    limeTable->PutNumber("camMode", track ? LimelightConstants::TRACK_MODE_ON : LimelightConstants::TRACK_MODE_OFF);
}

void Shooter::assessInputs()
{
    if (!operatorController)
    {
        return;
    }
    state.startButton = operatorController->GetStartButtonPressed();
    state.backButton = operatorController->GetBackButtonPressed(); 
    state.rightBumper = operatorController->GetRightBumper();
    state.leftStickX = -operatorController->GetLeftX();
    state.aButton = operatorController->GetAButtonPressed();
    state.yButton = operatorController->GetYButton();
    state.xButtonPressed = operatorController->GetXButtonPressed();
    state.bButton = operatorController->GetBButtonPressed();
    state.driverLeftTrigger = driverController->GetLeftTriggerAxis() > 0.9;
    
    //Turret
    if (std::abs(state.leftStickX) > ShooterConstants::kDeadband) {
        state.turretState = TurretState::TURRET_MANUAL; // Operator control
    }
    else if (state.bButton){
        state.turretState = TurretState::TURRET_TRACK; // Not moving
    }
    else if(state.turretState == TurretState::TURRET_MANUAL || (fabs(ShooterConstants::homePositionMid - turretEncoder.GetPosition()) < ShooterConstants::turretAllowedError * 10 && state.turretState == TurretState::TURRET_HOME_MID)){
        state.turretState = TurretState::TURRET_TRACK;
    }

    if (state.yButton) {
        state.turretState = TurretState::TURRET_HOME_MID;
    }

    //Hood
    
    if(state.aButton){
        state.hoodState = HoodState::HOOD_DOWN; // Low position
    }
    else if(state.xButtonPressed){
        state.hoodState = HoodState::HOOD_POOP;
    }
    else if (state.bButton){
        state.hoodState = HoodState::HOOD_TRACK; // High position
    }

    //Flywheel
    if (state.aButton){
        state.flywheelState = FlywheelState::FLYWHEEL_DEFAULT; // Lower speed
    }
    else if (state.xButtonPressed){
        state.flywheelState = FlywheelState::FLYWHEEL_POOP;
    }
    else if (state.bButton){
        state.flywheelState = FlywheelState::FLYWHEEL_TRACK; // Higher speed
    }

    state.trackCorner = false;//state.rightBumper ? true : false;
}

void Shooter::analyzeDashboard()
{
    // Limelight Distance calculations
    // Only update if a target is visible. Value is sticky if no target is present
    if (limeTable->GetNumber("tv", 0.0) == 1.0) {
        double angle = limeTable->GetNumber("ty", 0.0) + ShooterConstants::limelightAngle;
        double deltaH = ShooterConstants::hubHeight - ShooterConstants::limelightHeight;
        double xDist = deltaH / tan(angle * MathConstants::toRadians);
        state.distanceToHub = xDist;
        table->PutNumber("x distance to hub", xDist);
    }

    if (table->GetBoolean("Pit Disable", false)){
        state.turretState = TurretState::TURRET_DISABLE;
        state.hoodState = HoodState::HOOD_DOWN;
        state.flywheelState = FlywheelState::FLYWHEEL_DISABLE;
    }

    // Turret homing and zeroing
    if (table->GetBoolean("Zero Turret", false)) {
        turret.RestoreFactoryDefaults();
        turret.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
        turret.SetInverted(true);

        turret.EnableSoftLimit(rev::CANSparkMax::SoftLimitDirection::kForward, true);
        turret.SetSoftLimit(rev::CANSparkMax::SoftLimitDirection::kForward, ShooterConstants::turretLimitLeft);

        turret.EnableSoftLimit(rev::CANSparkMax::SoftLimitDirection::kReverse, true);
        turret.SetSoftLimit(rev::CANSparkMax::SoftLimitDirection::kReverse, ShooterConstants::turretLimitRight);

        turretEncoder = turret.GetEncoder();
        turretEncoder.SetPosition(0);
        turretEncoder.SetPositionConversionFactor(360.0 * ShooterConstants::turretGearRatio);
    }

    // Hood zeroing
    if (table->GetBoolean("Zero Hood", false)){
        hoodEncoder.SetPosition(0);
    }

    //slider
    state.flywheelLow = table->GetNumber("Flywheel Default Value", ShooterConstants::flywheelDefaultValue);
    state.flywheelHigh = table->GetNumber("Flywheel Primed Value", ShooterConstants::flywheelPrimedValue);
    state.hoodLow = table->GetNumber("Hood Bottom Position", ShooterConstants::hoodBottom);
    state.hoodHigh = table->GetNumber("Hood Top Position", ShooterConstants::hoodTop);


    if (liftTable->GetNumber("Lift Main Encoder Value", 0) > ShooterConstants::turretRotateLiftThreshold) {
        state.turretState = TurretState::TURRET_HOME_LEFT;
        limelightTrack(false);
        state.hoodState = HoodState::HOOD_DOWN;
        state.flywheelState = FlywheelState::FLYWHEEL_DISABLE;
    }

    if (state.turretState == TurretState::TURRET_TRACK && state.lastTurretState != TurretState::TURRET_TRACK){
        limelightTrack(true);
    }
    else if (state.turretState != TurretState::TURRET_TRACK && state.lastTurretState == TurretState::TURRET_TRACK){
        limelightTrack(false);
    }
    state.lastTurretState = state.turretState;

    table->PutNumber("Hood degrees", hoodEncoder.GetPosition());
    table->PutNumber("Turret pos", turretEncoder.GetPosition());

    table->PutNumber("Turret target", state.turretTarget);
    table->PutNumber("Turret Desired", state.turretDesired);

    table->PutNumber("left stick x", state.leftStickX);

    table->PutNumber("flywheel power", state.flywheelHigh);
    table->PutNumber("hood high", state.hoodHigh);

    table->PutNumber("Turret State", state.turretState);

    table->PutNumber("LoBF Zoom", state.LoBFZoom);

    state.hoodC_1x = table->GetNumber("Hood Y Int 1X", ShooterConstants::cHood_1x);
    state.powerC_1x = table->GetNumber("Power Y Int 1X", ShooterConstants::cPower_1x);

    state.hoodC_2x = table->GetNumber("Hood Y Int 2X", ShooterConstants::cHood_2x);
    state.powerC_2x = table->GetNumber("Power Y Int 2X", ShooterConstants::cPower_2x);

    state.pipeline = limeTable->GetNumber("pipeline", 0);
}

//0 is close (1x zoom), 1 is far (2x zoom), 2 is auto (1x zoom)
void Shooter::setLimelight(int pipeline){
    limeTable->PutNumber("pipeline", pipeline);
    state.pipeline = pipeline;
}

void Shooter::setPIDProfile(int slotID){
    flywheel_lead.SelectProfileSlot(slotID, 0);
}

void Shooter::assignOutputs()
{ 

    /*//////////////////////////////////////
    // Turret                             //  
    //////////////////////////////////////*/
    state.tx = limeTable->GetNumber("tx", 0.0);
    state.tv = limeTable->GetNumber("tv", 0.0);

    //MANUAL
    state.turretTarget = 0;
    if (state.turretState == TurretState::TURRET_MANUAL) {
        state.turretOutput = std::pow(state.leftStickX, 3) * ShooterConstants::TURRET_SPEED_MULTIPLIER;

        // Minimum power deadband
        if (std::abs(state.leftStickX) < ShooterConstants::pDeadband) {
            state.turretOutput = 0;
        }
        turret.Set(state.turretOutput);
    }
    //HOME
    else if(state.turretState == TurretState::TURRET_HOME_MID){
        if(fabs(turretEncoder.GetPosition() - ShooterConstants::homePositionMid) < 2){
            state.turretState = TurretState::TURRET_DISABLE;
        }
        else {
            state.turretTarget = ShooterConstants::homePositionMid;
            turretPidController.SetReference(state.turretTarget, rev::ControlType::kSmartMotion);  
        }
    }
    else if(state.turretState == TurretState::TURRET_HOME_LEFT){
        if(fabs(turretEncoder.GetPosition() - ShooterConstants::homePositionLeft) < 2){
            state.turretState = TurretState::TURRET_DISABLE;
        }
        else {
            state.turretTarget = ShooterConstants::homePositionLeft;
            turretPidController.SetReference(state.turretTarget, rev::ControlType::kSmartMotion);  
        }
    }
    else if(state.turretState == TurretState::TURRET_HOME_RIGHT){
        if(fabs(turretEncoder.GetPosition() - ShooterConstants::homePositionRight) < 2){
            state.turretState = TurretState::TURRET_DISABLE;
        }
        else {
            state.turretTarget = ShooterConstants::homePositionRight;
            turretPidController.SetReference(state.turretTarget, rev::ControlType::kSmartMotion);  
        }
    }
    //PRIMED
    else if (state.turretState == TurretState::TURRET_TRACK){
        state.turretTarget = state.turretDesired;
        turretPidController.SetReference(state.turretTarget, rev::ControlType::kSmartMotion);
    }
    //DISABLED
    else{
        state.turretOutput = 0;
        turret.Set(state.turretOutput);
    }
    table->PutNumber("Turret Error", state.turretTarget - turretEncoder.GetPosition());

    /*//////////////////////////////////////
    // Flywheel                           //  
    //////////////////////////////////////*/
    if (state.flywheelState == FlywheelState::FLYWHEEL_DISABLE){
        state.flywheelTarget = 0;
    }
    else if (state.flywheelState == FlywheelState::FLYWHEEL_DEFAULT){
        state.flywheelTarget = state.flywheelLow;
    }
    else if(state.flywheelState == FlywheelState::FLYWHEEL_TRACK){
        if (state.pipeline == 1) {
            state.flywheelTarget = ShooterConstants::aPower_2x *(state.distanceToHub * state.distanceToHub) + ShooterConstants::bPower_2x * state.distanceToHub + state.powerC_2x ;
        }
        else {
            state.flywheelTarget = ShooterConstants::aPower_1x *(state.distanceToHub * state.distanceToHub) + ShooterConstants::bPower_1x * state.distanceToHub + state.powerC_1x ;
        }
    }
    else if(state.flywheelState == FlywheelState::FLYWHEEL_POOP){
        state.flywheelTarget = ShooterConstants::flywheelPoopValue;
    }
    
    if (state.flywheelTarget > 0.6)
        state.flywheelTarget = 0.6;
    
    double rpm = state.flywheelTarget * ShooterConstants::falconMaxRPM;
    double rp100ms = rpm / 600.0;
    double ticsp100ms = rp100ms * ShooterConstants::falconGearRatio * ShooterConstants::ticsPerRev;

    table->PutNumber("FlyWheel State", state.flywheelState);
    table->PutNumber("FlyWheel Target", ticsp100ms);
    table->PutNumber("Flywheel Speed", flywheel_lead.GetSelectedSensorVelocity());
    
    flywheel_lead.Set(ControlMode::Velocity, ticsp100ms);

    /*//////////////////////////////////////
    // Hood                               //  
    //////////////////////////////////////*/
    if(state.hoodState == HoodState::HOOD_DOWN){
        state.hoodTarget = state.hoodLow;
    }
    else if(state.hoodState == HoodState::HOOD_TRACK){
        if (state.pipeline == 1) {
            state.hoodTarget = ShooterConstants::aHood_2x * (state.distanceToHub * state.distanceToHub) + ShooterConstants::bHood_2x * state.distanceToHub + state.hoodC_2x;
            state.LoBFZoom = 2;
        }
        else {
            state.hoodTarget = ShooterConstants::aHood_1x * (state.distanceToHub * state.distanceToHub) + ShooterConstants::bHood_1x * state.distanceToHub + state.hoodC_1x;
            state.LoBFZoom = 1;
        }
    }
    else if(state.hoodState == HoodState::HOOD_POOP){
        state.hoodTarget = ShooterConstants::hoodPoop;
    }
    if (state.hoodTarget < 0)
        state.hoodTarget = 0;
    hoodPidController.SetReference(state.hoodTarget, rev::ControlType::kSmartMotion);
}

void Shooter::assignTurret(double tg) {
    state.turretDesired = tg;
}
