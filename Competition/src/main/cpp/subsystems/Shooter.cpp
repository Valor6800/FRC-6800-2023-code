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
                    operatorController(NULL)
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
    
    table->PutBoolean("Home Turret", false);
    table->PutBoolean("Zero Hood", false);
    table->PutNumber("Flywheel Primed Value", ShooterConstants::flywheelPrimedValue);
    table->PutNumber("Flywheel Default Value", ShooterConstants::flywheelDefaultValue);
    table->PutNumber("Hood Top Position", ShooterConstants::hoodTop);
    table->PutNumber("Hood Bottom Position", ShooterConstants::hoodBottom);
    
    table->PutBoolean("Use line of best fit", false);

    flywheel_lead.ConfigFactoryDefault();
    flywheel_lead.ConfigAllowableClosedloopError(0, 0);
    flywheel_lead.Config_IntegralZone(0, 0);
    flywheel_lead.Config_kF(0, ShooterConstants::flywheelKFF);
    flywheel_lead.Config_kD(0, ShooterConstants::flywheelKD);
    flywheel_lead.Config_kI(0, ShooterConstants::flywheelKI);
    flywheel_lead.Config_kP(0, ShooterConstants::flywheelKP);
    flywheel_lead.ConfigMotionAcceleration(ShooterConstants::flywheelMaxAccel);
    flywheel_lead.ConfigMotionCruiseVelocity(ShooterConstants::flywheelCruiseVelo);
    flywheel_lead.SetNeutralMode(NeutralMode::Coast);

    flywheel_lead.SetInverted(false);

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
    
    resetState();
    resetEncoder();

    limelightTrack(false);
}

void Shooter::resetState(){
    state.turretState = TurretState::TURRET_DISABLE;
    //hack solution
    state.lastTurretState = TurretState::TURRET_TRACK;
    state.hoodState = HoodState::HOOD_DOWN;
    state.flywheelState = FlywheelState::FLYWHEEL_DEFAULT;
    state.trackCorner = false;

    state.turretOutput = 0;
    state.turretTarget = 0;

    state.flywheelTarget = 0;
    state.hoodTarget = 0;
    state.distanceToHub = 3;
    state.currentBall = 0;
}

void Shooter::resetEncoder(){
    turretEncoder.SetPosition(0);
    hoodEncoder.SetPosition(0);
}

void Shooter::setController(frc::XboxController *controller)
{
    operatorController = controller;
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
    state.xButton = operatorController->GetXButton();
    state.bButton = operatorController->GetBButtonPressed();
    
    //Turret
    if (std::abs(state.leftStickX) > ShooterConstants::kDeadband) {
        state.turretState = TurretState::TURRET_MANUAL; // Operator control
    }
    else if(state.rightBumper){
       state.turretState = TurretState::TURRET_TRACK; // Use limelight
    }
    else{
        state.turretState = TurretState::TURRET_DISABLE; // Not moving
    }

    //Hood
    if(state.bButton){
        state.hoodState = HoodState::HOOD_TRACK; // High position
    }
    else if(state.aButton){
        state.hoodState = HoodState::HOOD_DOWN; // Low position
    }

    //Flywheel
    if(state.bButton){
        state.flywheelState = FlywheelState::FLYWHEEL_TRACK; // Higher speed
    }
    else if (state.aButton){
        state.flywheelState = FlywheelState::FLYWHEEL_DEFAULT; // Lower speed
    }

    state.trackCorner = false;//state.rightBumper ? true : false;
    if (state.yButton) {
        state.turretState = TurretState::TURRET_HOME_MID;
    }
    // else if (state.xButton) {
    //     state.turretState = TurretState::TURRET_HOME_LEFT;
    // }
    // else if (state.bButton) {
    //     state.turretState = TurretState::TURRET_HOME_RIGHT;
    // }
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

    // Turret homing and zeroing
    if (table->GetBoolean("Zero Turret", false)) {
        turretEncoder.SetPosition(0);
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

    // Logic should exist here in case we need to turn off limelight for auto
    // Set the button to false and default speeds will be used in auto
    if (table->GetBoolean("Use line of best fit", false)) {
        if (state.flywheelState == FlywheelState::FLYWHEEL_PRIME)
            state.flywheelState = FlywheelState::FLYWHEEL_TRACK;
        if (state.hoodState == HoodState::HOOD_UP)
            state.hoodState = HoodState::HOOD_TRACK;
        //limeTable->PutNumber("pipeline", 1);
    }
    else{
        limeTable->PutNumber("pipeline", 0);
    }

    if (liftTable->GetNumber("Lift Main Encoder Value", 0) > ShooterConstants::turretRotateLiftThreshold) {
        state.turretState = TurretState::TURRET_HOME_LEFT;
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

    table->PutNumber("left stick x", state.leftStickX);

    table->PutNumber("flywheel power", state.flywheelHigh);
    table->PutNumber("hood high", state.hoodHigh);

    table->PutNumber("Turret State", state.turretState);
}

void Shooter::assignOutputs()
{ 

    /*//////////////////////////////////////
    // Turret                             //  
    //////////////////////////////////////*/

    //MANUAL
    state.turretTarget = 0;
    if (state.turretState == TurretState::TURRET_MANUAL) {
        state.turretOutput = std::pow(state.leftStickX, 3) * ShooterConstants::TURRET_SPEED_MULTIPLIER;

        // Minimum power deadband
        if (std::abs(state.leftStickX) < ShooterConstants::pDeadband) {
            state.turretOutput = 0;
        }
        // Stop deadband
        // else if (std::abs(state.turretOutput) < ShooterConstants::pSoftDeadband) {
        //     int direction = 1;
        //     if (state.turretOutput < 0) direction = -1;
        //     state.turretOutput = ShooterConstants::pSoftDeadband * direction;
        // }
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
        float tx = limeTable->GetNumber("tx", 0.0);
        float tv = limeTable->GetNumber("tv", 0.0);
        state.turretTarget = (-tx * tv * 0.75) + turretEncoder.GetPosition();
        std::clamp(state.turretTarget, ShooterConstants::turretLimitRight, ShooterConstants::turretLimitLeft);
        turretPidController.SetReference(state.turretTarget, rev::ControlType::kSmartMotion);
        // state.turretOutput = tv * -tx * ShooterConstants::limelightTurnKP;
        // turret.Set(state.turretOutput);
    }
    //DEFAULT
    else if (state.turretState == TurretState::TURRET_AUTO){
        //Odometry tracking
        frc::Pose2d currentPose = odom->getPose_m();
        double targetX = ShooterConstants::hubX;
        double targetY = ShooterConstants::hubY;
        if (state.trackCorner){
            targetX = ShooterConstants::cornerX;
            targetY = ShooterConstants::cornerY;
        }

        double tics = getTargetTics(currentPose.X().to<double>(), currentPose.Y().to<double>(), currentPose.Rotation().Radians().to<double>(),
                                        targetX, targetY
                                        , ShooterConstants::ticsPerRev, ShooterConstants::turretGearRatio);

        double error  = tics - turretEncoder.GetPosition(); //might need to invert this
        state.turretTarget = error * ShooterConstants::turretKP; //need to set value
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
        state.flywheelTarget = ShooterConstants::aPower * state.distanceToHub + ShooterConstants::bPower;
    }
    else if(state.flywheelState == FlywheelState::FLYWHEEL_AUTO){
        //state.flywheelTarget = ShooterConstants::flywheelAutoValue;
        state.flywheelTarget = ShooterConstants::flywheelSpeeds[state.currentBall];
    }
    else if (state.flywheelState == FlywheelState::FLYWHEEL_PRIME){
        state.flywheelTarget = state.flywheelHigh;
    }
    
    double rpm = state.flywheelTarget * ShooterConstants::falconMaxRPM;
    double rp100ms = rpm / 600.0;
    double ticsp100ms = rp100ms * ShooterConstants::falconGearRatio * ShooterConstants::ticsPerRev;

    table->PutNumber("FlyWheel State", state.flywheelState);
    table->PutNumber("FlyWheel Target", state.flywheelTarget);

    flywheel_lead.Set(ControlMode::Velocity, ticsp100ms);

    /*//////////////////////////////////////
    // Hood                               //  
    //////////////////////////////////////*/

    if(state.hoodState == HoodState::HOOD_DOWN){
        state.hoodTarget = state.hoodLow;
    }
    else if(state.hoodState == HoodState::HOOD_TRACK){
        state.hoodTarget = ShooterConstants::aHood * state.distanceToHub + ShooterConstants::bHood;
    }
    else if(state.hoodState = HoodState::HOOD_AUTO){
        //state.hoodTarget = ShooterConstants::hoodAuto;
        state.hoodTarget = ShooterConstants::hoodAngles[state.currentBall];
    }
    else if(state.hoodState == HoodState::HOOD_UP){
        state.hoodTarget = state.hoodHigh;
    }
    hoodPidController.SetReference(state.hoodTarget, rev::ControlType::kSmartMotion);
}
//testing if git is broken
double Shooter::getTargetTics(double x, 
double y,
double theta,
double hubX, 
double hubY,
double ticsPerRev,
double gearRatio){

    double deltaX = hubX - x;
    double deltaY = hubY - y;

    double targetThetaRad = atan2(deltaY, deltaX);
    double relativeAngle = targetThetaRad - theta;
    double rot = relativeAngle / (2 * M_PI);
    double targetTics = rot * ticsPerRev * gearRatio;

    while (targetTics > .5 * ticsPerRev * gearRatio){
        targetTics -= ticsPerRev * gearRatio;
    }
    while (targetTics < -.5 * ticsPerRev * gearRatio){
        targetTics += ticsPerRev * gearRatio;
    }
    return convertTargetTics(targetTics, ticsPerRev * gearRatio);
}

double Shooter::convertTargetTics(double originalTarget, double realTicsPerRev){
    //originalTarget will always be between limitLeft to limitRight
    while (originalTarget < ShooterConstants::turretLimitLeft){
        originalTarget += realTicsPerRev;
    }
    while(originalTarget > ShooterConstants::turretLimitRight){
        originalTarget -= realTicsPerRev;
    }
    return originalTarget;
}
