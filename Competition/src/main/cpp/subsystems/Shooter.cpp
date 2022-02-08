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
    initTable("Shooter");
    
    table->PutBoolean("Home Turret", false);
    table->PutNumber("Flywheel Primed Value", ShooterConstants::flywheelPrimedValue);
    table->PutNumber("Flywheel Default Value", ShooterConstants::flywheelDefaultValue);
    table->PutNumber("Hood Top Position", ShooterConstants::hoodTop);
    table->PutNumber("Hood Bottom Position", ShooterConstants::hoodBottom);


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
    hood.RestoreFactoryDefaults();

    turret.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    hood.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);

    turret.SetInverted(false);

    hood.SetInverted(false);

    //potentially need to flip left and right
    turret.EnableSoftLimit(rev::CANSparkMax::SoftLimitDirection::kForward, true);
    turret.SetSoftLimit(rev::CANSparkMax::SoftLimitDirection::kForward, ShooterConstants::limitLeft);

    turret.EnableSoftLimit(rev::CANSparkMax::SoftLimitDirection::kReverse, true);
    turret.SetSoftLimit(rev::CANSparkMax::SoftLimitDirection::kReverse, ShooterConstants::limitRight);

    hood.EnableSoftLimit(rev::CANSparkMax::SoftLimitDirection::kForward, true);
    hood.SetSoftLimit(rev::CANSparkMax::SoftLimitDirection::kForward, ShooterConstants::limitTop);

    hood.EnableSoftLimit(rev::CANSparkMax::SoftLimitDirection::kReverse, true);
    hood.SetSoftLimit(rev::CANSparkMax::SoftLimitDirection::kReverse, ShooterConstants::limitBottom);

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

    resetState();
}

void Shooter::resetState(){
    resetEncoder();
    state.turretState = TurretState::TURRET_DEFAULT;
    state.lastTurretState = TurretState::TURRET_DEFAULT;
    state.hoodState = HoodState::HOOD_DISABLE;
    state.flywheelState = FlywheelState::FLYWHEEL_DEFAULT;
    state.trackCorner = false;
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

    state.leftStickX = operatorController->GetLeftX();
    state.startButton = operatorController->GetStartButtonPressed();
    state.backButton = operatorController->GetBackButtonPressed(); 
    state.rightBumper = operatorController->GetRightBumper();
    
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

    state.trackCorner = state.rightBumper ? true : false;
}

void Shooter::analyzeDashboard()
{
    if(table->GetBoolean("Home Turret", false)){
        state.turretState = TurretState::TURRET_HOME;
    }

    if (table->GetBoolean("Zero Turret", false)){
        turretEncoder.SetPosition(0);
    }

    //slider
    state.flywheelHigh = table->GetNumber("Flywheel Primed Value", ShooterConstants::flywheelPrimedValue);
    state.flywheelLow = table->GetNumber("Flywheel Default Value", ShooterConstants::flywheelDefaultValue);

    state.hoodLow = table->GetNumber("Hood low position", ShooterConstants::hoodBottom);
    state.hoodHigh = table->GetNumber("Hood high position", ShooterConstants::hoodTop);

    // if (state.turretState == TurretState::TURRET_PRIME && state.lastTurretState != TurretState::TURRET_PRIME){
    //     limelightTrack(true);
    // }
    // else if (state.turretState != TurretState::TURRET_PRIME && state.lastTurretState == TurretState::TURRET_PRIME){
    //     limelightTrack(false);
    // }
    limelightTrack(TurretState::TURRET_PRIME == state.turretState);
    state.lastTurretState = state.turretState;



    table->PutString("state", std::to_string(state.turretState));
    table->PutString("last state", std::to_string(state.lastTurretState));

}

void Shooter::assignOutputs()
{   
    state.turretOutput = 0;
    state.turretTarget = 0;

    state.flywheelTarget = 0;
    state.hoodTarget = 0;

    bool useSmartMotion = false;

    //MANUAL
    if (state.turretState == TurretState::TURRET_MANUAL) {
        int sign = state.leftStickX >= 0 ? 1 : -1;
        state.turretOutput = sign * std::pow(state.leftStickX, 2) * ShooterConstants::TURRET_SPEED_MULTIPLIER;

        // Minimum power deadband
        if (std::abs(state.leftStickX) < ShooterConstants::pDeadband) {
            state.turretOutput = 0;
        }
        // Stop deadband
        else if (std::abs(state.turretOutput) < ShooterConstants::pSoftDeadband) {
            int direction = 1;
            if (state.turretOutput < 0) direction = -1;
            state.turretOutput = ShooterConstants::pSoftDeadband * direction;
        }
    }

    //HOME
    else if(state.turretState == TurretState::TURRET_HOME){
        state.turretTarget = ShooterConstants::homePosition;
        useSmartMotion = true;
    }

    //PRIMED
    else if (state.turretState == TurretState::TURRET_PRIME){
        float tx = limeTable->GetNumber("tx", 0.0);
        float tv = limeTable->GetNumber("tv", 0.0);
        state.turretOutput = tv * -tx * ShooterConstants::limelightTurnKP;
    }

    //DEFAULT
    else if (state.turretState == TurretState::TURRET_DEFAULT){
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

        double error  = tics - turretEncoder.GetPosition();
        state.turretTarget = error * ShooterConstants::turretKP; //need to set value
        
        //useSmartMotion = true; //potential issue, might just want to use pid
    }

    if (useSmartMotion){
        turretPidController.SetReference(state.turretTarget, rev::ControlType::kSmartMotion);
    }
    else{
        turretPidController.SetReference(state.turretOutput, rev::ControlType::kDutyCycle);
    }
    
    if(state.flywheelState == FlywheelState::FLYWHEEL_DISABLE){
        state.flywheelTarget = 0;
    }
    else if(state.flywheelState == FlywheelState::FLYWHEEL_PRIME){
        state.flywheelTarget = state.flywheelHigh;
    }
    else if (state.flywheelState == FlywheelState::FLYWHEEL_DEFAULT){
        state.flywheelTarget = state.flywheelLow;
    }
    
    double rpm = state.flywheelTarget * ShooterConstants::falconMaxRPM;
    double rp100ms = rpm / 600.0;
    double ticsp100ms = rp100ms * ShooterConstants::falconGearRatio * ShooterConstants::ticsPerRev;

    flywheel_lead.Set(ControlMode::Velocity, ticsp100ms);

    if(state.hoodState == HoodState::HOOD_DISABLE){
        state.hoodTarget = state.hoodLow;
    }
    else if(state.hoodState == HoodState::HOOD_PRIME){
        state.hoodTarget = state.hoodHigh;
    }
    hoodPidController.SetReference(state.hoodTarget, rev::ControlType::kSmartMotion);
}

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
    while (originalTarget < ShooterConstants::limitLeft){
        originalTarget += realTicsPerRev;
    }
    while(originalTarget > ShooterConstants::limitRight){
        originalTarget -= realTicsPerRev;
    }
    return originalTarget;
}