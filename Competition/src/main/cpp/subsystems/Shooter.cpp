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
    turret.SetInverted(false);

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
}

void Shooter::resetState(){
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
    state.leftStickX = -operatorController->GetLeftX();

    double angle = limeTable->GetNumber("ty", 0.0) + ShooterConstants::limelightAngle;
    double deltaH = ShooterConstants::hubHeight - ShooterConstants::limelightHeight;
    double xDist = deltaH / tan(angle * MathConstants::toRadians);

    state.distanceToHub = xDist;

    table->PutNumber("x distance to hub", xDist);

    if(table->GetBoolean("Home Turret", false)){
        state.turretState = TurretState::TURRET_HOME;
    }

    if (table->GetBoolean("Zero Turret", false)){
        turretEncoder.SetPosition(0);
    }

    if (table->GetBoolean("Zero Hood", false)){
        state.hoodState = HoodState::HOOD_RESET;
    }

    //slider
    state.flywheelLow = table->GetNumber("Flywheel Default Value", ShooterConstants::flywheelDefaultValue);
    state.hoodLow = table->GetNumber("Hood Bottom Position", ShooterConstants::hoodBottom);

    if(!table->GetBoolean("Use line of best fit", false)){
        state.hoodHigh = table->GetNumber("Hood Top Position", ShooterConstants::hoodTop);
        state.flywheelHigh = table->GetNumber("Flywheel Default Value", ShooterConstants::flywheelPrimedValue);
    }

    if (state.turretState == TurretState::TURRET_PRIME && state.lastTurretState != TurretState::TURRET_PRIME){
        limelightTrack(true);
    }
    else if (state.turretState != TurretState::TURRET_PRIME && state.lastTurretState == TurretState::TURRET_PRIME){
        limelightTrack(false);
    }
    state.lastTurretState = state.turretState;

    table->PutNumber("Hood degrees", hoodEncoder.GetPosition());
    table->PutNumber("Turret pos", turretEncoder.GetPosition());

    table->PutNumber("left stick x", state.leftStickX);

    table->PutNumber("flywheel power", state.flywheelHigh);
    table->PutNumber("hood high", state.hoodHigh);
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
        int sign = 1;//state.leftStickX >= 0 ? 1 : -1;
        state.turretOutput = sign * std::pow(state.leftStickX, 3) * ShooterConstants::TURRET_SPEED_MULTIPLIER;

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

        if(table->GetBoolean("Use line of best fit", false)){
            state.flywheelHigh = ShooterConstants::aPower * state.distanceToHub + ShooterConstants::bPower;
        }
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

        double error  = tics - turretEncoder.GetPosition(); //might need to invert this
        state.turretTarget = error * ShooterConstants::turretKP; //need to set value
        useSmartMotion = true;
    }

    //DISABLED
    else{
        state.turretOutput = 0;
    }

    if (useSmartMotion){
        //turretPidController.SetReference(state.turretTarget, rev::ControlType::kSmartMotion);
        turret.Set(0);

    }
    else{
        //turretPidController.SetReference(state.turretOutput, rev::ControlType::kDutyCycle);
        turret.Set(state.turretOutput);
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
    else if (state.hoodState == HoodState::HOOD_RESET){
        resetHood();
    }
    else if(state.hoodState == HoodState::HOOD_PRIME){
        if(table->GetBoolean("Use line of best fit", false)){
            state.hoodHigh = ShooterConstants::aHood * state.distanceToHub + ShooterConstants::bHood;
        }
        state.hoodTarget = state.hoodHigh;
    }
    hoodPidController.SetReference(state.hoodTarget, rev::ControlType::kSmartMotion);
}

void Shooter::resetHood(){
    while(hood.GetOutputCurrent() <= 10){
        hood.Set(-1);
    }
    hood.Set(0);
    hoodEncoder.SetPosition(0);
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
    while (originalTarget < ShooterConstants::turretLimitLeft){
        originalTarget += realTicsPerRev;
    }
    while(originalTarget > ShooterConstants::turretLimitRight){
        originalTarget -= realTicsPerRev;
    }
    return originalTarget;
}
