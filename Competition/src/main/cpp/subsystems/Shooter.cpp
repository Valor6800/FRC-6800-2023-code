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

void Shooter::setDrivetrain(Drivetrain *dt){
    odom = dt;
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

    hood.EnableSoftLimit(rev::CANSparkMax::SoftLimitDirection::kForward, true);
    hood.SetSoftLimit(rev::CANSparkMax::SoftLimitDirection::kForward, ShooterConstants::limitTop);

    hood.EnableSoftLimit(rev::CANSparkMax::SoftLimitDirection::kReverse, true);
    hood.SetSoftLimit(rev::CANSparkMax::SoftLimitDirection::kReverse, ShooterConstants::limitBottom);

    flywheelPidController.SetP(ShooterConstants::flywheelKP);
    flywheelPidController.SetI(ShooterConstants::flywheelKI);
    flywheelPidController.SetD(ShooterConstants::flywheelKD);
    flywheelPidController.SetIZone(ShooterConstants::flywheelKIZ);
    flywheelPidController.SetFF(ShooterConstants::flywheelKFF);
    flywheelPidController.SetOutputRange(0, 1);

    flywheelPidController.SetSmartMotionMaxVelocity(ShooterConstants::flywheelMaxV);
    flywheelPidController.SetSmartMotionMinOutputVelocity(ShooterConstants::flywheelMinV);
    flywheelPidController.SetSmartMotionMaxAccel(ShooterConstants::flywheelMaxAccel);
    flywheelPidController.SetSmartMotionAllowedClosedLoopError(ShooterConstants::flywheelAllowedError);

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
    state.turretVelocityTarget = 0;
    state.turretPositionSetpoint = 0;
    bool useSmartMotion = false;

    //DISABLED
    if(state.turretState == TurretState::TURRET_DISABLE){
        state.turretVelocityTarget = 0;
    }

    //MANUAL
    else if (state.turretState == TurretState::TURRET_MANUAL) {
        int sign = state.leftStickX >= 0 ? 1 : -1;
        state.turretVelocityTarget = sign * std::pow(state.leftStickX, 2) * ShooterConstants::TURRET_SPEED_MULTIPLIER;

        // Minimum power deadband
        if (std::abs(state.turretVelocityTarget) < ShooterConstants::pDeadband) {
            state.turretVelocityTarget = 0;
        }
        // Stop deadband
        else if (std::abs(state.turretVelocityTarget) < ShooterConstants::pSoftDeadband) {
            int direction = 1;
            if (state.turretVelocityTarget < 0) direction = -1;
            state.turretVelocityTarget = ShooterConstants::pSoftDeadband * direction;
        }
    }

    //HOME
    else if(state.turretState == TurretState::TURRET_HOME){
        state.turretPositionSetpoint = ShooterConstants::homeFrontPosition;
        useSmartMotion = true;
    }

    //PRIMED
    else if (state.turretState == TurretState::TURRET_PRIME){
        //limelight
        useSmartMotion = true;
    }

    //DEFAULT
    else if (state.turretState == TurretState::TURRET_DEFAULT){
        //Odometry tracking
        frc::Pose2d currentPose = odom->getPose_m();
        state.turretPositionSetpoint = getTargetTics(currentPose.X().to<double>(), currentPose.Y().to<double>(), currentPose.Rotation().Radians().to<double>(),
                                        ShooterConstants::hubX, ShooterConstants::hubY
                                        , ShooterConstants::ticsPerRev, ShooterConstants::gearRatio);
        useSmartMotion = true;
    }

    if (useSmartMotion){
        turretPidController.SetReference(state.turretPositionSetpoint, rev::ControlType::kSmartMotion);
    }
    else{
        turretPidController.SetReference(state.turretVelocityTarget, rev::ControlType::kSmartVelocity);
    }
    
    if(state.flywheelState == FlywheelState::FLYWHEEL_DISABLE){
        state.flywheelTarget = 0;
    }
    else if(state.flywheelTarget == FlywheelState::FLYWHEEL_PRIME){
        //math conditions, leaving 1 for now
        state.flywheelTarget = 1;
    }
    else if (state.flywheelTarget == FlywheelState::FLYWHEEL_DEFAULT){
        state.flywheelTarget = 0.5;
    }
    
    flywheel_follow.Set(state.flywheelTarget);
    flywheel_lead.Set(state.flywheelTarget);

    if(state.hoodState == HoodState::HOOD_DISABLE){
        state.hoodTarget = 0;
    }
    else if(state.hoodState == HoodState::HOOD_PRIME){
        state.hoodTarget = 0.1;
    }

    hood.Set(state.hoodTarget);
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
    return convertTargetTics(targetTics, turretEncoder.GetPosition(), ticsPerRev);
}

double Shooter::convertTargetTics(double originalTarget, double currentTics, double ticsPerRev){
    //current tics will always be between limitLeft to limitRight
    while (originalTarget < ShooterConstants::limitLeft){
        originalTarget += ticsPerRev;
    }
    while(originalTarget > ShooterConstants::limitRight){
        originalTarget -= ticsPerRev;
    }
}

