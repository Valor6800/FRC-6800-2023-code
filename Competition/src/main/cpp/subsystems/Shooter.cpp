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

#define P1_POWER_A 0.0553
#define P1_POWER_B -0.101
#define P1_POWER_C 0.471

#define P1_HOOD_A 6.29
#define P1_HOOD_B -7.48
#define P1_HOOD_C 0.993

#define P2_POWER_A 0.165
#define P2_POWER_B -0.432
#define P2_POWER_C 0.704

#define P2_HOOD_A 18.7
#define P2_HOOD_B -41.2
#define P2_HOOD_C 24.6

#define FLYWHEEL_SPD_PRIME 0.46
#define FLYWHEEL_SPD_AUTO 0.405
#define FLYWHEEL_SPD_DEFAULT 0.44
#define FLYWHEEL_SPD_POOP 0.3
#define FLYWHEEL_SPD_LAUNCHPAD 0.455

#define HOOD_POS_BOT 0
#define HOOD_POS_TOP 5
#define HOOD_POS_POOP 0
#define HOOD_POS_LAUNCHPAD 17.6
#define HOOD_POS_MAX 22

#define TURRET_POS_LEFT_LIMIT 190
#define TURRET_POS_RIGHT_LIMIT -7
#define TURRET_POS_LEFT 180
#define TURRET_POS_RIGHT 0
#define TURRET_POS_MID 90

#define TURRET_DEADBAND 0.08
#define TURRET_SPEED 0.75
#define TURRET_LIFT_THRESHOLD 20000

#define LIMELIGHT_KP 0.3/25.445*1.25
#define LIMELIGHT_ANGLE 50
#define LIMELIGHT_HEIGHT 0.6075
#define HUB_HEIGHT 2.64

Shooter::Shooter() : ValorSubsystem(),
                    shooterController{CANIDs::FLYWHEEL_LEAD, Coast, false},
                    turretController{CANIDs::TURRET, rev::CANSparkMax::IdleMode::kBrake, true},
                    hoodController{CANIDs::HOOD, rev::CANSparkMax::IdleMode::kBrake, true},
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
    table->PutNumber("Flywheel Primed Value", FLYWHEEL_SPD_PRIME);
    table->PutNumber("Flywheel Default Value", FLYWHEEL_SPD_DEFAULT);
    table->PutNumber("Hood Top Position", HOOD_POS_TOP);
    table->PutNumber("Hood Bottom Position", HOOD_POS_BOT);
    
    table->PutNumber("Hood Y Int 1X", P1_HOOD_C);
    table->PutNumber("Power Y Int 1X", P1_POWER_C);

    PIDF shooterPIDF;
    shooterPIDF.error = 0;
    shooterPIDF.velocity = 20000;
    shooterPIDF.acceleration = shooterPIDF.velocity * 1;
    shooterController.setPIDF(0, shooterPIDF);
    shooterController.preventBackwards();

    PIDF turretPIDF;
    turretPIDF.P = 1e-5;
    turretPIDF.F = 0.0001;
    turretPIDF.velocity = 10000;
    turretPIDF.acceleration = turretPIDF.velocity * 2;
    turretController.setPIDF(0, turretPIDF);
    turretController.setLimits(TURRET_POS_RIGHT_LIMIT, TURRET_POS_LEFT_LIMIT);
    turretController.setConversion(360.0 / 60);

    PIDF hoodPIDF;
    hoodPIDF.P = 5e-5;
    hoodPIDF.F = 0.000156 * 0.5;
    hoodPIDF.velocity = 10000;
    hoodPIDF.acceleration = hoodPIDF.velocity * 4;
    hoodController.setPIDF(0, hoodPIDF);
    hoodController.setLimits(HOOD_POS_BOT, HOOD_POS_TOP);
    hoodController.setConversion(360.0 / 454.17);

    state.pipeline = 0;
    state.LoBFZoom = 1;
    
    resetState();

    limelightTrack(true);
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

void Shooter::setControllers(ValorGamepad *controllerO, ValorGamepad *controllerD)
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
    
    //Turret
    if (operatorController->leftStickXActive()) {
        state.turretState = TurretState::TURRET_MANUAL; // Operator control
    }
    else if (operatorController->GetYButton() || driverController->GetBButton()) {
        state.turretState = TurretState::TURRET_HOME_MID;
    }
    else if (!table->GetBoolean("Pit Disable", false)){
        state.turretState = TurretState::TURRET_TRACK;
    }

    //Hood & Flywheel
    
    if(operatorController->GetAButtonPressed()){
        state.hoodState = HoodState::HOOD_DOWN; // Low position
        state.flywheelState = FlywheelState::FLYWHEEL_DEFAULT; // Lower speed
    }
    else if(operatorController->GetXButtonPressed()){
        state.hoodState = HoodState::HOOD_POOP;
        state.flywheelState = FlywheelState::FLYWHEEL_POOP;
    }
    else if (operatorController->GetBButtonPressed()){
        state.hoodState = HoodState::HOOD_TRACK; // High position
        state.flywheelState = FlywheelState::FLYWHEEL_TRACK; // Higher speed
    }

    state.trackCorner = false;//state.rightBumper ? true : false;
}

void Shooter::analyzeDashboard()
{
    // Limelight Distance calculations
    // Only update if a target is visible. Value is sticky if no target is present
    if (limeTable->GetNumber("tv", 0.0) == 1.0) {
        double angle = limeTable->GetNumber("ty", 0.0) + LIMELIGHT_ANGLE;
        double deltaH = HUB_HEIGHT - LIMELIGHT_HEIGHT;
        double xDist = deltaH / tan(angle * MathConstants::toRadians);
        state.distanceToHub = xDist;

        if (state.distanceToHub < 1.0)
            state.distanceToHub = 1.0;

        table->PutNumber("x distance to hub", xDist);
    }

    if (table->GetBoolean("Pit Disable", false)){
        state.turretState = TurretState::TURRET_DISABLE;
        state.hoodState = HoodState::HOOD_DOWN;
        state.flywheelState = FlywheelState::FLYWHEEL_DISABLE;
    }

    // Turret homing and zeroing
    if (table->GetBoolean("Zero Turret", false)) {
        turretController.reset();
    }

    // Hood zeroing
    if (table->GetBoolean("Zero Hood", false)){
        hoodController.reset();
    }

    //slider
    state.flywheelLow = table->GetNumber("Flywheel Default Value", FLYWHEEL_SPD_DEFAULT);
    state.flywheelHigh = table->GetNumber("Flywheel Primed Value", FLYWHEEL_SPD_PRIME);
    state.hoodLow = table->GetNumber("Hood Bottom Position", HOOD_POS_BOT);
    state.hoodHigh = table->GetNumber("Hood Top Position", HOOD_POS_TOP);


    if (liftTable->GetNumber("Lift Main Encoder Value", 0) > TURRET_LIFT_THRESHOLD) {
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

    table->PutNumber("Hood degrees", hoodController.getPosition());
    table->PutNumber("Turret pos", turretController.getPosition());

    table->PutNumber("Turret target", state.turretTarget);
    table->PutNumber("Turret Desired", state.turretDesired);

    table->PutNumber("Turret Output", state.turretOutput);

    table->PutNumber("flywheel power", state.flywheelHigh);
    table->PutNumber("hood high", state.hoodHigh);

    table->PutNumber("Turret State", state.turretState);

    table->PutNumber("LoBF Zoom", state.LoBFZoom);

    state.hoodC_1x = table->GetNumber("Hood Y Int 1X", P1_HOOD_C);
    state.powerC_1x = table->GetNumber("Power Y Int 1X", P1_POWER_C);

    state.hoodC_2x = table->GetNumber("Hood Y Int 2X", P2_HOOD_C);
    state.powerC_2x = table->GetNumber("Power Y Int 2X", P2_POWER_C);

    state.pipeline = limeTable->GetNumber("pipeline", 0);
}

//0 is close (1x zoom), 1 is far (2x zoom), 2 is auto (1x zoom)
void Shooter::setLimelight(int pipeline){
    limeTable->PutNumber("pipeline", pipeline);
    state.pipeline = pipeline;
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
        state.turretOutput = operatorController->leftStickX(3) * TURRET_SPEED;
        if( (turretController.getPosition() > 160 && state.turretOutput > TURRET_DEADBAND) ||
         (turretController.getPosition() < 20 && state.turretOutput < -1 * TURRET_DEADBAND) ){
            state.turretOutput = 0;
        }
        turretController.setSpeed(state.turretOutput);
    }
    //HOME
    else if(state.turretState == TurretState::TURRET_HOME_MID){
        if(fabs(turretController.getPosition() - TURRET_POS_MID) < 2){
            state.turretState = TurretState::TURRET_DISABLE;
        }
        else {
            state.turretTarget = TURRET_POS_MID;
            turretController.setPosition(state.turretTarget);  
        }
    }
    else if(state.turretState == TurretState::TURRET_HOME_LEFT){
        if(fabs(turretController.getPosition() - TURRET_POS_LEFT) < 2){
            state.turretState = TurretState::TURRET_DISABLE;
        }
        else {
            state.turretTarget = TURRET_POS_LEFT;
            turretController.setPosition(state.turretTarget);  
        }
    }
    else if(state.turretState == TurretState::TURRET_HOME_RIGHT){
        if(fabs(turretController.getPosition() - TURRET_POS_RIGHT) < 2){
            state.turretState = TurretState::TURRET_DISABLE;
        }
        else {
            state.turretTarget = TURRET_POS_RIGHT;
            turretController.setPosition(state.turretTarget);  
        }
    }
    //PRIMED
    else if (state.turretState == TurretState::TURRET_TRACK){
        state.turretTarget = state.turretDesired;
        turretController.setPosition(state.turretTarget);
    }
    //DISABLED
    else{
        state.turretOutput = 0;
        turretController.setSpeed(state.turretOutput);
    }
    table->PutNumber("Turret Error", state.turretTarget - turretController.getPosition());

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
            state.flywheelTarget = P2_POWER_A * pow(state.distanceToHub, 2) + P2_POWER_B * state.distanceToHub + state.powerC_2x;
        }
        else {
            state.flywheelTarget = P1_POWER_A * pow(state.distanceToHub, 2) + P1_POWER_B * state.distanceToHub + state.powerC_1x;
        }
    }
    else if(state.flywheelState == FlywheelState::FLYWHEEL_POOP){
        state.flywheelTarget = FLYWHEEL_SPD_POOP;
    }
    
    if (state.flywheelTarget > 0.6)
        state.flywheelTarget = 0.6;
    
    double rpm = state.flywheelTarget * ShooterConstants::falconMaxRPM;
    double rp100ms = rpm / 600.0;
    double ticsp100ms = rp100ms * ShooterConstants::falconGearRatio * ShooterConstants::ticsPerRev;

    table->PutNumber("FlyWheel State", state.flywheelState);
    table->PutNumber("FlyWheel Target", ticsp100ms);
    table->PutNumber("Flywheel Speed", shooterController.getSpeed());
    
    shooterController.setSpeed(ticsp100ms);

    /*//////////////////////////////////////
    // Hood                               //  
    //////////////////////////////////////*/
    if(state.hoodState == HoodState::HOOD_DOWN){
        state.hoodTarget = state.hoodLow;
    }
    else if(state.hoodState == HoodState::HOOD_TRACK){
        if (state.pipeline == 1) {
            state.hoodTarget = P2_HOOD_A * pow(state.distanceToHub, 2) + P2_HOOD_B * state.distanceToHub + state.hoodC_2x;
            state.LoBFZoom = 2;
        }
        else {
            state.hoodTarget = P1_HOOD_A * pow(state.distanceToHub, 2) + P1_HOOD_B * state.distanceToHub + state.hoodC_1x;
            state.LoBFZoom = 1;
        }
    }
    else if(state.hoodState == HoodState::HOOD_POOP){
        state.hoodTarget = HOOD_POOP;
    }
    if (state.hoodTarget < 0)
        state.hoodTarget = 0;
    hoodController.setPosition(state.hoodTarget);
}

void Shooter::assignTurret(double tg) {
    state.turretDesired = tg;
}
