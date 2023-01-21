/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
#include "subsystems/Elevarm.h"
#include <iostream>


#define ROTATE_GEAR_RATIO 143.73f
#define ROTATE_OUTPUT_DIAMETER 0.1334516f
#define CARRIAGE_GEAR_RATIO 4.0f
#define CARRAIAGE_OUTPUT_DIAMETER 0.0364f
#define CARRIAGE_UPPER_LIMIT 1.05f 
#define CARRIAGE_LOWER_LIMIT 0.07f
#define ROTATE_FORWARD_LIMIT 180.0f
#define ROTATE_REVERSE_LIMIT -180.0f

#define CARRIAGE_K_F 0.05f  
#define CARRIAGE_K_P 0.01f
#define CARRIAGE_K_I 0.0f
#define CARRIAGE_K_D 0.0f
#define CARRIAGE_K_ERROR 0.01f
#define CARRIAGE_K_VEL 0.02f
#define CARRIAGE_K_ACC_MUL 1.0f

#define ROTATE_K_F 0.0000753f
#define ROTATE_K_P 5e-5f
#define ROTATE_K_I 0.0f
#define ROTATE_K_D 0.0f
#define ROTATE_K_ERROR 0.2f
#define ROTATE_K_VEL 0.2f
#define ROTATE_K_ACC_MUL 1.0f

#define PREVIOUS_HEIGHT_DEADBAND 0.01f
#define PREVIOUS_ROTATION_DEADBAND 0.5f

#define X_BUMPER_WIDTH 0.0762f
#define X_HALF_WIDTH 0.254f
#define X_CARRIAGE_OFFSET 0.1778f
#define X_ARM_LENGTH 1.01854f

#define D_CARRIAGE_JOINT_OFFSET 0.0724154f
#define D_CARRIAGE_FLOOR_OFFSET 0.2286f
#define D_CARRIAGE_STOW_Z 0.4f
#define D_CARRIAGE_STOW_X -0.508f

#define Z_INTAKE_OFFSET 0.138f

Elevarm::Elevarm(frc::TimedRobot *_robot) : ValorSubsystem(_robot, "Elevarm"),                        
                            carriageMotors(CANIDs::CARRIAGE_MAIN, rev::CANSparkMax::IdleMode::kBrake, false),
                            armRotateMotor(CANIDs::ARM_ROTATE, rev::CANSparkMax::IdleMode::kBrake, false),
                            manualMaxArmSpeed(1.0),
                            manualMaxCarriageSpeed(1.0)
{
    frc2::CommandScheduler::GetInstance().RegisterSubsystem(this);
    init();
}

Elevarm::~Elevarm()
{
}

void Elevarm::resetState()
{
    futureState.pieceState = ElevarmPieceState::ELEVARM_CONE;
    futureState.directionState = ElevarmDirectionState::ELEVARM_FRONT;
    futureState.positionState = ElevarmPositionState::ELEVARM_STOW;
    futureState.manualCarriage = 0;
    futureState.manualArm = 0;
    futureState.deadManEnabled = false;
    previousState = futureState;
}

void Elevarm::init()
{ 
    ValorPIDF carriagePID;
    carriagePID.velocity = CARRIAGE_K_VEL;
    carriagePID.acceleration = carriagePID.velocity * CARRIAGE_K_ACC_MUL;
    carriagePID.F = CARRIAGE_K_F;
    carriagePID.P = CARRIAGE_K_P;
    carriagePID.I = CARRIAGE_K_I;
    carriagePID.D = CARRIAGE_K_D;
    carriagePID.error = CARRIAGE_K_ERROR;
    
    ValorPIDF rotatePID;
    rotatePID.velocity = ROTATE_K_VEL;
    rotatePID.acceleration = rotatePID.velocity * ROTATE_K_ACC_MUL;
    rotatePID.F = ROTATE_K_F;
    rotatePID.P = ROTATE_K_P;
    rotatePID.I = ROTATE_K_I;
    rotatePID.D = ROTATE_K_D;
    rotatePID.error = ROTATE_K_ERROR; 
    
    carriageMotors.setConversion(1.0 / CARRIAGE_GEAR_RATIO * M_PI * CARRAIAGE_OUTPUT_DIAMETER);
    carriageMotors.setForwardLimit(CARRIAGE_UPPER_LIMIT);
    carriageMotors.setReverseLimit(CARRIAGE_LOWER_LIMIT);
    carriageMotors.setPIDF(carriagePID, 0);
    carriageMotors.setupFollower(CANIDs::CARRIAGE_FOLLOW, false);

    armRotateMotor.setConversion((1.0 / ROTATE_GEAR_RATIO) / 360.0);
    armRotateMotor.setForwardLimit(ROTATE_FORWARD_LIMIT);
    armRotateMotor.setReverseLimit(ROTATE_REVERSE_LIMIT);
    armRotateMotor.setPIDF(rotatePID, 0);

    stowPos = frc::Pose3d((units::meter_t) D_CARRIAGE_STOW_X,  0.0_m, (units::meter_t) D_CARRIAGE_STOW_Z, frc::Rotation3d());

    // 0.77

    posMap[ElevarmPieceState::ELEVARM_CONE][ElevarmDirectionState::ELEVARM_FRONT][ElevarmPositionState::ELEVARM_GROUND] = frc::Pose3d( 0.408_m,  0.0_m,  0.0_m, frc::Rotation3d() );
    posMap[ElevarmPieceState::ELEVARM_CONE][ElevarmDirectionState::ELEVARM_BACK][ElevarmPositionState::ELEVARM_GROUND] = frc::Pose3d( -0.408_m,  0.0_m,  0.0_m, frc::Rotation3d() );
    posMap[ElevarmPieceState::ELEVARM_CONE][ElevarmDirectionState::ELEVARM_FRONT][ElevarmPositionState::ELEVARM_PLAYER] = frc::Pose3d( 0.5_m,  0.0_m,  0.946150_m, frc::Rotation3d() );
    posMap[ElevarmPieceState::ELEVARM_CONE][ElevarmDirectionState::ELEVARM_BACK][ElevarmPositionState::ELEVARM_PLAYER] = frc::Pose3d( -0.5_m,  0.0_m,  0.946150_m, frc::Rotation3d() );
    posMap[ElevarmPieceState::ELEVARM_CONE][ElevarmDirectionState::ELEVARM_FRONT][ElevarmPositionState::ELEVARM_MID] = frc::Pose3d( 0.576898_m,  0.0_m,  0.862775_m, frc::Rotation3d() );
    posMap[ElevarmPieceState::ELEVARM_CONE][ElevarmDirectionState::ELEVARM_BACK][ElevarmPositionState::ELEVARM_MID] = frc::Pose3d( -0.576898_m,  0.0_m,  0.862775_m, frc::Rotation3d() );
    posMap[ElevarmPieceState::ELEVARM_CONE][ElevarmDirectionState::ELEVARM_FRONT][ElevarmPositionState::ELEVARM_HIGH] = frc::Pose3d( 0.866_m,  0.0_m,  1.166813_m, frc::Rotation3d() );
    posMap[ElevarmPieceState::ELEVARM_CONE][ElevarmDirectionState::ELEVARM_BACK][ElevarmPositionState::ELEVARM_HIGH] = frc::Pose3d( -0.866_m,  0.0_m,  1.166813_m, frc::Rotation3d() );
    posMap[ElevarmPieceState::ELEVARM_CUBE][ElevarmDirectionState::ELEVARM_FRONT][ElevarmPositionState::ELEVARM_GROUND] = frc::Pose3d( 0.408_m,  0.0_m,  0.0_m, frc::Rotation3d() );
    posMap[ElevarmPieceState::ELEVARM_CUBE][ElevarmDirectionState::ELEVARM_BACK][ElevarmPositionState::ELEVARM_GROUND] = frc::Pose3d( -0.408_m,  0.0_m,  0.0_m, frc::Rotation3d() );
    posMap[ElevarmPieceState::ELEVARM_CUBE][ElevarmDirectionState::ELEVARM_FRONT][ElevarmPositionState::ELEVARM_PLAYER] = frc::Pose3d( 0.5_m,  0.0_m,  0.946150_m, frc::Rotation3d() );
    posMap[ElevarmPieceState::ELEVARM_CUBE][ElevarmDirectionState::ELEVARM_BACK][ElevarmPositionState::ELEVARM_PLAYER] = frc::Pose3d( -0.5_m,  0.0_m,  0.946150_m, frc::Rotation3d() );
    posMap[ElevarmPieceState::ELEVARM_CUBE][ElevarmDirectionState::ELEVARM_FRONT][ElevarmPositionState::ELEVARM_MID] = frc::Pose3d( 0.576898_m,  0.0_m,  0.80_m, frc::Rotation3d() );
    posMap[ElevarmPieceState::ELEVARM_CUBE][ElevarmDirectionState::ELEVARM_BACK][ElevarmPositionState::ELEVARM_MID] = frc::Pose3d( -0.576898_m,  0.0_m,  0.80_m, frc::Rotation3d() );
    posMap[ElevarmPieceState::ELEVARM_CUBE][ElevarmDirectionState::ELEVARM_FRONT][ElevarmPositionState::ELEVARM_HIGH] = frc::Pose3d( 0.866_m,  0.0_m,  1.166813_m, frc::Rotation3d() );
    posMap[ElevarmPieceState::ELEVARM_CUBE][ElevarmDirectionState::ELEVARM_BACK][ElevarmPositionState::ELEVARM_HIGH] = frc::Pose3d( -0.866_m,  0.0_m,  1.166813_m, frc::Rotation3d() );

    table->PutNumber("Carriage Max Manual Speed", manualMaxCarriageSpeed);
    table->PutNumber("Arm Rotate Max Manual Speed", manualMaxArmSpeed);

    resetState();
}

void Elevarm::assessInputs()
{
    futureState.manualCarriage = 0;
    futureState.manualArm = 0;
    if (operatorGamepad->leftStickYActive() || operatorGamepad->rightStickYActive()) {
        futureState.manualCarriage = operatorGamepad->leftStickY() * manualMaxCarriageSpeed;
        futureState.manualArm = operatorGamepad->rightStickY() * manualMaxArmSpeed;
        futureState.positionState = ElevarmPositionState::ELEVARM_MANUAL;
    } else if (operatorGamepad->GetRightBumper()) {
        futureState.positionState = ElevarmPositionState::ELEVARM_STOW;
    } else if (operatorGamepad->GetAButton() || operatorGamepad->DPadDown()){
        futureState.positionState = ElevarmPositionState::ELEVARM_GROUND;
    } else if(operatorGamepad->GetXButton() || operatorGamepad->DPadLeft()){
        futureState.positionState = ElevarmPositionState::ELEVARM_PLAYER;
    } else if(operatorGamepad->GetYButton() || operatorGamepad->DPadUp()){
        futureState.positionState = ElevarmPositionState::ELEVARM_HIGH;
    } else if(operatorGamepad->GetBButton() || operatorGamepad->DPadRight()){
        futureState.positionState = ElevarmPositionState::ELEVARM_MID;
    } else {
        if (previousState.positionState != ElevarmPositionState::ELEVARM_MANUAL) {
            futureState.positionState = ElevarmPositionState::ELEVARM_STOW;
        } 
    }

    if (operatorGamepad->DPadUp() || operatorGamepad->DPadDown() 
    || operatorGamepad->DPadLeft() || operatorGamepad->DPadRight()){
        futureState.pieceState = ElevarmPieceState::ELEVARM_CUBE;
    } else {
        futureState.pieceState = ElevarmPieceState::ELEVARM_CONE;
    }

    if (operatorGamepad->GetLeftBumper()){
        futureState.directionState = ElevarmDirectionState::ELEVARM_BACK;
    } else {
        futureState.directionState = ElevarmDirectionState::ELEVARM_FRONT;
    }

    futureState.deadManEnabled = operatorGamepad->GetRightTriggerAxis();
}

void Elevarm::analyzeDashboard()
{
    manualMaxCarriageSpeed = table->GetNumber("Carriage Max Manual Speed", 1.0);
    manualMaxArmSpeed = table->GetNumber("Arm Rotate Max Manual Speed", 1.0);
    table->PutNumber("Carriage Encoder Value", carriageMotors.getPosition());
    table->PutNumber("Arm rotate Encoder Value", armRotateMotor.getPosition());

    table->PutNumber("EA position state", futureState.positionState);
    table->PutNumber("EA piece state", futureState.pieceState);
    table->PutNumber("EA direction state", futureState.directionState);
    table->PutNumber("target height", futureState.targetPose.h);
    table->PutNumber("target rotation", futureState.targetPose.theta);
}

void Elevarm::assignOutputs()
{    
    if (futureState.positionState == ElevarmPositionState::ELEVARM_STOW) {
        futureState.targetPose = reverseKinematics(stowPos, ElevarmSolutions::ELEVARM_LEGS);
    } else {
        if (futureState.directionState != previousState.directionState) {
        futureState.targetPose = reverseKinematics(stowPos, ElevarmSolutions::ELEVARM_LEGS);
        } else {
            if (futureState.positionState == ElevarmPositionState::ELEVARM_PLAYER || futureState.positionState == ElevarmPositionState::ELEVARM_MID) {
                futureState.targetPose = reverseKinematics(posMap[futureState.pieceState][futureState.directionState][futureState.positionState], ElevarmSolutions::ELEVARM_ARMS);
            } else 
                futureState.targetPose = reverseKinematics(posMap[futureState.pieceState][futureState.directionState][futureState.positionState], ElevarmSolutions::ELEVARM_LEGS);
        }
    }


    if (futureState.deadManEnabled) {
        if (futureState.positionState == ElevarmPositionState::ELEVARM_MANUAL) {
            carriageMotors.setPower(futureState.manualCarriage);
            armRotateMotor.setPower(futureState.manualArm);
            previousState.positionState = ElevarmPositionState::ELEVARM_MANUAL;
        } else {
            carriageMotors.setPosition(futureState.targetPose.h);
            armRotateMotor.setPosition(futureState.targetPose.theta);
            previousState = ((std::abs(carriageMotors.getPosition() - futureState.targetPose.h)  <= PREVIOUS_HEIGHT_DEADBAND) && 
            (std::abs(armRotateMotor.getPosition() - futureState.targetPose.theta) <= PREVIOUS_ROTATION_DEADBAND))  ? futureState : previousState;
        }
    } else {
        carriageMotors.setPower(0);
        armRotateMotor.setPower(0);
        previousState = ((std::abs(carriageMotors.getPosition() - futureState.targetPose.h)  <= PREVIOUS_HEIGHT_DEADBAND) && 
        (std::abs(armRotateMotor.getPosition() - futureState.targetPose.theta) <= PREVIOUS_ROTATION_DEADBAND))  ? futureState : previousState;
    }
}


Elevarm::Positions Elevarm::reverseKinematics(frc::Pose3d pose, ElevarmSolutions solution) 
{
    double phi = 0.0;
    double theta = 0.0;
    double height = 0.0;

    table->PutNumber("target Xt", pose.X().to<double>());
    table->PutNumber("target Zt", pose.Z().to<double>());

    // Arms solution
    if (solution == ElevarmSolutions::ELEVARM_ARMS) {
        phi = std::acos((pose.X().to<double>() + X_BUMPER_WIDTH + X_HALF_WIDTH - X_CARRIAGE_OFFSET ) / X_ARM_LENGTH);
        theta = phi + M_PI / 2.0;
        height = pose.Z().to<double>() - (X_ARM_LENGTH * std::sin(phi)) + Z_INTAKE_OFFSET;

    // Legs Solution
    } else {
        theta = std::asin((pose.X().to<double>() + X_BUMPER_WIDTH + X_HALF_WIDTH - X_CARRIAGE_OFFSET ) / X_ARM_LENGTH);
        height = pose.Z().to<double>() + (X_ARM_LENGTH * std::cos(theta)) - Z_INTAKE_OFFSET;
    }
    height -= (D_CARRIAGE_FLOOR_OFFSET + D_CARRIAGE_JOINT_OFFSET);

    return Positions(height,theta * 180.0 / M_PI);
}