/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
#include "subsystems/Elevarm.h"
#include <iostream>


#define ROTATE_GEAR_RATIO 12.0f
#define ROTATE_OUTPUT_DIAMETER 0.1f //get value from cad
#define CARRIAGE_GEAR_RATIO 4.0f
#define CARRAIAGE_OUTPUT_DIAMETER 0.1f //get value from cad
#define CARRIAGE_UPPER_LIMIT 1.0f
#define CARRIAGE_LOWER_LIMIT 0.0f
#define ROTATE_FORWARD_LIMIT 180.0f
#define ROTATE_REVERSE_LIMIT 180.0f

#define CARRIAGE_K_F 0.05f  
#define CARRIAGE_K_P 0.1f
#define CARRIAGE_K_I 0.0f
#define CARRIAGE_K_D 0.0f
#define CARRIAGE_K_ 0.0f
#define CARRIAGE_K_VEL 0.0f
#define CARRIAGE_K_ACC_MUL 0.0f

#define ROTATE_K_F 0.0000753f
#define ROTATE_K_P 5e-5f
#define ROTATE_K_I 0.0f
#define ROTATE_K_D 0.0f
#define ROTATE_K_ 0.0f
#define ROTATE_K_VEL 0.0f
#define ROTATE_K_ACC_MUL 0.0f

#define PREVIOUS_HEIGHT_DEADBAND 0.01f
#define PREVIOUS_ROTATION_DEADBAND 0.5f

#define X_BUMPER_WIDTH 0.0762f
#define X_HALF_WIDTH 0.254f
#define X_CARRIAGE_OFFSET 0.1778f
#define X_ARM_LENGTH 1.01854f

#define D_CARRIAGE_JOINT_OFFSET 0.0724154f
#define D_CARRIAGE_FLOOR_OFFSET 0.02345502f
#define D_CARRIAGE_MAX_TRAVEL_M 1.0668f

#define SCORING_HEIGHT_OFFSET_M 0.0508f



Elevarm::Elevarm(frc::TimedRobot *_robot) : ValorSubsystem(_robot, "Elevarm"),                        
                            carriageMotors(CANIDs::CARRIAGE_MAIN, rev::CANSparkMax::IdleMode::kBrake, false),
                            armRotateMotor(CANIDs::ARM_ROTATE, rev::CANSparkMax::IdleMode::kBrake, false)
{
    frc2::CommandScheduler::GetInstance().RegisterSubsystem(this);
    init();
}

Elevarm::~Elevarm()
{
    
}


void Elevarm::resetState()
{
   
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
    
    ValorPIDF rotatePID;
    rotatePID.velocity = ROTATE_K_VEL;
    rotatePID.acceleration = rotatePID.velocity * ROTATE_K_ACC_MUL;
    rotatePID.F = ROTATE_K_F;
    rotatePID.P = ROTATE_K_P;
    rotatePID.I = ROTATE_K_I;
    rotatePID.D = ROTATE_K_D;
    
    
    carriageMotors.setConversion(1 / CARRIAGE_GEAR_RATIO * M_PI * CARRAIAGE_OUTPUT_DIAMETER);
    carriageMotors.setForwardLimit(CARRIAGE_UPPER_LIMIT);
    carriageMotors.setReverseLimit(CARRIAGE_LOWER_LIMIT);
    carriageMotors.setPIDF(carriagePID, 0);
    carriageMotors.setupFollower(12, false);

    armRotateMotor.setConversion(1 / ROTATE_GEAR_RATIO * M_PI * ROTATE_OUTPUT_DIAMETER);
    armRotateMotor.setForwardLimit(ROTATE_FORWARD_LIMIT);
    armRotateMotor.setReverseLimit(ROTATE_REVERSE_LIMIT);
    armRotateMotor.setPIDF(rotatePID, 0);


    frc::Pose3d stowPos = frc::Pose3d(0.0_m,  0.0_m, (units::meter_t) D_CARRIAGE_MAX_TRAVEL_M, frc::Rotation3d());


    posMap[ElevarmPieceState::ELEVARM_CONE][ElevarmDirectionState::ELEVARM_FRONT][ElevarmPositionState::ELEVARM_GROUND] == frc::Pose3d( 0.408_m,  0.0_m,  0.0_m, frc::Rotation3d() );
    posMap[ElevarmPieceState::ELEVARM_CONE][ElevarmDirectionState::ELEVARM_BACK][ElevarmPositionState::ELEVARM_GROUND] == frc::Pose3d( -0.408_m,  0.0_m,  0.0_m, frc::Rotation3d() );
    posMap[ElevarmPieceState::ELEVARM_CONE][ElevarmDirectionState::ELEVARM_FRONT][ElevarmPositionState::ELEVARM_PLAYER] == frc::Pose3d( 0.5_m,  0.0_m,  0.946150_m, frc::Rotation3d() );
    posMap[ElevarmPieceState::ELEVARM_CONE][ElevarmDirectionState::ELEVARM_BACK][ElevarmPositionState::ELEVARM_PLAYER] == frc::Pose3d( -0.5_m,  0.0_m,  0.946150_m, frc::Rotation3d() );
    posMap[ElevarmPieceState::ELEVARM_CONE][ElevarmDirectionState::ELEVARM_FRONT][ElevarmPositionState::ELEVARM_MID] == frc::Pose3d( 0.576898_m,  0.0_m,  0.862775_m, frc::Rotation3d() );
    posMap[ElevarmPieceState::ELEVARM_CONE][ElevarmDirectionState::ELEVARM_BACK][ElevarmPositionState::ELEVARM_MID] == frc::Pose3d( -0.576898_m,  0.0_m,  0.862775_m, frc::Rotation3d() );
    posMap[ElevarmPieceState::ELEVARM_CONE][ElevarmDirectionState::ELEVARM_FRONT][ElevarmPositionState::ELEVARM_HIGH] == frc::Pose3d( 1.00903_m,  0.0_m,  1.166813_m, frc::Rotation3d() );
    posMap[ElevarmPieceState::ELEVARM_CONE][ElevarmDirectionState::ELEVARM_BACK][ElevarmPositionState::ELEVARM_HIGH] == frc::Pose3d( -1.00903_m,  0.0_m,  1.166813_m, frc::Rotation3d() );
    posMap[ElevarmPieceState::ELEVARM_CUBE][ElevarmDirectionState::ELEVARM_FRONT][ElevarmPositionState::ELEVARM_GROUND] == frc::Pose3d( 0.408_m,  0.0_m,  0.0_m, frc::Rotation3d() );
    posMap[ElevarmPieceState::ELEVARM_CUBE][ElevarmDirectionState::ELEVARM_BACK][ElevarmPositionState::ELEVARM_GROUND] == frc::Pose3d( -0.408_m,  0.0_m,  0.0_m, frc::Rotation3d() );
    posMap[ElevarmPieceState::ELEVARM_CUBE][ElevarmDirectionState::ELEVARM_FRONT][ElevarmPositionState::ELEVARM_PLAYER] == frc::Pose3d( 0.5_m,  0.0_m,  0.946150_m, frc::Rotation3d() );
    posMap[ElevarmPieceState::ELEVARM_CUBE][ElevarmDirectionState::ELEVARM_BACK][ElevarmPositionState::ELEVARM_PLAYER] == frc::Pose3d( -0.5_m,  0.0_m,  0.946150_m, frc::Rotation3d() );
    posMap[ElevarmPieceState::ELEVARM_CUBE][ElevarmDirectionState::ELEVARM_FRONT][ElevarmPositionState::ELEVARM_MID] == frc::Pose3d( 0.576898_m,  0.0_m,  0.80_m, frc::Rotation3d() );
    posMap[ElevarmPieceState::ELEVARM_CUBE][ElevarmDirectionState::ELEVARM_BACK][ElevarmPositionState::ELEVARM_MID] == frc::Pose3d( -0.576898_m,  0.0_m,  0.80_m, frc::Rotation3d() );
    posMap[ElevarmPieceState::ELEVARM_CUBE][ElevarmDirectionState::ELEVARM_FRONT][ElevarmPositionState::ELEVARM_HIGH] == frc::Pose3d( 1.00903_m,  0.0_m,  1.166813_m, frc::Rotation3d() );
    posMap[ElevarmPieceState::ELEVARM_CUBE][ElevarmDirectionState::ELEVARM_BACK][ElevarmPositionState::ELEVARM_HIGH] == frc::Pose3d( -1.00903_m,  0.0_m,  1.166813_m, frc::Rotation3d() );




    table->PutNumber("Carriage Encoder Value", carriageMotors.getPosition());
    table->PutNumber("Arm rotate Encoder Value", armRotateMotor.getPosition());

    resetState();
}

void Elevarm::assessInputs()
{
    
    if (operatorGamepad->GetAButton() || operatorGamepad->DPadDown()){
        futureState.positionState = ElevarmPositionState::ELEVARM_GROUND;
    } else if(operatorGamepad->GetXButton() || operatorGamepad->DPadLeft()){
        futureState.positionState = ElevarmPositionState::ELEVARM_PLAYER;
    } else if(operatorGamepad->GetYButton() || operatorGamepad->DPadUp()){
        futureState.positionState = ElevarmPositionState::ELEVARM_HIGH;
    } else if(operatorGamepad->GetBButton() || operatorGamepad->DPadRight()){
        futureState.positionState = ElevarmPositionState::ELEVARM_MID;
    } else {
        futureState.positionState = ElevarmPositionState::ELEVARM_STOW;
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

}
void Elevarm::analyzeDashboard()
{
    

}

void Elevarm::assignOutputs()
{    
    std::pair<double,double> targetPose;

    if (futureState.positionState == ElevarmPositionState::ELEVARM_STOW) {
        targetPose = reverseKinematics(stowPos, ElevarmSolutions::ELEVARM_D);
    } else {
        if (futureState.directionState == previousState.directionState) {
            if (futureState.directionState == ElevarmDirectionState::ELEVARM_BACK) {
                if (futureState.positionState == ElevarmPositionState::ELEVARM_PLAYER || futureState.positionState == ElevarmPositionState::ELEVARM_MID) {
                    solutionsEnum = ElevarmSolutions::ELEVARM_C;
                } else {
                    solutionsEnum = ElevarmSolutions::ELEVARM_D;
                }
            } else {
                if (futureState.positionState == ElevarmPositionState::ELEVARM_PLAYER || futureState.positionState == ElevarmPositionState::ELEVARM_MID) {
                    solutionsEnum = ElevarmSolutions::ELEVARM_A;
                } else {
                    solutionsEnum = ElevarmSolutions::ELEVARM_B;
                }
            }
        } else if (futureState.directionState != previousState.directionState) {
            targetPose = reverseKinematics(stowPos, ElevarmSolutions::ELEVARM_D);
        } else
        targetPose = reverseKinematics(posMap[futureState.pieceState][futureState.directionState][futureState.positionState], solutionsEnum);
    }

    carriageMotors.setPosition(targetPose.first);
    armRotateMotor.setPosition(targetPose.second);

    previousState = ((std::abs(carriageMotors.getPosition() - targetPose.first)  <= PREVIOUS_HEIGHT_DEADBAND) && 
    (std::abs(armRotateMotor.getPosition() - targetPose.second) <= PREVIOUS_ROTATION_DEADBAND))  ? futureState : previousState;



}


std::pair<double, double> Elevarm::reverseKinematics(frc::Pose3d pose, ElevarmSolutions solution) 
{
    double arms = ((solution == ElevarmSolutions::ELEVARM_A) || (solution == ElevarmSolutions::ELEVARM_C))  ? 1.0 : -1.0;
    double direction = (futureState.directionState == ElevarmDirectionState::ELEVARM_FRONT)  ? 1.0 : -1.0;
    
    double h = pose.Z().to<double>() - D_CARRIAGE_JOINT_OFFSET - D_CARRIAGE_FLOOR_OFFSET + (arms * X_ARM_LENGTH) * std::sqrt(1 - std::pow(((pose.X().to<double>() + X_BUMPER_WIDTH + X_HALF_WIDTH - (direction * X_CARRIAGE_OFFSET) ) / X_ARM_LENGTH) , 2.0 ));
    double r = direction * std::asin((pose.X().to<double>() + X_BUMPER_WIDTH + X_HALF_WIDTH - (direction * X_CARRIAGE_OFFSET) ) / X_ARM_LENGTH );
 
    return std::pair(h, r);
}