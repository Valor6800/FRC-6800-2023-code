/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
#include "subsystems/Elevarm.h"
#include <iostream>

#define ROTATE_GEAR_RATIO 12.0f
#define CARRIAGE_GEAR_RATIO 4.0f


Elevarm::Elevarm(frc::TimedRobot *_robot) : ValorSubsystem(_robot, "Elevarm"),                        
                            carriageMain(11, rev::CANSparkMax::IdleMode::kBrake, false),
                            carriageFollow(12, rev::CANSparkMax::IdleMode::kBrake, false),
                            armRotate(13, rev::CANSparkMax::IdleMode::kBrake, false)
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
    // table->PutNumber("Motor 3 Max Speed", 0.0); 
    carriageMain.setConversion(CARRIAGE_GEAR_RATIO);
    carriageMain.setupFollower(12, false);
    

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
    
    // table->GetNumber("Motor 3 Max Speed", motor3MaxSpeed);

}

void Elevarm::assignOutputs()
{    
    if (futureState.positionState == ElevarmPositionState::ELEVARM_STOW){
        
    }
}
