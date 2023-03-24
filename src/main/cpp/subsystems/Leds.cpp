#include "subsystems/Leds.h"
#include <iostream>

#define LED_COUNT 286

Leds::Leds(frc::TimedRobot *_robot, Elevarm *_elevarm, Intake *_intake, Drivetrain *_drivetrain) : ValorSubsystem(_robot, "Leds"),
    elevarm(_elevarm),
    intake(_intake),
    drivetrain(_drivetrain),
    candle(_robot, LED_COUNT, CANIDs::CANDLE, "baseCAN")
{
    frc2::CommandScheduler::GetInstance().RegisterSubsystem(this);
    init();
}

void Leds::init(){resetState();}
void Leds::resetState(){
    candle.setColor(0xEEA800);
    candle.clearAnimation();
}

void Leds::assessInputs(){}

void Leds::analyzeDashboard(){

    if (intake && intake->state.intakeState == Intake::IntakeStates::SPIKED) {
        candle.setColor(255,0,0);
        candle.setAnimation(ValorCANdleSensor::AnimationType::Strobe);
    } else if (robot->IsDisabled()) {
        if (elevarm->futureState.armInRange) candle.setColor(0,255,0);
        else candle.setColor(255,0,0);
    } else if (robot->IsAutonomous()) {
        if (elevarm->futureState.atCarriage && elevarm->futureState.atArm && elevarm->futureState.atWrist){
            candle.setColor(0,255,0);
        } else {
            candle.setColor(0,0,255);
        }
    } else if (intake->getFuturePiece() == Piece::CUBE){
        candle.setColor(156,0,255);
        candle.clearAnimation();
    } else if (intake->getFuturePiece() == Piece::CONE){
        candle.setColor(255,196,0);
        candle.clearAnimation();
    }
}

void Leds::assignOutputs(){}

void Leds::InitSendable(wpi::SendableBuilder& builder){
    builder.SetSmartDashboardType("Leds");
}