#include "subsystems/Leds.h"
#include <iostream>

#define LED_COUNT 286

Leds::Leds(frc::TimedRobot *_robot, Elevarm *_elevarm, Intake *_intake, Drivetrain *_drivetrain) : ValorSubsystem(_robot, "Leds"),
    elevarm(_elevarm),
    intake(_intake),
    drivetrain(_drivetrain),
    candle(_robot, LED_COUNT, 4, CANIDs::CANDLE, "baseCAN")
{
    frc2::CommandScheduler::GetInstance().RegisterSubsystem(this);
    init();
}

void Leds::init(){resetState();}
void Leds::resetState(){
    
    candle.setColor(ValorCANdleSensor::toRGB(0xEEA800));
    candle.clearAnimation();
}

void Leds::assessInputs(){}

void Leds::analyzeDashboard(){

    if (robot->IsDisabled()) {
        if (elevarm->futureState.armInRange) candle.setColor(ValorCANdleSensor::RGBColor(0,255,0));
        else candle.setColor(ValorCANdleSensor::RGBColor(255,0,0));
    } else if (robot->IsAutonomous()) {
        if (elevarm->futureState.atCarriage && elevarm->futureState.atArm && elevarm->futureState.atWrist){
            candle.setColor(ValorCANdleSensor::RGBColor(0,255,0));
        } else {
            candle.setColor(ValorCANdleSensor::RGBColor(0,0,255));
        }
    }
    else{
        if (intake && intake->state.intakeState == Intake::IntakeStates::SPIKED) {
            // candle.setColor(4,ValorCANdleSensor::RGBColor(255,0,0));
            candle.setAnimation(4,ValorCANdleSensor::AnimationType::Strobe,.25);
        } 
        if (intake->getFuturePiece() == Piece::CUBE){
            candle.setColor(4,ValorCANdleSensor::RGBColor(156,0,255));
            candle.clearAnimation(4);
        } else if(intake->getFuturePiece() == Piece::CONE){
            candle.setColor(4,ValorCANdleSensor::RGBColor(255,196,0));
            candle.clearAnimation(4);
        }
    }
    candle.setAnimation(3, ValorCANdleSensor::AnimationType::Rainbow);
    // candle.setAnimation(2, ValorCANdleSensor::AnimationType::Rainbow);
    candle.setAnimation(1, ValorCANdleSensor::AnimationType::Rainbow);
}

void Leds::assignOutputs(){}

void Leds::InitSendable(wpi::SendableBuilder& builder){
    builder.SetSmartDashboardType("Leds");
}