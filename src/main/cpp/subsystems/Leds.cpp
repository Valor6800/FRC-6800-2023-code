#include "subsystems/Leds.h"
#include <iostream>

#define LED_COUNT 294

#define CARRIAGE_UPPER_LIMIT 0.89f 
#define CARRIAGE_LOWER_LIMIT 0.0f

#define ROTATE_FORWARD_LIMIT 180.0f
#define ROTATE_REVERSE_LIMIT -180.0f

#define WRIST_FORWARD_LIMIT 325.0f
#define WRIST_REVERSE_LIMIT -325.0f

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
    candle.clearAnimation();
    candle.setColor(ValorCANdleSensor::RGBColor(0,0,0));
    // candle.setColor(ValorCANdleSensor::toRGB(0xEEA800));
}

void Leds::assessInputs(){}

void Leds::analyzeDashboard(){

    if (elevarm->futureState.pitModeEnabled){
        candle.setColor(2, ValorCANdleSensor::RGBColor(255, 80, 0));
        candle.setColor(1, ValorCANdleSensor::RGBColor(255, 80, 0));
    } else{
        candle.setColor(2,ValorCANdleSensor::RGBColor(0,0,0));
        candle.setColor(1,ValorCANdleSensor::RGBColor(0,0,0));
    }
    if (robot->IsDisabled()) {//Disabled
        if (elevarm->futureState.armInRange) {
            candle.setColor(4, ValorCANdleSensor::RGBColor(0, 255, 0));
        } else {
            candle.setColor(4, ValorCANdleSensor::RGBColor(255, 0, 0));
        }
    } else if (robot->IsAutonomous()) {//Checks if robot is in auto
        if (elevarm->futureState.atCarriage && elevarm->futureState.atArm && elevarm->futureState.atWrist){
            candle.setColor(4, ValorCANdleSensor::RGBColor(0, 255, 0));
        } else {//When the elevarm is not at the correct state
            candle.setColor(4, ValorCANdleSensor::RGBColor(0, 0, 255));
        }
    } else {//Teleop
        if (intake && intake->state.intakeState == Intake::IntakeStates::SPIKED) {//spiked
            candle.setColor(3, ValorCANdleSensor::RGBColor(255, 192, 203));
        } else {
            candle.setColor(3, candle.getColor(4));
        }
        
        if (intake->getFuturePiece() == Piece::CUBE){
            candle.setColor(4, ValorCANdleSensor::RGBColor(156, 0, 255));
        } else if(intake->getFuturePiece() == Piece::CONE){
            candle.setColor(4, ValorCANdleSensor::RGBColor(255, 196, 0));
        }
        //elevarm being set up with candle
        // candle.setColor(2, ValorCANdleSensor::RGBColor(
        //     255*(elevarm->getCarriagePosition()/CARRIAGE_UPPER_LIMIT),
        //     255*((elevarm->getArmPosition()+180)/(ROTATE_FORWARD_LIMIT*2)),
        //     255*((elevarm->getWristPosition()+325)/(WRIST_FORWARD_LIMIT*2))
        // ));
    }
}

void Leds::assignOutputs(){}

void Leds::InitSendable(wpi::SendableBuilder& builder){
    builder.SetSmartDashboardType("Leds");
    builder.AddDoubleArrayProperty(
        "Seg 4 colors", 
        [this] {
            std::vector<double> color{(double) candle.getColor(4).red, (double) candle.getColor(4).green, (double) candle.getColor(4).blue};
            return color;
            },
        nullptr
    );

    builder.AddDoubleArrayProperty(
        "Seg 3 colors",
        [this] {
            std::vector<double> color{(double) candle.getColor(3).red, (double) candle.getColor(3).green, (double) candle.getColor(3).blue};
            return color;
        },
        nullptr
    );

    builder.AddDoubleArrayProperty(
        "Seg 2 colors", 
        [this] {
            std::vector<double> color{(double) candle.getColor(2).red, (double) candle.getColor(2).green, (double) candle.getColor(2).blue};
            return color;
            },
        nullptr
    );

    builder.AddDoubleArrayProperty(
        "Seg 1 colors", 
        [this] {
            std::vector<double> color{(double) candle.getColor(1).red, (double) candle.getColor(1).green, (double) candle.getColor(1).blue};
            return color;
            },
        nullptr
    );
}