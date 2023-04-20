#include "subsystems/Leds.h"
#include <iostream>

#define LED_COUNT 288

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
    candle(_robot, LED_COUNT, 2, CANIDs::CANDLE, "baseCAN")
{
    frc2::CommandScheduler::GetInstance().RegisterSubsystem(this);
    init();

    state.startedAnimating = std::vector<units::second_t>(4, -1_s);
}

void Leds::init(){resetState();}
void Leds::resetState(){
    candle.clearAnimation();
    candle.setColor(ValorCANdleSensor::RGBColor(0,0,0));
    // candle.setColor(ValorCANdleSensor::toRGB(0xEEA800));
}

void Leds::assessInputs(){}

void Leds::analyzeDashboard(){

    if (elevarm->futureState.pitModeEnabled) {
        candle.setColor(ValorCANdleSensor::RGBColor(255, 90, 0));
    } else if (robot->IsDisabled()) {
        if (elevarm->futureState.wristInRange) candle.setColor(1, ValorCANdleSensor::RGBColor(0, 255, 0));
        else candle.setColor(1, ValorCANdleSensor::RGBColor(255, 0, 0));
        if (elevarm->futureState.armInRange) candle.setColor(2, ValorCANdleSensor::RGBColor(0, 255, 0));
        else candle.setColor(2, ValorCANdleSensor::RGBColor(255, 0, 0));
    } else if (robot->IsAutonomous()) {
        if (elevarm->futureState.atCarriage && elevarm->futureState.atArm && elevarm->futureState.atWrist){
            candle.setColor(ValorCANdleSensor::RGBColor(0, 255, 0));
        } else{
            candle.setColor(ValorCANdleSensor::RGBColor(0, 0, 255));
        }
    } else if (robot->IsEnabled()){
        if (intake->getFuturePiece() == Piece::CUBE) {
            candle.setColor(1, ValorCANdleSensor::RGBColor(156, 0, 255));
        } else {
            candle.setColor(1, ValorCANdleSensor::RGBColor(255, 196, 0));
        }

        if (intake->state.intakeState == Intake::IntakeStates::SPIKED){
            candle.setColor(2,ValorCANdleSensor::RGBColor(255, 0, 0));
        } else {
            candle.setColor(2, ValorCANdleSensor::RGBColor(255, 255, 255));
        }
    }

}

void Leds::assignOutputs(){}

void Leds::InitSendable(wpi::SendableBuilder& builder){
    builder.SetSmartDashboardType("Leds");

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