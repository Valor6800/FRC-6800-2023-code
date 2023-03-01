#include "subsystems/LED.h"
#include <iostream>

#define LED_COUNT 152

LED::LED(frc::TimedRobot *_robot) : ValorSubsystem(_robot, "ValorLED"),
    led(robot, LED_COUNT, 60, "baseCAN")
{
    frc2::CommandScheduler::GetInstance().RegisterSubsystem(this);
    init();
}        

void LED::init(){led.setRangeLED(CANdleColors::VALOR_GOLD, 0, LED_COUNT);}

void LED::assessInputs(){state.yButton=driverGamepad->GetYButton();}

void LED::analyzeDashboard(){}

void LED::assignOutputs()
{
    int pieceColor;
    if (state.yButton){
        pieceColor = CANdleColors::PURPLE;
    } else pieceColor = CANdleColors::VALOR_GOLD;

    led.setRangeLED(pieceColor, 0, LED_COUNT);
}

void LED::resetState(){led.setRangeLED(CANdleColors::VALOR_GOLD, 0, LED_COUNT);}

void LED::InitSendable(wpi::SendableBuilder& builder){
    builder.SetSmartDashboardType("LEDs");
}