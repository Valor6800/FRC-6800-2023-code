#pragma once
#include <functional>
#include <frc/TimedRobot.h>

#include "ValorSubsystem.h"
#include "sensors/ValorCANdleSensor.h"
#include "Constants.h"

#include "subsystems/Elevarm.h"
#include "subsystems/Drivetrain.h"
#include "subsystems/Intake.h"

#include "ctre/Phoenix.h"
#include "ctre/phoenix/led/CANdle.h"
#include "ctre/phoenix/led/StrobeAnimation.h"

#include <vector>
class Leds : public ValorSubsystem
{
    public:
        Leds(frc::TimedRobot *robot, Elevarm *elevarm, Intake *intake, Drivetrain *drivetrain);
        ValorCANdleSensor::RGBColor value;
        
        void init();
        void assessInputs();
        void analyzeDashboard();
        void assignOutputs();
        void resetState();
        void InitSendable(wpi::SendableBuilder& builder) override;

        struct State{
            std::vector<units::second_t> startedAnimating; 
        } state;
        
    private:
        ValorCANdleSensor candle;
        Elevarm *elevarm;
        Intake *intake;
        Drivetrain *drivetrain;

};