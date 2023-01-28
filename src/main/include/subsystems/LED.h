#pragma once

#include <functional>
#include <frc/TimedRobot.h>

#include "ValorSubsystem.h"
#include "subsystems/Elevarm.h"
#include "Constants.h"

#include "controllers/ValorFalconController.h"
#include "controllers/ValorNeoController.h"
#include "controllers/ValorPIDF.h"

#include "ctre/Phoenix.h"
#include "ctre/phoenix/led/CANdle.h"
#include "ctre/phoenix/led/StrobeAnimation.h"

#include "sensors/ValorCANdleSensor.h"

class LED : public ValorSubsystem
{
    public:
        LED(frc::TimedRobot *robot);
        ValorCANdleSensor led;

        ValorCANdleSensor::RGBColor value;

        void init();
        void assessInputs();
        void analyzeDashboard();
        void assignOutputs();
        void resetState();
        void InitSendable(wpi::SendableBuilder& builder) override;
};