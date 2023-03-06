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
        struct x {bool yButton;}state;
        

        void init();
        void assessInputs();
        void analyzeDashboard();
        void assignOutputs();
        void resetState();
        void InitSendable(wpi::SendableBuilder& builder) override;
        
        int VALOR_GOLD = 0xEEA800;
        int PURPLE = 0x3C14BE;

        int LIME_COLOR = 0x054C00;
        int XMODE = 0x000000;

        int PINK = 0xFE019A;
        int ORANGE = 0xed7e15;
        int TEAL = 0x3dffcb;
        int WHITE = 0xfafafa;
        int LIGHT_GREEN = 0xd4f0a5;
        int BABY_BLUE = 0x71d2f0;

};