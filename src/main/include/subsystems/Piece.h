#pragma once

#include "ValorSubsystem.h"
#include "Constants.h"
#include "controllers/ValorFalconController.h"
#include "controllers/ValorNeoController.h"
#include "sensors/ValorCurrentSensor.h"

#include <frc/smartdashboard/SendableChooser.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include <frc2/command/FunctionalCommand.h>
class Piece : public ValorSubsystem
{
public:  
     enum PieceState{
        CONE,
        CUBE
     };

};