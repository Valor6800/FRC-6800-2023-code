#pragma once

#include "ValorSubsystem.h"
#include "Constants.h"
#include "controllers/ValorFalconController.h"
#include "controllers/ValorNeoController.h"
#include "sensors/ValorCurrentSensor.h"

#include <frc/smartdashboard/SendableChooser.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include <frc2/command/FunctionalCommand.h>
class Direction : public ValorSubsystem
{
public:
     /**
      * @brief Construct a new Intake object
      * 
      * @param robot Top level robot object to parse out smart dashboard and table information
      */
     Direction(frc::TimedRobot *robot);

     /**
      * @brief Destroy the Intake object
      * 
      * Intake objects have member objects on the heap - need a destructor to take care of memory on destruction
      */
     ~Direction();

     /**
      * @brief Initialize the Intake
      * 
      */
     void init();

     void assessInputs();
     void analyzeDashboard();
     void assignOutputs();

     void resetState();

     void InitSendable(wpi::SendableBuilder& builder);

     


     //this state will come from elevarm, elevarm not currently connected to dev

     enum DirectionState{
        FRONT,
        BACK
     };

     struct x
     {
         DirectionState directionState;

     }state, prevState;
};