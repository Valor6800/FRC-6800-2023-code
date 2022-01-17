#pragma once

#include "ValorSubsystem.h"
#include "Constants.h"
#include <frc/XboxController.h>


#ifndef LIFT_H
#define LIFT_H


class lift : public ValorSubsystem 
{
public:
    lift();
    ~lift();

    void init();
    void setController(frc::XboxController *controller);

    void assessInputs();
    void analyzeDashboard();
    void assignOutputs();

    void resetState();

    enum LiftMainState {
            DISABLED,
            EXTEND,
            RETRACT,
        };
        
    enum LiftAuxState {
            DISABLED,
            EXTEND,
            RETRACT,
        };
        
    enum LiftRotateState {
            DISABLED,
            ROTATE,
        };


    struct x
    {
        LiftMainState liftstateMain;
        LiftAuxState liftstateAux;
        LiftRotateState liftstateRotate;
        bool xButtonPressed;
        bool yButtonPressed;

        bool dPadUpPressed;  
        bool dPadDownPressed;

        double leftStickY;

    } state;

};

#endif