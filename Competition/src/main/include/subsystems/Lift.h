#pragma once

#include "ValorSubsystem.h"
#include "Constants.h"
#include <frc/XboxController.h>
#include <rev/CANSparkMax.h>

#ifndef LIFT_H
#define LIFT_H

class Lift : public ValorSubsystem 
{
public:
    Lift();
    ~Lift();

    void init();
    void setController(frc::XboxController *controller);

    void assessInputs();
    void analyzeDashboard();
    void assignOutputs();

    void resetState();

    enum LiftMainState {
        LIFT_MAIN_DISABLED,
        LIFT_MAIN_EXTEND,
        LIFT_MAIN_RETRACT
    };
        
    enum LiftAuxState {
        LIFT_AUX_DISABLED,
        LIFT_AUX_EXTEND,
        LIFT_AUX_RETRACT
    };
        
    enum LiftRotateState {
        LIFT_ROTATE_DISABLED,
        LIFT_ROTATE_ENABLE
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

        double powerRetract;
        double powerExtend;
        double powerAuxRetract;
        double powerAuxExtend;
        double powerRotate;

    } state;

    frc::XboxController *operatorController;

    rev::CANSparkMax leadMainMotor;
    rev::CANSparkMax followMainMotor;

    rev::CANSparkMax auxMotor;
    
    rev::CANSparkMax rotateMotor;

};

#endif