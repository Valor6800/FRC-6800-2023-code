#pragma once

#include "ValorSubsystem.h"
#include "Constants.h"
#include <frc/XboxController.h>
#include <rev/CANSparkMax.h>

#include "ValorSubsystem.h"
#include "Constants.h"
#include <ctre/Phoenix.h>
#include <vector>

#include <rev/CANSparkMax.h>
#include <rev/CANEncoder.h>

#include <frc/XboxController.h>

#ifndef LIFT_H
#define LIFT_H

class Lift : public ValorSubsystem 
{
public:
    Lift();

    void init();
    void setController(frc::XboxController *controller);

    void assessInputs();
    void analyzeDashboard();
    void assignOutputs();

    void resetState();

    enum LiftRotateState {
        LIFT_ROTATE_DISABLED,
        LIFT_ROTATE_EXTEND,
        LIFT_ROTATE_RETRACT,
        LIFT_ROTATE_TOPOSITION
    };
        
    enum LiftMainState {
        LIFT_MAIN_DISABLED,
        LIFT_MAIN_ENABLE,
        LIFT_MAIN_TOPOSITION,
        LIFT_MAIN_FIRSTPOSITION
    };


    struct x
    {
        LiftMainState liftstateMain;
        LiftRotateState liftstateRotate;

        bool dPadUpPressed;
        bool dPadDownPressed;
        bool dPadLeftPressed;
        bool dPadRightPressed;

        bool leftTriggerPressed;
        bool rightTriggerPressed;

        double rightStickY;

        double powerRetract;
        double powerExtend;
        double powerMain;

        double desiredRotatePos;
        double desiredMainPos;
        double desiredMainFirstPos;

    } state;

private:
    frc::XboxController *operatorController;

    WPI_TalonFX leadMainMotor;

    WPI_TalonFX followMainMotor;
    
    rev::CANSparkMax rotateMotor;

    rev::SparkMaxPIDController rotateMotorPidController = rotateMotor.GetPIDController();

    rev::SparkMaxRelativeEncoder rotateEncoder = rotateMotor.GetEncoder();

};


#endif