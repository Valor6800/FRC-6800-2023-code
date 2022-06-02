#pragma once

#include "ValorSubsystem.h"
#include "Constants.h"
#include "ValorGamepad.h"
#include <rev/CANSparkMax.h>

#include "ValorSubsystem.h"
#include "Constants.h"
#include <ctre/Phoenix.h>
#include <vector>

#include <rev/CANSparkMax.h>
#include <rev/CANEncoder.h>

#include <frc2/command/FunctionalCommand.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/ParallelCommandGroup.h>

#ifndef LIFT_H
#define LIFT_H

class Lift : public ValorSubsystem 
{
public:
    Lift();

    void init();
    void setController(ValorGamepad *controller);

    void assessInputs();
    void analyzeDashboard();
    void assignOutputs();

    void resetState();

    enum LiftRotateState {
        LIFT_ROTATE_DISABLED,
        LIFT_ROTATE_EXTEND,
        LIFT_ROTATE_RETRACT,
        LIFT_ROTATE_TOPOSITION,
        LIFT_ROTATE_ROTATEBAR
    };
        
    enum LiftMainState {
        LIFT_MAIN_DISABLED,
        LIFT_MAIN_ENABLE,
        LIFT_MAIN_TOPOSITION,
        LIFT_MAIN_FIRSTPOSITION,
        LIFT_MAIN_MAXPOS,
        LIFT_MAIN_DOWN,
        LIFT_MAIN_PULLUP,
        LIFT_MAIN_SLOWUP
    };

    enum LiftAutomationState {
        LIFT_AUTOMATION_DISABLED,
        LIFT_AUTOMATION_INITIAL_GRAB,
        LIFT_AUTOMATION_PULL_UP,
        LIFT_AUTOMATION_UPANDOUT
    };


    struct x
    {
        LiftMainState liftstateMain;
        LiftRotateState liftstateRotate;
        LiftAutomationState liftStateAutomation;

        double powerRetract;
        double powerExtend;
        double powerMain;

        double desiredRotatePos;
        double desiredMainPos;
        double desiredMainFirstPos;

    } state;

    double getExtensionEncoderValue();
    double getRotationEncoderValue();

    void setupCommands();

private:
    ValorGamepad *operatorController;

    WPI_TalonFX leadMainMotor;

    WPI_TalonFX followMainMotor;
    
    rev::CANSparkMax rotateMotor;

    rev::SparkMaxPIDController rotateMotorPidController = rotateMotor.GetPIDController();

    rev::SparkMaxRelativeEncoder rotateEncoder = rotateMotor.GetEncoder();

    frc2::SequentialCommandGroup liftSequenceUp;
    frc2::SequentialCommandGroup liftSequenceDown;
};

#endif