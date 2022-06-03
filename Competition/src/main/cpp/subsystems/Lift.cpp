#include "subsystems/Lift.h"
#include <iostream>

#define LIFT_EXTEND_POS_BOTTOM 0
#define LIFT_EXTEND_POS_DOWN 0.06
#define LIFT_EXTEND_POS_SLOW 2.69
#define LIFT_EXTEND_POS_FIRST 30.27
#define LIFT_EXTEND_POS_SECOND 38.33
#define LIFT_EXTEND_POS_NO_ROTATE 37.84
#define LIFT_EXTEND_POS_MAX 47.85

#define LIFT_ROTATE_POS_BAR 28
#define LIFT_ROTATE_POS_FIRST 40

#define LIFT_ROTATE_LIMIT_REVERSE 0
#define LIFT_ROTATE_LIMIT_FORWARD 40

#define LIFT_EXTEND_SPD 0.7
#define LIFT_RETRACT_SPD -0.7

#define LIFT_ROTATE_MAX_SPEED 0.5
#define LIFT_EXTEND_MAX_SPEED 0.5

Lift::Lift() : ValorSubsystem(),
               operatorController(NULL),
               leadMotorController(CANIDs::LIFT_EXTEND, Brake, true),
               rotateMotorController(CANIDs::LIFT_ROTATE, rev::CANSparkMax::IdleMode::kBrake, true)
{
    frc2::CommandScheduler::GetInstance().RegisterSubsystem(this);
    init();
}

void Lift::init()
{
    initTable("Lift");

    leadMotorController.setLimits(LIFT_EXTEND_POS_DOWN, LIFT_EXTEND_POS_MAX);
    leadMotorController.setupFollower(CANIDs::LIFT_FOLLOW);

    PIDF rotatePIDF;
    rotatePIDF.P = 5e-5;
    rotatePIDF.F = 0.000156/2;
    rotatePIDF.velocity = 10000;
    rotatePIDF.acceleration = rotatePIDF.velocity * 2;

    rotateMotorController.setPIDF(0, rotatePIDF);
    rotateMotorController.setRange(0, -LIFT_ROTATE_MAX_SPEED, LIFT_ROTATE_MAX_SPEED);
    rotateMotorController.setLimits(LIFT_ROTATE_LIMIT_REVERSE, LIFT_ROTATE_LIMIT_FORWARD);
    rotateMotorController.setConversion(360.0/95.67); // Convert to degrees (utilizing gear ratio)

    table->PutNumber("Rotate First Angle", LIFT_ROTATE_POS_FIRST);
    table->PutNumber("Main Lift First Pos", LIFT_EXTEND_POS_FIRST);
    table->PutNumber("Main Lift Second Pos", LIFT_EXTEND_POS_SECOND);

    setupCommands();
}

void Lift::setController(ValorGamepad *controller)
{
    operatorController = controller;
}

void Lift::setupCommands()
{
    frc2::FunctionalCommand liftExtend(
        [this]() { //onInit
            state.liftstateMain = Lift::LiftMainState::LIFT_MAIN_TOPOSITION;
        },
        [this](){}, //onExecute
        [this](bool){ //onEnd
            state.liftstateMain = Lift::LiftMainState::LIFT_MAIN_DISABLED;
        },
        [this](){ //isFinished
            return getExtensionEncoderValue() > (LIFT_EXTEND_POS_SECOND - 2000);
        },
        {}
    );
    frc2::FunctionalCommand liftRotateOut(
        [this]() {
            state.liftstateRotate = Lift::LiftRotateState::LIFT_ROTATE_TOPOSITION;
        },
        [this](){},
        [this](bool){
            state.liftstateRotate = Lift::LiftRotateState::LIFT_ROTATE_DISABLED;
        },
        [this](){
            return getRotationEncoderValue() > (LIFT_ROTATE_POS_FIRST - 3);
        },
        {}
    );
    frc2::FunctionalCommand liftExtend2(
        [this]() { //onInit
            state.liftstateMain = Lift::LiftMainState::LIFT_MAIN_MAXPOS;
        },
        [this](){}, //onExecute
        [this](bool){ //onEnd
            state.liftstateMain = Lift::LiftMainState::LIFT_MAIN_DISABLED;
        },
        [this](){ //isFinished
            return getExtensionEncoderValue() > (LIFT_EXTEND_POS_MAX - 2000);
        },
        {}
    );
    frc2::FunctionalCommand liftRotateIn(
        [this]() { //onInit
            state.liftstateRotate = Lift::LiftRotateState::LIFT_ROTATE_ROTATEBAR;
        },
        [this](){}, //onExecute
        [this](bool){ //onEnd
            state.liftstateRotate = Lift::LiftRotateState::LIFT_ROTATE_DISABLED;
        },
        [this](){ //isFinished
            return getRotationEncoderValue() < LIFT_ROTATE_POS_BAR + 2;
        },
        {}
    );
    // frc2::FunctionalCommand liftPullUp(
    //     [this]() { //onInit
    //         state.liftstateMain = Lift::LiftMainState::LIFT_MAIN_ENABLE;
    //     },
    //     [this](){}, //onExecute
    //     [this](bool){ //onEnd
    //         state.liftstateMain = Lift::LiftMainState::LIFT_MAIN_DISABLED;
    //     },
    //     [this](){ //isFinished
    //         return getExtensionEncoderValue() < 2000;
    //     },
    //     {}
    // );
       frc2::FunctionalCommand liftPullUpStop(
        [this]() { //onInit
            state.liftstateMain = Lift::LiftMainState::LIFT_MAIN_PULLUP;
        },
        [this](){}, //onExecute
        [this](bool){ //onEnd
            state.liftstateMain = Lift::LiftMainState::LIFT_MAIN_DISABLED;
        },
        [this](){ //isFinished
            return getExtensionEncoderValue() < LIFT_EXTEND_POS_BOTTOM + 1000;
        },
        {}
    );
    frc2::FunctionalCommand liftMainSlowUp(
        [this]() { //onInit
            state.liftstateMain = Lift::LiftMainState::LIFT_MAIN_SLOWUP;
        },
        [this](){}, //onExecute
        [this](bool){ //onEnd
            state.liftstateMain = Lift::LiftMainState::LIFT_MAIN_DISABLED;
        },
        [this](){ //isFinished
            return getExtensionEncoderValue() > LIFT_EXTEND_POS_SLOW;
        },
        {}
    );
    //frc2::ParallelCommandGroup grabBar(liftRotateIn, liftPullUp);

    liftSequenceUp.AddCommands(liftExtend, liftRotateOut, liftExtend2, liftRotateIn);
    liftSequenceDown.AddCommands(liftPullUpStop);//, liftMainSlowUp);
}

void Lift::assessInputs()
{
    if (!operatorController)
    {
        return;
    }

    if(operatorController->rightStickYActive() && liftSequenceUp.IsScheduled()){
        liftSequenceUp.Cancel();
    }
    if(operatorController->rightStickYActive() && liftSequenceDown.IsScheduled()){
        liftSequenceDown.Cancel();
    }

    if (operatorController->DPadLeft() && leadMotorController.getPosition() > LIFT_EXTEND_POS_NO_ROTATE)
    {
        state.liftstateRotate = LiftRotateState::LIFT_ROTATE_EXTEND;
    }
    else if (operatorController->DPadRight() && leadMotorController.getPosition() > LIFT_EXTEND_POS_NO_ROTATE)
    {
        state.liftstateRotate = LiftRotateState::LIFT_ROTATE_RETRACT;
    }
    else if(!liftSequenceUp.IsScheduled() && !liftSequenceDown.IsScheduled())
    {
        state.liftstateRotate = LiftRotateState::LIFT_ROTATE_DISABLED;
    }

    if (operatorController->rightStickYActive())
    {
        state.liftstateMain = LiftMainState::LIFT_MAIN_ENABLE;
    }
    else if (operatorController->leftTriggerActive() && operatorController->rightTriggerActive()) {
        state.liftstateMain = LiftMainState::LIFT_MAIN_FIRSTPOSITION;
    }
    else if(!liftSequenceUp.IsScheduled() && !liftSequenceDown.IsScheduled())
    {
        state.liftstateMain = LiftMainState::LIFT_MAIN_DISABLED;
    }

    if (operatorController->DPadDown()){
        liftSequenceDown.Schedule();
    }
    else if (operatorController->DPadUp()) {
        liftSequenceUp.Schedule();
    }

    if (liftSequenceDown.IsScheduled()){
        state.liftStateAutomation = LiftAutomationState::LIFT_AUTOMATION_PULL_UP;
    }
    else if (liftSequenceUp.IsScheduled()){
        state.liftStateAutomation = LiftAutomationState::LIFT_AUTOMATION_UPANDOUT;
    }
    else if (leadMotorController.getPosition() > LIFT_EXTEND_POS_SLOW){
        state.liftStateAutomation = LiftAutomationState::LIFT_AUTOMATION_INITIAL_GRAB;
    }
    else{
        state.liftStateAutomation = LiftAutomationState::LIFT_AUTOMATION_DISABLED;
    }
}

double Lift::getExtensionEncoderValue()
{
    return leadMotorController.getPosition();
}

double Lift::getRotationEncoderValue()
{
    return rotateMotorController.getPosition();
}

void Lift::analyzeDashboard()
{
    state.powerMain = table->GetNumber("Rotate Speed", LIFT_ROTATE_MAX_SPEED);

    table->PutNumber("Lift Main Encoder Value", getExtensionEncoderValue());
    table->PutNumber("Lift Rotate Encoder Value", getRotationEncoderValue());
    table->PutNumber("Lift rotate current draw", rotateMotorController.getMotor()->GetOutputCurrent());

    table->PutNumber("Lift Automation State", state.liftStateAutomation);

    state.desiredRotatePos = table->GetNumber("Rotate First Angle", LIFT_ROTATE_POS_FIRST);
    state.desiredMainPos = table->GetNumber("Main Lift Second Pos", LIFT_EXTEND_POS_SECOND);
    state.desiredMainFirstPos = table->GetNumber("Main Lift First Pos", LIFT_EXTEND_POS_FIRST);
}

void Lift::assignOutputs()
{
    if (state.liftstateRotate == LiftRotateState::LIFT_ROTATE_DISABLED &&
        !(state.liftstateMain == LiftMainState::LIFT_MAIN_ENABLE &&
        operatorController->rightStickYActive() &&
        operatorController->rightStickY() < 0))
    {
        rotateMotorController.setPower(0);
    }
    else if (state.liftstateRotate == LiftRotateState::LIFT_ROTATE_EXTEND)
    {
        rotateMotorController.setPower(LIFT_ROTATE_MAX_SPEED);
    }
    else if (state.liftstateRotate == LiftRotateState::LIFT_ROTATE_RETRACT)
    {
        rotateMotorController.setPower(-LIFT_ROTATE_MAX_SPEED);
    }
    else if (state.liftstateRotate == LiftRotateState::LIFT_ROTATE_TOPOSITION) {
        rotateMotorController.setPosition(state.desiredRotatePos);
    }
    else if (state.liftstateRotate == LiftRotateState::LIFT_ROTATE_ROTATEBAR) {
        rotateMotorController.setPosition(LIFT_ROTATE_POS_BAR);
    }

    if (state.liftstateMain == LiftMainState::LIFT_MAIN_DISABLED) {
        leadMotorController.setPower(0);
    }
    else if (state.liftstateMain == LiftMainState::LIFT_MAIN_ENABLE) {
        if (operatorController->rightStickYActive() && operatorController->rightStickY() > 0) {
            leadMotorController.setPower(operatorController->rightStickY() * LIFT_EXTEND_SPD);
        }
        else if (operatorController->rightStickYActive() && operatorController->rightStickY() < 0) {
            leadMotorController.setPower(operatorController->rightStickY() * -LIFT_RETRACT_SPD);
            rotateMotorController.setPower(-0.2);
        }
    }
    else if (state.liftstateMain == LiftMainState::LIFT_MAIN_FIRSTPOSITION) {
        leadMotorController.setPosition(state.desiredMainFirstPos);
    }
    else if (state.liftstateMain == LiftMainState::LIFT_MAIN_TOPOSITION) {
        leadMotorController.setPosition(state.desiredMainPos);
    }
    else if (state.liftstateMain == LiftMainState::LIFT_MAIN_MAXPOS) {
        leadMotorController.setPosition(LIFT_EXTEND_POS_MAX);
    }
    else if (state.liftstateMain == LiftMainState::LIFT_MAIN_PULLUP){
        leadMotorController.setPosition(LIFT_EXTEND_POS_BOTTOM);
        rotateMotorController.setPower(-0.2);
    }
    else if (state.liftstateMain == LiftMainState::LIFT_MAIN_SLOWUP){
        leadMotorController.setPower(0.05);
    }
}

void Lift::resetState()
{
    state.liftstateMain = LiftMainState::LIFT_MAIN_DISABLED;
    state.liftstateRotate = LiftRotateState::LIFT_ROTATE_DISABLED;
    state.liftStateAutomation = LiftAutomationState::LIFT_AUTOMATION_DISABLED;
}
