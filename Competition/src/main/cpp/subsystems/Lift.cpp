#include "subsystems/Lift.h"
#include <iostream>

Lift::Lift() : ValorSubsystem(),
               operatorController(NULL),
               leadMainMotor{LiftConstants::MAIN_CAN_ID},
               followMainMotor{LiftConstants::MAIN_FOLLOW_CAN_ID},
               rotateMotor{LiftConstants::ROTATE_CAN_ID, rev::CANSparkMax::MotorType::kBrushless}
{
    frc2::CommandScheduler::GetInstance().RegisterSubsystem(this);
    init();
}

void Lift::init()
{
    initTable("Lift");

    rotateMotor.RestoreFactoryDefaults();
    rotateMotor.SetInverted(true);

    rotateMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    rotateEncoder.SetPositionConversionFactor(360 * LiftConstants::pivotGearRatio);

    rotateMotor.EnableSoftLimit(rev::CANSparkMax::SoftLimitDirection::kForward, true);
    rotateMotor.SetSoftLimit(rev::CANSparkMax::SoftLimitDirection::kForward, LiftConstants::rotateForwardLimit);

    rotateMotor.EnableSoftLimit(rev::CANSparkMax::SoftLimitDirection::kReverse, true);
    rotateMotor.SetSoftLimit(rev::CANSparkMax::SoftLimitDirection::kReverse, LiftConstants::rotateReverseLimit);

    leadMainMotor.ConfigForwardSoftLimitThreshold(LiftConstants::extendForwardLimit);
    leadMainMotor.ConfigReverseSoftLimitThreshold(LiftConstants::extendReverseLimit);

    leadMainMotor.ConfigForwardSoftLimitEnable(true);
    leadMainMotor.ConfigReverseSoftLimitEnable(true);

    rotateEncoder.SetPosition(0);

    leadMainMotor.SetSelectedSensorPosition(0);
    leadMainMotor.SetInverted(true);
    followMainMotor.Follow(leadMainMotor);

    leadMainMotor.SetNeutralMode(ctre::phoenix::motorcontrol::Brake);
    followMainMotor.SetNeutralMode(ctre::phoenix::motorcontrol::Brake);

    rotateMotorPidController.SetP(LiftConstants::rotate_kP);
    rotateMotorPidController.SetI(LiftConstants::rotate_kI);
    rotateMotorPidController.SetD(LiftConstants::rotate_kD);
    rotateMotorPidController.SetIZone(LiftConstants::rotate_kIz);
    rotateMotorPidController.SetFF(LiftConstants::rotate_kFF);
    rotateMotorPidController.SetOutputRange(LiftConstants::rotate_kMinOutput, LiftConstants::rotate_kMaxOutput);

    rotateMotorPidController.SetSmartMotionMaxVelocity(LiftConstants::rotate_kMaxVel);
    rotateMotorPidController.SetSmartMotionMinOutputVelocity(LiftConstants::rotate_kMinVel);
    rotateMotorPidController.SetSmartMotionMaxAccel(LiftConstants::rotate_kMaxAcc);
    rotateMotorPidController.SetSmartMotionAllowedClosedLoopError(LiftConstants::rotate_kAllErr);

    leadMainMotor.ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor, 0, 10);
    leadMainMotor.ConfigAllowableClosedloopError(0, 0);
    leadMainMotor.Config_IntegralZone(0, 0);
    leadMainMotor.Config_kF(0, LiftConstants::main_KF);
    leadMainMotor.Config_kD(0, LiftConstants::main_KD);
    leadMainMotor.Config_kI(0, LiftConstants::main_KI);
    leadMainMotor.Config_kP(0, LiftConstants::main_KP);
    leadMainMotor.ConfigMotionAcceleration(LiftConstants::MAIN_MOTION_ACCELERATION);
    leadMainMotor.ConfigMotionCruiseVelocity(LiftConstants::MAIN_MOTION_CRUISE_VELOCITY);

    table->PutNumber("Rotate First Angle", LiftConstants::ROTATE_FIRST_POSITION);
    table->PutNumber("Main Lift First Pos", LiftConstants::MAIN_FIRST_POSITION);
    table->PutNumber("Main Lift Second Pos", LiftConstants::MAIN_SECOND_POSITION);

    setupCommands();
}

void Lift::setController(frc::XboxController *controller)
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
            return getExtensionEncoderValue() > (LiftConstants::MAIN_SECOND_POSITION - 2000);
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
            return getRotationEncoderValue() > (LiftConstants::ROTATE_FIRST_POSITION - 3);
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
            return getExtensionEncoderValue() > (LiftConstants::extendForwardLimit - 2000);
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
            return getRotationEncoderValue() < LiftConstants::ROTATE_BAR_POSITION;
        },
        {}
    );
    frc2::FunctionalCommand liftPullUp(
        [this]() { //onInit
            state.liftstateMain = Lift::LiftMainState::LIFT_MAIN_ENABLE;
        },
        [this](){}, //onExecute
        [this](bool){ //onEnd
            state.liftstateMain = Lift::LiftMainState::LIFT_MAIN_DISABLED;
        },
        [this](){ //isFinished
            return getExtensionEncoderValue() < 2000;
        },
        {}
    );
    frc2::ParallelCommandGroup grabBar(liftRotateIn, liftPullUp);

    liftSequence.AddCommands(liftExtend, liftRotateOut, liftExtend2, liftRotateIn);
}

void Lift::assessInputs()
{
    if (!operatorController)
    {
        return;
    }

    state.rightStickY = -1 * operatorController->GetRightY();

    state.dPadLeftPressed = operatorController->GetPOV() == OIConstants::dpadLeft;
    state.dPadRightPressed = operatorController->GetPOV() == OIConstants::dpadRight;
    state.dPadDownPressed = operatorController->GetPOV() == OIConstants::dpadDown;
    state.dPadUpPressed = operatorController->GetPOV() == OIConstants::dpadUp;

    state.leftTriggerPressed = operatorController->GetLeftTriggerAxis() > LiftConstants::kDeadBandTrigger;
    state.rightTriggerPressed = operatorController->GetRightTriggerAxis() > LiftConstants::kDeadBandTrigger;

    if((std::abs(state.rightStickY) > OIConstants::kDeadbandY) && liftSequence.IsScheduled()){
        liftSequence.Cancel();
        std::cout << "canceling" << std::endl;
    }

    if (state.dPadLeftPressed && leadMainMotor.GetSelectedSensorPosition() > LiftConstants::rotateNoLowerThreshold)
    {
        state.liftstateRotate = LiftRotateState::LIFT_ROTATE_EXTEND;
    }
    else if (state.dPadRightPressed && leadMainMotor.GetSelectedSensorPosition() > LiftConstants::rotateNoLowerThreshold)
    {
        state.liftstateRotate = LiftRotateState::LIFT_ROTATE_RETRACT;
    }
    else if (state.dPadDownPressed && leadMainMotor.GetSelectedSensorPosition() > LiftConstants::rotateNoLowerThreshold) {
        state.liftstateRotate = LiftRotateState::LIFT_ROTATE_TOPOSITION;
    }
    else if(!liftSequence.IsScheduled())
    {
        state.liftstateRotate = LiftRotateState::LIFT_ROTATE_DISABLED;
    }

    if (std::abs(state.rightStickY) > OIConstants::kDeadbandY)
    {
        state.liftstateMain = LiftMainState::LIFT_MAIN_ENABLE;
    }
    else if (state.leftTriggerPressed && state.rightTriggerPressed) {
        state.liftstateMain = LiftMainState::LIFT_MAIN_FIRSTPOSITION;
    }
    else if (state.dPadUpPressed) {
        liftSequence.Schedule();
    }
    else if(!liftSequence.IsScheduled())
    {
        state.liftstateMain = LiftMainState::LIFT_MAIN_DISABLED;
    }
}

double Lift::getExtensionEncoderValue()
{
    return leadMainMotor.GetSelectedSensorPosition();
}

double Lift::getRotationEncoderValue()
{
    return rotateEncoder.GetPosition();
}

void Lift::analyzeDashboard()
{
    state.powerMain = table->GetNumber("Rotate Speed", LiftConstants::DEFAULT_EXTEND_SPD);

    table->PutNumber("Lift Main Encoder Value", getExtensionEncoderValue());
    table->PutNumber("Lift Rotate Encoder Value", getRotationEncoderValue());

    state.desiredRotatePos = table->GetNumber("Rotate First Angle", LiftConstants::ROTATE_FIRST_POSITION);
    state.desiredMainPos = table->GetNumber("Main Lift Second Pos", LiftConstants::MAIN_SECOND_POSITION);
    state.desiredMainFirstPos = table->GetNumber("Main Lift First Pos", LiftConstants::MAIN_FIRST_POSITION);
}

void Lift::assignOutputs()
{
    if (state.liftstateRotate == LiftRotateState::LIFT_ROTATE_DISABLED)
    {
        rotateMotor.Set(0);
    }
    else if (state.liftstateRotate == LiftRotateState::LIFT_ROTATE_EXTEND)
    {
        rotateMotor.Set(LiftConstants::DEFAULT_EXTEND_SPD);
    }
    else if (state.liftstateRotate == LiftRotateState::LIFT_ROTATE_RETRACT)
    {
        rotateMotor.Set(LiftConstants::DEFAULT_RETRACT_SPD);
    }
    else if (state.liftstateRotate == LiftRotateState::LIFT_ROTATE_TOPOSITION) {
        rotateMotorPidController.SetReference(state.desiredRotatePos, rev::ControlType::kSmartMotion);
    }
    else if (state.liftstateRotate == LiftRotateState::LIFT_ROTATE_ROTATEBAR) {
        rotateMotorPidController.SetReference(LiftConstants::ROTATE_BAR_POSITION, rev::ControlType::kSmartMotion);
    }

    if (state.liftstateMain == LiftMainState::LIFT_MAIN_DISABLED) {
        leadMainMotor.Set(0);
    }
    else if (state.liftstateMain == LiftMainState::LIFT_MAIN_ENABLE) {
        if(state.rightStickY > OIConstants::kDeadbandY){
            leadMainMotor.Set(state.rightStickY * LiftConstants::DEFAULT_MAIN_EXTEND_SPD);
        }
        else if(state.rightStickY < (-1 * OIConstants::kDeadbandY)){
            leadMainMotor.Set(state.rightStickY * LiftConstants::DEFAULT_MAIN_RETRACT_SPD);
            rotateMotor.Set(-.1);
        }
    }
    else if (state.liftstateMain == LiftMainState::LIFT_MAIN_FIRSTPOSITION) {
        leadMainMotor.Set(ControlMode::MotionMagic, state.desiredMainFirstPos);

    }
    else if (state.liftstateMain == LiftMainState::LIFT_MAIN_TOPOSITION) {
        leadMainMotor.Set(ControlMode::MotionMagic, state.desiredMainPos);
    }
    else if (state.liftstateMain == LiftMainState::LIFT_MAIN_MAXPOS) {
        leadMainMotor.Set(ControlMode::MotionMagic, LiftConstants::extendForwardLimit);
    }
    else if (state.liftstateMain == LiftMainState::LIFT_MAIN_DOWN) {
        leadMainMotor.Set(ControlMode::MotionMagic, LiftConstants::MAIN_DOWN_POSITION);
        rotateMotor.Set(-.1);
    }
}

void Lift::resetState()
{
    state.liftstateMain = LiftMainState::LIFT_MAIN_DISABLED;
    state.liftstateRotate = LiftRotateState::LIFT_ROTATE_DISABLED;
}
