#include "subsystems/Lift.h"

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
}

void Lift::setController(frc::XboxController *controller)
{
    operatorController = controller;
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
    else
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
        state.liftstateMain = LiftMainState::LIFT_MAIN_TOPOSITION;
    }
    else
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

    if (state.liftstateMain == LiftMainState::LIFT_MAIN_DISABLED) {
        leadMainMotor.Set(0);
    }
    else if (state.liftstateMain == LiftMainState::LIFT_MAIN_ENABLE) {
        if(state.rightStickY > OIConstants::kDeadbandY){
            leadMainMotor.Set(state.rightStickY * LiftConstants::DEFAULT_MAIN_EXTEND_SPD);
        }
        else if(state.rightStickY < (-1 * OIConstants::kDeadbandY)){
            leadMainMotor.Set(state.rightStickY * LiftConstants::DEFAULT_MAIN_RETRACT_SPD);
        }
    }
    else if (state.liftstateMain == LiftMainState::LIFT_MAIN_FIRSTPOSITION) {
        leadMainMotor.Set(ControlMode::MotionMagic, state.desiredMainFirstPos);

    }
    else if (state.liftstateMain == LiftMainState::LIFT_MAIN_TOPOSITION) {
        leadMainMotor.Set(ControlMode::MotionMagic, state.desiredMainPos);

    }
}

void Lift::resetState()
{
    state.liftstateMain = LiftMainState::LIFT_MAIN_DISABLED;
    state.liftstateRotate = LiftRotateState::LIFT_ROTATE_DISABLED;
}
