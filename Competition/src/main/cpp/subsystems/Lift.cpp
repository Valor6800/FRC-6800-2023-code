#include "subsystems/Lift.h"

Lift::Lift() : ValorSubsystem(),
                operatorController(NULL),
                leadMainMotor{LiftConstants::MAIN_CAN_ID, rev::CANSparkMax::MotorType::kBrushless},
                followMainMotor{LiftConstants::MAIN_FOLLOW_CAN_ID, rev::CANSparkMax::MotorType::kBrushless},
                auxMotor{LiftConstants::AUX_CAN_ID, rev::CANSparkMax::MotorType::kBrushless},
                rotateMotor{LiftConstants::ROTATE_CAN_ID, rev::CANSparkMax::MotorType::kBrushless}
{
    frc2::CommandScheduler::GetInstance().RegisterSubsystem(this);
    init();
}

void Lift::init() {
    initTable("Lift");
    table->PutNumber("Lift Speed Extend", LiftConstants::DEFAULT_EXTEND_SPD);
    table->PutNumber("Lift Speed Retract", LiftConstants::DEFAULT_RETRACT_SPD);

    leadMainMotor.RestoreFactoryDefaults();
    followMainMotor.RestoreFactoryDefaults();

    leadMainMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    followMainMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);

    leadMainMotor.Follow(rev::CANSparkMax::kFollowerDisabled, false);
    followMainMotor.Follow(leadMainMotor);

    auxMotor.RestoreFactoryDefaults();
    rotateMotor.RestoreFactoryDefaults();

    auxMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    rotateMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
}

void Lift::setController(frc::XboxController *controller) {
    operatorController = controller;
}

void Lift::assessInputs() {
    if (!operatorController) {
        return;
    }

    state.leftStickY = std::abs(operatorController->GetLeftY()) < 0.05 ? 0 : operatorController->GetLeftY();

    state.xButtonPressed = operatorController->GetXButton();
    state.yButtonPressed = operatorController->GetYButton();

    state.dPadDownPressed = operatorController->GetPOV() == OIConstants::dpadDown;
    state.dPadUpPressed = operatorController->GetPOV() == OIConstants::dpadUp;

}

void Lift::analyzeDashboard() {
    state.powerRetract = table->GetNumber("Lift Speed Retract", LiftConstants::DEFAULT_RETRACT_SPD);
    state.powerExtend = table->GetNumber("Lift Speed Extend", LiftConstants::DEFAULT_EXTEND_SPD);
    
    state.powerAuxRetract = table->GetNumber("Aux Lift Speed Retract", LiftConstants::DEFAULT_AUX_RETRACT_SPD);
    state.powerAuxExtend = table->GetNumber("Aux Lift Speed Extend", LiftConstants::DEFAULT_AUX_EXTEND_SPD);
    
    state.powerRotate = table->GetNumber("Rotate Speed", LiftConstants::DEFAULT_ROTATE_SPD);
}

void Lift::assignOutputs() {

    if (state.liftstateMain == LiftMainState::LIFT_MAIN_DISABLED){
        leadMainMotor.Set(0);
    } else if (state.liftstateMain == LiftMainState::LIFT_MAIN_EXTEND){
        leadMainMotor.Set(LiftConstants::DEFAULT_EXTEND_SPD);
    } else if (state.liftstateMain == LiftMainState::LIFT_MAIN_RETRACT){
        leadMainMotor.Set(LiftConstants::DEFAULT_RETRACT_SPD);
    }

    if (state.liftstateAux == LiftAuxState::LIFT_AUX_DISABLED){
        auxMotor.Set(0);
    } else if (state.liftstateAux == LiftAuxState::LIFT_AUX_EXTEND){
        auxMotor.Set(LiftConstants::DEFAULT_AUX_EXTEND_SPD);
    } else if (state.liftstateAux == LiftAuxState::LIFT_AUX_RETRACT){
        auxMotor.Set(LiftConstants::DEFAULT_AUX_RETRACT_SPD);
    }

    if (state.liftstateRotate == LiftRotateState::LIFT_ROTATE_DISABLED){
        rotateMotor.Set(0);
    } else if (state.liftstateRotate == LiftRotateState::LIFT_ROTATE_ENABLE){
        rotateMotor.Set(state.leftStickY);
    } 
    
}

void Lift::resetState()
{
    state.liftstateMain = LiftMainState::LIFT_MAIN_DISABLED;
    state.liftstateAux = LiftAuxState::LIFT_AUX_DISABLED;
    state.liftstateRotate = LiftRotateState::LIFT_ROTATE_DISABLED;
}
