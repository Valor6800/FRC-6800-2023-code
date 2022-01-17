#include "subsystems/Lift.h"


Lift::Lift() : ValorSubsystem(),
                           operatorController(NULL),
                           leadMainMotor{LiftConstants::MAIN_CAN_ID, rev::CANSparkMax::MotorType::kBrushless},
                           followMainMotor{LiftConstants::MAIN_FOLLOW_CAN_ID, rev::CANSparkMax::MotorType::kBrushless},
                           auxMotor{IntakeConstants::AUX_CAN_ID, rev::CANSparkMax::MotorType::kBrushless},
                           rotateMotor{IntakeConstants::ROTATE_CAN_ID, rev::CANSparkMax::MotorType::kBrushless},

{
    frc2::CommandScheduler::GetInstance().RegisterSubsystem(this);
    init();
}

Lift::~Lift()

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

    
}

void Lift::setController(frc::XboxController *controller) {
    operatorontroller = controller;
}

void Lift::assessInputs() {
if (!operatorController) {
        return;
    }

    state.operator_leftStickY = std::abs(operatorController->GetY(frc::GenericHID::kLefttHand)) < 0.05 ? 0 : operatorController->GetY(frc::GenericHID::kLeftHand);

    state.operator_xButtonPressed = operatorController->GetXButton();
    state.operator_yButtonPressed = operatorController->GetYButton();

    operatorController->GetPOV() == OIConstants::dpadDown ? state.operator_dPadDownPressed = true : state.operator_dPadDownPressed = false;
    operatorController->GetPOV() == OIConstants::dpadUp ? state.operator_dPadUpPressed = true : state.operator_dPadUpPressed = false;

}

void Lift::analyzeDashboard() {
    state.powerRetract = table->GetNumber("Lift Speed Retract", LiftConstants::DEFAULT_RETRACT_SPD);
    state.powerExtend = table->GetNumber("Lift Speed Extend", LiftConstants::DEFAULT_EXTEND_SPD);
    
    state.powerAuxRetract = table->GetNumber("Aux Lift Speed Retract", LiftConstants::DEFAULT_AUX_RETRACT_SPD);
    state.powerAuxExtend = table->GetNumber("Aux Lift Speed Extend", LiftConstants::DEFAULT_AUX_EXTEND_SPD);
    
    state.powerRotate = table->GetNumber("Rotate Speed", LiftConstants::DEFAULT_ROTATE_SPD);
}

void Lift::assignOutputs() {
    if (state.liftStateMain == LiftMainState::Disabled){
        leadMainMotor.Set(0);
        followMainMotor.Set(0);
    } else if (state.liftStateMain == LiftMainState::Extend){
        leadMainMotor.set(LiftConstants::DEFAULT_EXTEND_SPD);
        followMainMotor.set(LiftConstants::DEFAULT_EXTEND_SPD);
    } else if (state.liftStateMain == LiftMainState::Retract){
        leadMainMotor.set(LiftConstants::DEFAULT_RETRACT_SPD);
        followMainMotor.set(LiftConstants::DEFAULT_RETRACT_SPD);
    }

    if (state.liftStateAux == LiftAuxState::Disabled){
        AuxMotor.Set(0);
    } else if (state.liftStateAux == LiftAuxState::Extend){
        AuxMotor.set(LiftConstants::DEFAULT_AUX_EXTEND_SPD);
    } else if (state.liftStateAux == LiftAuxState::Retract){
        AuxMotor.set(LiftConstants::DEFAULT_AUX_RETRACT_SPD);
    }

    if (state.liftStateRotate == LiftRotateState::Disabled){
        RotateMotor.Set(0);
    } else if (state.liftStateRotate == LiftRotateState::Rotate){
        RotateMotor.set(LiftConstants::DEFAULT_ROTATE_SPD);
    } 
    
}



void Lift::resetState()
{

}
