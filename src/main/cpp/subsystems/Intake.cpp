#include <iostream>
#include "subsystems/Intake.h"

Intake::Intake(frc::TimedRobot *_robot) : ValorSubsystem(_robot, "Intake"),  
                            intakeMotor(CANIDs::INTAKE_LEAD_CAN, rev::CANSparkMax::IdleMode::kCoast, false),
                            currySensor(_robot),
                            intakeSpeed(-1),
                            outtakeSpeed(1),
                            outtakeConeSpeed(-0.5),
                            outtakeCubeSpeed(-0.25),
                            spikeAmps(50)
{  
    frc2::CommandScheduler::GetInstance().RegisterSubsystem(this);
    init();
}

Intake::~Intake()
{  
}

void Intake::resetState()
{
   state.intakeState = DISABLED;
   currySensor.reset();
}

void Intake::init()
{
    intakeMotor.setupFollower(CANIDs::INTAKE_FOLLOW_CAN,true);
    currySensor.setSpikeSetpoint(spikeAmps);
    currySensor.setGetter([this]() { return intakeMotor.getCurrent(); });
    currySensor.setSpikeCallback([this]() { state.intakeState = SPIKED;});
    table->PutNumber("Intake Speed",intakeSpeed);
    table->PutNumber("Outtake Speed",outtakeSpeed);
    table->PutNumber("Outtake Cone Speed",outtakeConeSpeed);
    table->PutNumber("Outtake Cube Speed",outtakeCubeSpeed);
    table->PutNumber("Spiked Amperage",spikeAmps);
    resetState();
}

void Intake::assessInputs()
{
    if (driverGamepad->GetAButton()) {
        // works with elevarm locations in order to outtake at speed for cone/cube
        if (operatorGamepad->DPadDown() || operatorGamepad->DPadUp()
            || operatorGamepad->DPadLeft() || operatorGamepad->DPadRight()) {
            state.intakeState = OUTTAKE_CUBE;
        } else if (operatorGamepad->GetAButton() || operatorGamepad->GetBButton() 
                || operatorGamepad->GetXButton() || operatorGamepad->GetYButton()) {
            state.intakeState = OUTTAKE_CONE;
        } else {
            state.intakeState = OUTTAKE;
        }
    } else if (operatorGamepad->rightTriggerActive()) {
        state.intakeState = OUTTAKE;
    } else if (state.intakeState != SPIKED) {
        if (driverGamepad->GetLeftBumper() || driverGamepad->GetRightBumper() || operatorGamepad->leftTriggerActive()) {
            state.intakeState = INTAKE;
        } else{
            state.intakeState = DISABLED;
        }
    } else if(state.intakeState == SPIKED) {
        if (!driverGamepad->GetLeftBumper() && !driverGamepad->GetRightBumper() && !operatorGamepad->leftTriggerActive()) {
            state.intakeState = DISABLED;
        } 
    }
}

void Intake::analyzeDashboard()
{
    table->PutNumber("Intake Current", currySensor.getSensor());
    table->PutNumber("State", state.intakeState);
    intakeSpeed = table->GetNumber("Intake Speed", 0);
    outtakeSpeed = table->GetNumber("Outtake Speed", 0);
    outtakeConeSpeed = table->GetNumber("Outtake Cone Speed", 0);
    outtakeCubeSpeed = table->GetNumber("Outtake Cube Speed", 0);
    spikeAmps = table->GetNumber("Spiked Amperage", 0);
}
 
void Intake::assignOutputs()
{ 
    if (state.intakeState == DISABLED || state.intakeState == SPIKED){
        intakeMotor.setPower(0);
    } else if (state.intakeState == OUTTAKE_CONE){
        intakeMotor.setPower(outtakeConeSpeed);
    } else if (state.intakeState == OUTTAKE_CUBE){
        intakeMotor.setPower(outtakeCubeSpeed);
    } else if (state.intakeState == OUTTAKE){
        intakeMotor.setPower(outtakeSpeed);
    } else if (state.intakeState == INTAKE){
        intakeMotor.setPower(intakeSpeed);
    }
}

void Intake::InitSendable(wpi::SendableBuilder& builder)
    {
        builder.SetSmartDashboardType("Subsystem");

        builder.AddDoubleProperty(
            "state",
            [this] { return state.intakeState; },
            nullptr
        );
        builder.AddDoubleProperty(
            "pieceState",
            [this] { return state.pieceState; },
            nullptr
        );
    }