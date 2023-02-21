#include <iostream>
#include "subsystems/Intake.h"

#define DEFAULT_INTAKE_SPD 0.7f
#define DEFAULT_OUTTAKE_SPD -0.4f
#define DEFAULT_OUTTAKE_CONE_SPD -0.5f
#define DEFAULT_OUTTAKE_CUBE_SPD -0.3f
#define DEFAULT_HOLD_SPD 0.015f

#define STALL_CURRENT 20.0f

Intake::Intake(frc::TimedRobot *_robot) : ValorSubsystem(_robot, "Intake"),  
                            intakeMotor(CANIDs::INTAKE_LEAD_CAN, ValorNeutralMode::Coast, false),
                            currySensor(_robot, subsystemName)
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
    intakeMotor.setupFollower(CANIDs::INTAKE_FOLLOW_CAN, true);

    state.intakeSpeed = DEFAULT_INTAKE_SPD;
    state.outtakeSpeed = DEFAULT_OUTTAKE_SPD;
    state.outtakeConeSpeed = DEFAULT_OUTTAKE_CONE_SPD;
    state.outtakeCubeSpeed = DEFAULT_OUTTAKE_CUBE_SPD;
    state.holdSpeed = DEFAULT_HOLD_SPD;
    
    state.stallCurrent = STALL_CURRENT;

    currySensor.setSpikeSetpoint(state.stallCurrent);
    currySensor.setGetter([this]() { return intakeMotor.getCurrent(); });
    currySensor.setSpikeCallback([this]() { state.intakeState = SPIKED;});
    
    resetState();
}

void Intake::assessInputs()
{
    // SCORE
    if (driverGamepad->rightTriggerActive() || operatorGamepad->rightTriggerActive()) {
        // Operator holding cube score
        if (operatorGamepad->DPadDown() || operatorGamepad->DPadUp()
            || operatorGamepad->DPadLeft() || operatorGamepad->DPadRight()) {
            state.intakeState = OUTTAKE_CUBE;
        // Operator holding cone score
        } else if (operatorGamepad->GetAButton() || operatorGamepad->GetBButton() 
                || operatorGamepad->GetXButton() || operatorGamepad->GetYButton()) {
            state.intakeState = OUTTAKE_CONE;
        // Driver/Operator scoring independently
        } else {
            state.intakeState = OUTTAKE;
        }

    // No game element in intake, driver/operator requesting intake
    } else if (state.intakeState != SPIKED) {

        // Driver or operator ground pickup
        if (driverGamepad->GetLeftBumper() || driverGamepad->GetRightBumper() || operatorGamepad->leftTriggerActive() ||
            // Operator human player pickup
            operatorGamepad->GetXButton()  || operatorGamepad->DPadLeft()) {
            state.intakeState = INTAKE;

        // Nothing pressed
        } else{
            state.intakeState = DISABLED;
        }

    // Game element in intake, do not allow intaking until button release
    } else if(state.intakeState == SPIKED) {

        // Matches the intake buttons - once released, reset spiked
        if (!driverGamepad->GetLeftBumper() && !driverGamepad->GetRightBumper() && !operatorGamepad->leftTriggerActive() &&
            !operatorGamepad->GetXButton() && !operatorGamepad->DPadLeft()) {
            state.intakeState = DISABLED;
        }

    // Should never happen
    } else {
        state.intakeState = DISABLED;
    }
}

void Intake::analyzeDashboard()
{
}
 
void Intake::assignOutputs()
{ 
    if (state.intakeState == DISABLED) {
        intakeMotor.setPower(0);
    } else if (state.intakeState == SPIKED) {
        intakeMotor.setPower(state.holdSpeed);
    } else if (state.intakeState == OUTTAKE_CONE){
        intakeMotor.setPower(state.outtakeConeSpeed);
    } else if (state.intakeState == OUTTAKE_CUBE){
        intakeMotor.setPower(state.outtakeCubeSpeed);
    } else if (state.intakeState == OUTTAKE){
        intakeMotor.setPower(state.outtakeSpeed);
    } else if (state.intakeState == INTAKE){
        intakeMotor.setPower(state.intakeSpeed);
    }
}

void Intake::InitSendable(wpi::SendableBuilder& builder)
    {
        builder.SetSmartDashboardType("Subsystem");

        builder.AddDoubleProperty(
            "Intake State",
            [this]{ return state.intakeState; },
            nullptr
        );
        builder.AddDoubleProperty(
            "Outtake Cone Speed",
            [this]{ return state.outtakeConeSpeed; },
            [this](double value) { state.outtakeConeSpeed = value; }
        );
        builder.AddDoubleProperty(
            "Outtake Cube Speed",
            [this]{ return state.outtakeCubeSpeed; },
            [this](double value) { state.outtakeCubeSpeed = value; }
        );
        builder.AddDoubleProperty(
            "Outtake Speed",
            [this]{ return state.outtakeSpeed; },
            [this](double value) { state.outtakeSpeed = value; }
        );
        builder.AddDoubleProperty(
            "Intake Speed",
            [this]{ return state.intakeSpeed; },
            [this](double value) { state.intakeSpeed = value; }
        );
        builder.AddDoubleProperty(
            "Hold Speed",
            [this]{ return state.holdSpeed; },
            [this](double value) { state.holdSpeed = value; }
        );
        builder.AddDoubleProperty(
            "Stall Current",
            [this]{ return state.stallCurrent; },
            [this](double value) {
                state.stallCurrent = value;
                currySensor.setSpikeSetpoint(state.stallCurrent);
            }
        );
    }