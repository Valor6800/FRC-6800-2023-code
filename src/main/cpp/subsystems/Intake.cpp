#include <iostream>
#include "subsystems/Intake.h"
//intaking with cone and cube has different directions, so flipped
#define DEFAULT_INTAKE_CUBE_SPD -0.7f
#define DEFAULT_INTAKE_CONE_SPD 0.9f
#define DEFAULT_OUTTAKE_SPD -0.7f
//since intaking with cone vs cube is different, outtaking follows the same rule
#define DEFAULT_OUTTAKE_CUBE_SPD 0.4f
#define DEFAULT_OUTTAKE_CONE_SPD -0.8f

#define DEFAULT_HOLD_SPD 0.05f

#define STALL_CURRENT 60.0f
#define CACHE_SIZE 30.0f

Intake::Intake(frc::TimedRobot *_robot) : ValorSubsystem(_robot, "Intake"),  
                            intakeMotor(CANIDs::INTAKE_LEAD_CAN, ValorNeutralMode::Coast, false, "baseCAN"),
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
   state.pieceState = CONE;
   currySensor.reset();
}

void Intake::init()
{
    state.intakeConeSpeed = DEFAULT_INTAKE_CONE_SPD;
    state.intakeCubeSpeed = DEFAULT_INTAKE_CUBE_SPD;
    state.outtakeSpeed = DEFAULT_OUTTAKE_SPD;
    state.outtakeConeSpeed = DEFAULT_OUTTAKE_CONE_SPD;
    state.outtakeCubeSpeed = DEFAULT_OUTTAKE_CUBE_SPD;
    state.holdSpeed = DEFAULT_HOLD_SPD;
    
    state.stallCurrent = STALL_CURRENT;

    currySensor.setSpikeSetpoint(state.stallCurrent);
    currySensor.setGetter([this]() { return intakeMotor.getCurrent(); });
    currySensor.setSpikeCallback([this]() { state.intakeState = SPIKED;});
    currySensor.setCacheSize(CACHE_SIZE);

    table->PutNumber("outtake speed", state.outtakeSpeed);
    table->PutNumber("outtake Cone speed", state.outtakeConeSpeed);
    table->PutNumber("outtake Cube speed", state.outtakeCubeSpeed);
    table->PutNumber("Hold Speed", state.holdSpeed);
    table->PutNumber("Stall Current", state.stallCurrent);
    table->PutNumber("intake cone speed", state.intakeConeSpeed);
    table->PutNumber("intake cube speed", state.intakeCubeSpeed);
    
    resetState();
}

void Intake::assessInputs()
{
    // SCORE
    if (driverGamepad->rightTriggerActive() || operatorGamepad->rightTriggerActive()) {
        // Operator holding cube score
        if (operatorGamepad->DPadDown() || operatorGamepad->DPadUp()
            || operatorGamepad->DPadLeft() || operatorGamepad->DPadRight() || driverGamepad->GetYButton()) {
            state.intakeState = OUTTAKE_CUBE;
        // Operator holding cone score
        } else{
            state.intakeState = OUTTAKE_CONE;
        }    
        // Driver/Operator scoring independently
    // No game element in intake, driver/operator requesting intake
    } else if (state.intakeState != SPIKED) {

        // Driver or operator ground pickup
        if (driverGamepad->GetLeftBumper() || driverGamepad->GetRightBumper() || operatorGamepad->leftTriggerActive() ||
            // Operator human player pickup
            operatorGamepad->GetXButton()  || operatorGamepad->DPadLeft()) {
            if (driverGamepad->GetYButton()){
                state.intakeState = INTAKE_CUBE;
                state.pieceState = CUBE;
            }
            else{
                state.intakeState = INTAKE_CONE;
                state.pieceState = CONE;
            }
        // Nothing pressed
        } else{
            state.intakeState = DISABLED;
        }

    // Game element in intake, do not allow intaking until button release
    } else if(state.intakeState == SPIKED) {

    // Should never happen
    } else {
        state.intakeState = DISABLED;
    }
}

void Intake::analyzeDashboard()
{
    state.outtakeSpeed = table->GetNumber("outtake speed", DEFAULT_OUTTAKE_SPD);
    state.outtakeConeSpeed = table->GetNumber("outtake Cone speed", DEFAULT_OUTTAKE_CONE_SPD);
    state.outtakeCubeSpeed =  table->GetNumber("outtake Cube speed", DEFAULT_OUTTAKE_CUBE_SPD);
    state.holdSpeed = table->GetNumber("Hold Speed", DEFAULT_HOLD_SPD);
    state.stallCurrent = table->GetNumber("Stall Current", STALL_CURRENT);
    state.intakeConeSpeed = table->GetNumber("intake cone speed", DEFAULT_INTAKE_CONE_SPD);
    state.intakeCubeSpeed = table->GetNumber("intake cube speed", DEFAULT_INTAKE_CUBE_SPD);
}
 
void Intake::assignOutputs()
{ 
    if (state.intakeState == DISABLED) {
        intakeMotor.setPower(0);
    } else if (state.intakeState == SPIKED) {
        if (state.pieceState == CONE){
            intakeMotor.setPower(state.holdSpeed);
        } else{
            intakeMotor.setPower(-state.holdSpeed);
        }

        
    } else if (state.intakeState == OUTTAKE_CONE){
        intakeMotor.setPower(state.outtakeConeSpeed);
    } else if (state.intakeState == OUTTAKE_CUBE){
        intakeMotor.setPower(state.outtakeCubeSpeed);
    } else if (state.intakeState == OUTTAKE){
        intakeMotor.setPower(state.outtakeSpeed);
    } else if (state.intakeState == INTAKE_CONE){
        intakeMotor.setPower(state.intakeConeSpeed);
    } else if (state.intakeState == INTAKE_CUBE){
        intakeMotor.setPower(state.intakeCubeSpeed);
    }
    
   
}

frc2::FunctionalCommand * Intake::getAutoCommand(std::string intakeState){
    Intake::IntakeStates inState = stringToIntakeState(intakeState);
    return new frc2::FunctionalCommand(
        // OnInit
        [&, inState]() {
            state.intakeState = inState;
        }, 
        //onExecute
        [&](){
            
        }, 
        [&](bool){
    
        }, // onEnd
        [&](){ //isFinished
            return true;
        },
        {}
    );
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
            // [this](double value) { state.outtakeConeSpeed = value; }
            nullptr
        );
        builder.AddDoubleProperty(
            "Outtake Cube Speed",
            [this]{ return state.outtakeCubeSpeed; },
            // [this](double value) { state.outtakeCubeSpeed = value; }
            nullptr
        );
        builder.AddDoubleProperty(
            "Outtake Speed",
            [this]{ return state.outtakeSpeed; },
            // [this](double value) { state.outtakeSpeed = value; }
            nullptr
        );
        builder.AddDoubleProperty(
            "Intake Cone Speed",
            [this]{ return state.intakeConeSpeed; },
            // [this](double value) { state.intakeSpeed = value; }
            nullptr
        );
        builder.AddDoubleProperty(
            "Intake Cube Speed",
            [this]{ return state.intakeCubeSpeed; },
            // [this](double value) { state.intakeSpeed = value; }
            nullptr
        );
        builder.AddDoubleProperty(
            "Hold Speed",
            [this]{ return state.holdSpeed; },
            // [this](double value) { state.holdSpeed = value; }
            nullptr
        );
        builder.AddDoubleProperty(
            "Stall Current",
            [this]{ return state.stallCurrent; },
            // [this](double value) {
            //     state.stallCurrent = value;
            //     currySensor.setSpikeSetpoint(state.stallCurrent);
            // }
            nullptr
        );
    }