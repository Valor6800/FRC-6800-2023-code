#include <iostream>
#include "subsystems/Intake.h"
#include "subsystems/Piece.h"
//intaking with cone and cube has different directions, so flipped
#define DEFAULT_INTAKE_CUBE_SPD -0.7f
#define DEFAULT_INTAKE_CONE_SPD 0.9f
#define DEFAULT_OUTTAKE_SPD -0.7f
//since intaking with cone vs cube is different, outtaking follows the same rule
#define DEFAULT_OUTTAKE_CUBE_SPD 0.4f
#define DEFAULT_OUTTAKE_CONE_SPD -0.8f

#define CONE_HOLD_SPD 0.02f
#define CUBE_HOLD_SPD -0.03f

#define CUBE_SPIKED_CURRENT 60.0f
#define CONE_SPIKED_CURRENT 60.0f

#define CUBE_CACHE_SIZE 120.0f
#define CONE_CACHE_SIZE 250.0f

Intake::Intake(frc::TimedRobot *_robot) : ValorSubsystem(_robot, "Intake"),  
                            intakeMotor(CANIDs::INTAKE_LEAD_CAN, ValorNeutralMode::Coast, false, "baseCAN"),
                            currentSensor(_robot, subsystemName)

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
   state.pieceState = Piece::CONE;
   currentSensor.reset();
}

void Intake::init()
{
    state.intakeConeSpeed = DEFAULT_INTAKE_CONE_SPD;
    state.intakeCubeSpeed = DEFAULT_INTAKE_CUBE_SPD;
    state.outtakeSpeed = DEFAULT_OUTTAKE_SPD;
    state.outtakeConeSpeed = DEFAULT_OUTTAKE_CONE_SPD;
    state.outtakeCubeSpeed = DEFAULT_OUTTAKE_CUBE_SPD;
    state.coneHoldSpeed = CONE_HOLD_SPD;
    state.cubeHoldSpeed = CUBE_HOLD_SPD;
    
    state.cubeSpikeCurrent = CUBE_SPIKED_CURRENT;
    state.coneSpikeCurrent = CONE_SPIKED_CURRENT;

    state.coneCacheSize = CONE_CACHE_SIZE;
    state.cubeCacheSize = CUBE_CACHE_SIZE;

    state.intakeOp = false;

    prevState = state;

    currentSensor.setSpikeSetpoint(state.coneSpikeCurrent);
    currentSensor.setGetter([this]() { return intakeMotor.getCurrent(); });
    currentSensor.setSpikeCallback([this]() { state.intakeState = SPIKED;});
    currentSensor.setCacheSize(CONE_CACHE_SIZE);

    table->PutNumber("outtake speed", state.outtakeSpeed);
    table->PutNumber("outtake Cone speed", state.outtakeConeSpeed);
    table->PutNumber("outtake Cube speed", state.outtakeCubeSpeed);
    table->PutNumber("Cone Hold Speed", state.coneHoldSpeed);
    table->PutNumber("Cube Hold Speed", state.cubeHoldSpeed);
    table->PutNumber("Cone Spiked Current", state.coneSpikeCurrent);
    table->PutNumber("Cube Spiked Current", state.cubeSpikeCurrent);
    table->PutNumber("Cone Cache Size", state.coneCacheSize);
    table->PutNumber("Cube Cache Size", state.cubeCacheSize);

    intakeMotor.getMotor()->ConfigSupplyCurrentLimit(SupplyCurrentLimitConfiguration(true, 40, 60, .75));

    
    resetState();
}

void Intake::assessInputs()
{
    // SCORE
    if (driverGamepad->rightTriggerActive() || operatorGamepad->rightTriggerActive()) {
        // Operator holding cube score
        if (operatorGamepad->GetYButton() || driverGamepad->GetYButton()) {
            state.intakeState = OUTTAKE_CUBE;
        // Operator holding cone score
        } else{
            state.intakeState = OUTTAKE_CONE;
        }    
        // Driver/Operator scoring independently
    // No game element in intake, driver/operator requesting intake
    } else if (state.intakeState != SPIKED) {

        // Driver or operator ground pickup
        if (driverGamepad->GetLeftBumper() || driverGamepad->GetRightBumper() || 
        (operatorGamepad->DPadLeft() && driverGamepad->leftTriggerActive())) {
            
            if (driverGamepad->GetYButton() || operatorGamepad->GetYButton()){
                state.intakeState = INTAKE_CUBE;
                state.pieceState = Piece::CUBE;
            }
            else{
                state.intakeState = INTAKE_CONE;
                state.pieceState = Piece::CONE;
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

    if(operatorGamepad->leftTriggerActive()){
        state.intakeOp = true;
    } else{
        state.intakeOp = false;
    }
}

void Intake::analyzeDashboard()
{
    state.outtakeSpeed = table->GetNumber("outtake speed", DEFAULT_OUTTAKE_SPD);
    state.outtakeConeSpeed = table->GetNumber("outtake Cone speed", DEFAULT_OUTTAKE_CONE_SPD);
    state.outtakeCubeSpeed =  table->GetNumber("outtake Cube speed", DEFAULT_OUTTAKE_CUBE_SPD);
    state.cubeHoldSpeed = table->GetNumber("Cube Hold Speed", CUBE_HOLD_SPD);
    state.coneHoldSpeed = table->GetNumber("Cone Hold Speed", CONE_HOLD_SPD);
    state.coneSpikeCurrent = table->GetNumber("Cone Spiked Current", CONE_SPIKED_CURRENT);
    state.cubeSpikeCurrent = table->GetNumber("Cube Spiked Current", CUBE_SPIKED_CURRENT);
    state.coneCacheSize = table->GetNumber("Cone Cache Size", CONE_CACHE_SIZE);
    state.cubeCacheSize = table->GetNumber("Cube Cache Size", CUBE_CACHE_SIZE);
    state.intakeConeSpeed = table->GetNumber("intake cone speed", DEFAULT_INTAKE_CONE_SPD);
    state.intakeCubeSpeed = table->GetNumber("intake cube speed", DEFAULT_INTAKE_CUBE_SPD);
}
 
void Intake::assignOutputs()
{ 
    if (state.intakeOp){
         if (state.pieceState ==Piece::CONE){
            intakeMotor.setPower(state.intakeConeSpeed);
        } else{
            intakeMotor.setPower(state.intakeCubeSpeed);
        }
    } else if (state.intakeState == DISABLED) {
        intakeMotor.setPower(0);
    } else if (state.intakeState == SPIKED) {
        if (state.pieceState ==Piece::CONE){
            intakeMotor.setPower(state.coneHoldSpeed);
        } else{
            intakeMotor.setPower(state.cubeHoldSpeed);
        }
    } else if (state.intakeState == OUTTAKE_CONE){
        intakeMotor.setPower(state.outtakeConeSpeed);
    } else if (state.intakeState == OUTTAKE_CUBE){
        intakeMotor.setPower(state.outtakeCubeSpeed);
    } else if (state.intakeState == OUTTAKE){
        intakeMotor.setPower(state.outtakeSpeed);
    } else if (state.intakeState == INTAKE_CONE){
        if (prevState.pieceState !=Piece::CONE) {
            currentSensor.setCacheSize(state.coneCacheSize);
            currentSensor.setSpikeSetpoint(state.coneSpikeCurrent);
        }
        intakeMotor.setPower(state.intakeConeSpeed);
    } else if (state.intakeState == INTAKE_CUBE){
        if (prevState.pieceState != Piece::CUBE) {
            currentSensor.setCacheSize(state.cubeCacheSize);
            currentSensor.setSpikeSetpoint(state.cubeSpikeCurrent);
        }
        intakeMotor.setPower(state.intakeCubeSpeed);
    }
    prevState = state;
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
            "Cone Cache Size",
            [this]{ return state.coneCacheSize; },
            // [this](double value) { state.intakeSpeed = value; }
            nullptr
        );
        builder.AddDoubleProperty(
            "Cube Cache Size",
            [this]{ return state.cubeCacheSize; },
            // [this](double value) { state.intakeSpeed = value; }
            nullptr
        );
        builder.AddDoubleProperty(
            "Cube Hold Speed",
            [this]{ return state.cubeHoldSpeed; },
            // [this](double value) { state.holdSpeed = value; }
            nullptr
        );
        builder.AddDoubleProperty(
            "Cone Hold Speed",
            [this]{ return state.coneHoldSpeed; },
            // [this](double value) { state.holdSpeed = value; }
            nullptr
        );
        builder.AddDoubleProperty(
            "Cube Spike Current",
            [this]{ return state.cubeSpikeCurrent; },
            // [this](double value) {
            //     state.stallCurrent = value;
            //     cubeCurrentSensor.setSpikeSetpoint(state.stallCurrent);
            // }
            nullptr
        );
        builder.AddDoubleProperty(
            "Cone Spike Current",
            [this]{ return state.coneSpikeCurrent; },
            // [this](double value) {
            //     state.stallCurrent = value;
            //     cubeCurrentSensor.setSpikeSetpoint(state.stallCurrent);
            // }
            nullptr
        );
    }