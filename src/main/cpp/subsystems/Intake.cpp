#include <iostream>
#include "subsystems/Intake.h"
#include "subsystems/Piece.h"

#define IS_COMP

#ifdef IS_COMP
#define CONE_HOLD_SPD 0.04f
#define CUBE_HOLD_SPD -0.04f

#define AUTO_CONE_HOLD_SPD 0.15f
#else
#define CONE_HOLD_SPD 0.03f
#define CUBE_HOLD_SPD -0.03f

#define AUTO_CONE_HOLD_SPD 0.06f
#endif

//intaking with cone and cube has different directions, so flipped
#define DEFAULT_INTAKE_CUBE_SPD -0.7f
#define DEFAULT_INTAKE_CONE_SPD 1.0f
//since intaking with cone vs cube is different, outtaking follows the same rule
#define DEFAULT_OUTTAKE_CUBE_SPD 0.55f
#define DEFAULT_OUTTAKE_CONE_SPD -0.8f
#define GROUND_OUTTAKE_GROUND_CUBE_SPD 0.3f

#define CUBE_SPIKED_CURRENT 60.0f
#define CONE_SPIKED_CURRENT 90.0f

#define CUBE_CACHE_SIZE 350.0f
#define CONE_CACHE_SIZE 650.0f

#define POOP_FULL_SPEED 1.0f

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
   currentSensor.reset();
}

void Intake::init()
{
    state.intakeConeSpeed = DEFAULT_INTAKE_CONE_SPD;
    state.intakeCubeSpeed = DEFAULT_INTAKE_CUBE_SPD;
    state.outtakeConeSpeed = DEFAULT_OUTTAKE_CONE_SPD;
    state.outtakeCubeSpeed = DEFAULT_OUTTAKE_CUBE_SPD;
    state.coneHoldSpeed = CONE_HOLD_SPD;
    state.cubeHoldSpeed = CUBE_HOLD_SPD;
    
    state.cubeSpikeCurrent = CUBE_SPIKED_CURRENT;
    state.coneSpikeCurrent = CONE_SPIKED_CURRENT;

    state.coneCacheSize = CONE_CACHE_SIZE;
    state.cubeCacheSize = CUBE_CACHE_SIZE;

    state.intakeOp = false;

    state.isCubeStall = false;

    prevState = state;

    currentSensor.setSpikeSetpoint(state.coneSpikeCurrent);
    currentSensor.setGetter([this]() { return intakeMotor.getCurrent(); });
    currentSensor.setSpikeCallback([this]() { state.intakeState = SPIKED;});
    currentSensor.setCacheSize(CONE_CACHE_SIZE);

    table->PutNumber("outtake Cone speed", state.outtakeConeSpeed);
    table->PutNumber("outtake Cube speed", state.outtakeCubeSpeed);
    table->PutNumber("Cone Hold Speed", state.coneHoldSpeed);
    table->PutNumber("Cube Hold Speed", state.cubeHoldSpeed);
    table->PutNumber("Cone Spiked Current", state.coneSpikeCurrent);
    table->PutNumber("Cube Spiked Current", state.cubeSpikeCurrent);
    table->PutNumber("Cone Cache Size", state.coneCacheSize);
    table->PutNumber("Cube Cache Size", state.cubeCacheSize);

    intakeMotor.getMotor()->ConfigSupplyCurrentLimit(SupplyCurrentLimitConfiguration(true, 60, 90, .75));
    intakeMotor.setVoltageCompensation(12.0);


    resetState();
}

void Intake::assessInputs()
{
    // Driver/Operator scoring independently
    

    // SCORE
    if (driverGamepad->rightTriggerActive()) {
        state.intakeState = OUTTAKE;  
    // No game element in intake, driver/operator requesting intake
    } else if (state.intakeState != SPIKED) {

        // Driver or operator ground pickup
        if (driverGamepad->GetLeftBumper() || driverGamepad->GetRightBumper() || 
        (operatorGamepad->DPadLeft() && driverGamepad->leftTriggerActive()) || 
        driverGamepad->DPadLeft() || 
        driverGamepad->DPadRight() ||
        (driverGamepad->leftTriggerActive() && operatorGamepad->rightTriggerActive())) {
            state.intakeState = INTAKE;
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

    state.intakeOp = operatorGamepad->leftTriggerActive();
}

void Intake::analyzeDashboard()
{
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

    if (prevState.pieceState != state.pieceState) {
        if (state.pieceState == Piece::CONE) {
            currentSensor.setCacheSize(state.coneCacheSize);
            currentSensor.setSpikeSetpoint(state.coneSpikeCurrent);
        } else if (state.pieceState == Piece::CUBE) {
            currentSensor.setCacheSize(state.cubeCacheSize);
            currentSensor.setSpikeSetpoint(state.cubeSpikeCurrent);
        }
        
    }
}
 
void Intake::assignOutputs()
{ 
    if (state.intakeOp){
         if (state.pieceState == Piece::CUBE){
            intakeMotor.setPower(state.intakeCubeSpeed);
        } else{
            intakeMotor.setPower(state.intakeConeSpeed);
        }
    } else if (state.intakeState == DISABLED) {
        intakeMotor.setPower(0);
    } else if (state.intakeState == SPIKED) {
        if (state.isCubeStall){
            intakeMotor.setPower(state.cubeHoldSpeed);
        } else{
            intakeMotor.setPower(state.coneHoldSpeed);
        }
    } else if (state.intakeState == OUTTAKE){
        state.isCubeStall = false;

        if (state.pieceState == Piece::CUBE) {
            if (state.elevarmPoopFull) {
                intakeMotor.setPower(POOP_FULL_SPEED);
            } else if (state.elevarmGround){
                intakeMotor.setPower(GROUND_OUTTAKE_GROUND_CUBE_SPD);
            } else {
                intakeMotor.setPower(state.outtakeCubeSpeed);
            }
        } else {
            intakeMotor.setPower(state.outtakeConeSpeed);
        }  
    } else if (state.intakeState == INTAKE){
        if (state.pieceState == Piece::CUBE) {
            state.isCubeStall = true;
            intakeMotor.setPower(state.intakeCubeSpeed);
        } else {
            intakeMotor.setPower(state.intakeConeSpeed);
            state.isCubeStall = false;
        }
    }
    prevState = state;
}

frc2::FunctionalCommand * Intake::getAutoCommand(std::string intakeState, std::string pieceState){
    IntakeStates inState = stringToIntakeState(intakeState);
    Piece inPiece = stringToPieceState(pieceState);
    return new frc2::FunctionalCommand(
        // OnInit
        [&, inState, inPiece]() {
            state.intakeState = inState;
            
            if (inState != IntakeStates::DISABLED)
                state.pieceState = inPiece;
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

Piece Intake::getPrevPiece(){
    return prevState.pieceState;
}
Piece Intake::getFuturePiece(){
    return state.pieceState;
}

void Intake::setConeHoldSpeed(double isAuto) {
    state.intakeConeSpeed = (isAuto ? AUTO_CONE_HOLD_SPD : CONE_HOLD_SPD); 
}