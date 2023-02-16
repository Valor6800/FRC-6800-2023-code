/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
#include "subsystems/Elevarm.h"
#include <iostream>


#define ROTATE_GEAR_RATIO 143.73f
#define ROTATE_OUTPUT_DIAMETER 0.1334516f
#define CARRIAGE_GEAR_RATIO 4.0f
#define CARRAIAGE_OUTPUT_DIAMETER 0.0364f
#define CARRIAGE_UPPER_LIMIT 1.05f 
#define CARRIAGE_LOWER_LIMIT 0.07f
#define ROTATE_FORWARD_LIMIT 180.0f
#define ROTATE_REVERSE_LIMIT -180.0f

#define CARRIAGE_K_F 0.000156f  
#define CARRIAGE_K_P 0.00005f
#define CARRIAGE_K_I 0.0f
#define CARRIAGE_K_D 0.0f
#define CARRIAGE_K_ERROR 0.005f
#define CARRIAGE_K_VEL 4.0f
#define CARRIAGE_K_ACC_MUL 20.0f

#define ROTATE_K_F 0.000156f
#define ROTATE_K_P 0.00005f
#define ROTATE_K_I 0.0f
#define ROTATE_K_D 0.0f
#define ROTATE_K_ERROR 0.1f
#define ROTATE_K_VEL 180.0f
#define ROTATE_K_ACC_MUL 10.0f

#define PREVIOUS_HEIGHT_DEADBAND 0.02f
#define PREVIOUS_ROTATION_DEADBAND 2.0f

#define X_BUMPER_WIDTH 0.0762f
#define X_HALF_WIDTH 0.254f
#define X_CARRIAGE_OFFSET 0.1778f
#define X_ARM_LENGTH 1.01854f
#define X_INTAKE_FORWARD_OFFSET 0.0f  // 0.2450338f
#define X_INTAKE_REVERSE_OFFSET 0.0f  // 0.126285117f

#define Z_CARRIAGE_JOINT_OFFSET 0.0724154f
#define Z_CARRIAGE_FLOOR_OFFSET 0.2286f
#define Z_INTAKE_OFFSET 0.03f

// #define X_CHASSIS_FRONT_BOUND 0.0f
#define X_CHASSIS_FRONT_BOUND 0.2f
// #define X_CHASSIS_BACK_BOUND  -0.6604f
#define X_CHASSIS_BACK_BOUND  -0.8604f
#define Z_FORK 0.465f
#define Z_GROUND 0.45f


#define P_MIN_CARRIAGE 0.0125f
#define P_MIN_ARM 0.0125f



Elevarm::Elevarm(frc::TimedRobot *_robot, Intake *_intake) : ValorSubsystem(_robot, "Elevarm"),                        
                            intake(_intake),
                            carriageMotors(CANIDs::CARRIAGE_MAIN, ValorNeutralMode::Brake, false),
                            armRotateMotor(CANIDs::ARM_ROTATE, ValorNeutralMode::Brake, false),
                            manualMaxArmSpeed(1.0),
                            manualMaxCarriageSpeed(1.0),
                            carriageStallPower(P_MIN_CARRIAGE)
{
    frc2::CommandScheduler::GetInstance().RegisterSubsystem(this);
    init();
}

Elevarm::~Elevarm()
{
}

void Elevarm::resetState()
{
    futureState.pieceState = ElevarmPieceState::ELEVARM_CONE;
    futureState.directionState = ElevarmDirectionState::ELEVARM_FRONT;
    futureState.positionState = ElevarmPositionState::ELEVARM_STOW;
    futureState.manualCarriage = 0;
    futureState.manualArm = 0;
    futureState.deadManEnabled = false;
    futureState.pitModeEnabled = false;
    previousState = futureState;

}

void Elevarm::init()
{ 
    ValorPIDF carriagePID;
    carriagePID.velocity = CARRIAGE_K_VEL;
    carriagePID.acceleration = carriagePID.velocity * CARRIAGE_K_ACC_MUL;
    carriagePID.F = CARRIAGE_K_F;
    carriagePID.P = CARRIAGE_K_P;
    carriagePID.I = CARRIAGE_K_I;
    carriagePID.D = CARRIAGE_K_D;
    carriagePID.error = CARRIAGE_K_ERROR;

    
    ValorPIDF rotatePID;
    rotatePID.velocity = ROTATE_K_VEL;
    rotatePID.acceleration = rotatePID.velocity * ROTATE_K_ACC_MUL;
    rotatePID.F = ROTATE_K_F;
    rotatePID.P = ROTATE_K_P;
    rotatePID.I = ROTATE_K_I;
    rotatePID.D = ROTATE_K_D;
    rotatePID.error = ROTATE_K_ERROR; 
    
    
    carriageMotors.setConversion(1.0 / CARRIAGE_GEAR_RATIO * M_PI * CARRAIAGE_OUTPUT_DIAMETER);
    carriageMotors.setForwardLimit(CARRIAGE_UPPER_LIMIT);
    carriageMotors.setReverseLimit(CARRIAGE_LOWER_LIMIT);
    carriageMotors.setPIDF(carriagePID, 0);

    carriageMotors.setupFollower(CANIDs::CARRIAGE_FOLLOW, false);

    armRotateMotor.setConversion(1.0 / ROTATE_GEAR_RATIO * 360.0);
    armRotateMotor.setReverseLimit(ROTATE_REVERSE_LIMIT);
    armRotateMotor.setPIDF(rotatePID, 0);

    stowPos = frc::Pose3d(-0.508_m, 0_m, 0.431_m, frc::Rotation3d());
    posMap[ElevarmPieceState::ELEVARM_CONE][ElevarmDirectionState::ELEVARM_FRONT][ElevarmPositionState::ELEVARM_GROUND] = frc::Pose3d( 0.408_m,  0.0_m,  0_m, frc::Rotation3d() );
    posMap[ElevarmPieceState::ELEVARM_CONE][ElevarmDirectionState::ELEVARM_BACK][ElevarmPositionState::ELEVARM_GROUND] = frc::Pose3d( -1.0_m,  0.0_m,  0.4_m, frc::Rotation3d() );
    posMap[ElevarmPieceState::ELEVARM_CONE][ElevarmDirectionState::ELEVARM_FRONT][ElevarmPositionState::ELEVARM_PLAYER] = frc::Pose3d( 0.75_m,  0.0_m,  0.946150_m, frc::Rotation3d() );
    posMap[ElevarmPieceState::ELEVARM_CONE][ElevarmDirectionState::ELEVARM_BACK][ElevarmPositionState::ELEVARM_PLAYER] = frc::Pose3d( -0.76_m,  0.0_m,  1.25_m, frc::Rotation3d() );
    posMap[ElevarmPieceState::ELEVARM_CONE][ElevarmDirectionState::ELEVARM_FRONT][ElevarmPositionState::ELEVARM_MID] = frc::Pose3d( 0.576898_m,  0.0_m,  1.15_m, frc::Rotation3d() );
    posMap[ElevarmPieceState::ELEVARM_CONE][ElevarmDirectionState::ELEVARM_BACK][ElevarmPositionState::ELEVARM_MID] = frc::Pose3d( -0.76_m,  0.0_m,  1.25_m, frc::Rotation3d() );
    posMap[ElevarmPieceState::ELEVARM_CONE][ElevarmDirectionState::ELEVARM_FRONT][ElevarmPositionState::ELEVARM_HIGH] = frc::Pose3d( 0.866_m,  0.0_m,  1.1668_m, frc::Rotation3d() );
    posMap[ElevarmPieceState::ELEVARM_CONE][ElevarmDirectionState::ELEVARM_BACK][ElevarmPositionState::ELEVARM_HIGH] = frc::Pose3d( -1.1709_m,  0.0_m,  1.1668_m, frc::Rotation3d() );
    posMap[ElevarmPieceState::ELEVARM_CUBE][ElevarmDirectionState::ELEVARM_FRONT][ElevarmPositionState::ELEVARM_GROUND] = frc::Pose3d( 0.408_m,  0.0_m,  0_m, frc::Rotation3d() );
    posMap[ElevarmPieceState::ELEVARM_CUBE][ElevarmDirectionState::ELEVARM_BACK][ElevarmPositionState::ELEVARM_GROUND] = frc::Pose3d( -1.0_m,  0.0_m,  0.4_m, frc::Rotation3d() );
    posMap[ElevarmPieceState::ELEVARM_CUBE][ElevarmDirectionState::ELEVARM_FRONT][ElevarmPositionState::ELEVARM_PLAYER] = frc::Pose3d( 0.75_m,  0.0_m,  0.946150_m, frc::Rotation3d() );
    posMap[ElevarmPieceState::ELEVARM_CUBE][ElevarmDirectionState::ELEVARM_BACK][ElevarmPositionState::ELEVARM_PLAYER] = frc::Pose3d( -0.76_m,  0.0_m,  1.25_m, frc::Rotation3d() );
    posMap[ElevarmPieceState::ELEVARM_CUBE][ElevarmDirectionState::ELEVARM_FRONT][ElevarmPositionState::ELEVARM_MID] = frc::Pose3d( 0.576898_m,  0.0_m,  1.15_m, frc::Rotation3d() );
    posMap[ElevarmPieceState::ELEVARM_CUBE][ElevarmDirectionState::ELEVARM_BACK][ElevarmPositionState::ELEVARM_MID] = frc::Pose3d( -0.76_m,  0.0_m,  1.25_m, frc::Rotation3d() );
    posMap[ElevarmPieceState::ELEVARM_CUBE][ElevarmDirectionState::ELEVARM_FRONT][ElevarmPositionState::ELEVARM_HIGH] = frc::Pose3d( 0.866_m,  0.0_m,  1.1668_m, frc::Rotation3d() );
    posMap[ElevarmPieceState::ELEVARM_CUBE][ElevarmDirectionState::ELEVARM_BACK][ElevarmPositionState::ELEVARM_HIGH] = frc::Pose3d( -1.1709_m,  0.0_m,  1.1668_m, frc::Rotation3d() );
    posMap[ElevarmPieceState::ELEVARM_CONE][ElevarmDirectionState::ELEVARM_FRONT][ElevarmPositionState::ELEVARM_SNAKE] = frc::Pose3d( 0.06_m,  0.0_m,  1.44_m, frc::Rotation3d() );
    posMap[ElevarmPieceState::ELEVARM_CONE][ElevarmDirectionState::ELEVARM_BACK][ElevarmPositionState::ELEVARM_SNAKE] = frc::Pose3d( -0.6_m,  0.0_m,  1.34_m, frc::Rotation3d() );
    posMap[ElevarmPieceState::ELEVARM_CUBE][ElevarmDirectionState::ELEVARM_FRONT][ElevarmPositionState::ELEVARM_SNAKE] = frc::Pose3d( 0.06_m,  0.0_m,  1.44_m, frc::Rotation3d() );
    posMap[ElevarmPieceState::ELEVARM_CUBE][ElevarmDirectionState::ELEVARM_BACK][ElevarmPositionState::ELEVARM_SNAKE] = frc::Pose3d( -0.6_m,  0.0_m,  1.34_m, frc::Rotation3d() );
    

    table->PutNumber("Carriage Max Manual Speed", manualMaxCarriageSpeed);
    table->PutNumber("Arm Rotate Max Manual Speed", manualMaxArmSpeed);
    table->PutBoolean("Pit Mode", futureState.pitModeEnabled);
    table->PutNumber("Carraige Stall", carriageStallPower);

    resetState();
    armRotateMotor.setEncoderPosition(180.0);
}

void Elevarm::assessInputs()
{   
    futureState.manualCarriage = 0;
    futureState.manualArm = 0;
    
    if (operatorGamepad->leftStickYActive() || operatorGamepad->rightStickYActive()) {
        futureState.manualCarriage = operatorGamepad->leftStickY() * manualMaxCarriageSpeed;
        futureState.manualArm = operatorGamepad->rightStickY() * manualMaxArmSpeed;
        futureState.positionState = ElevarmPositionState::ELEVARM_MANUAL;
    } else if (operatorGamepad->GetRightBumper()) {
        futureState.positionState = ElevarmPositionState::ELEVARM_STOW;
    } else if (operatorGamepad->GetAButton() || operatorGamepad->DPadDown() || driverGamepad->GetLeftBumper() || driverGamepad->GetRightBumper()){
        if (intake->state.intakeState == Intake::IntakeStates::SPIKED) futureState.positionState = ElevarmPositionState::ELEVARM_STOW;
        else futureState.positionState = ElevarmPositionState::ELEVARM_GROUND;
    } else if(operatorGamepad->GetXButton() || operatorGamepad->DPadLeft()){
        if (intake->state.intakeState == Intake::IntakeStates::SPIKED) futureState.positionState = ElevarmPositionState::ELEVARM_SNAKE;
        else futureState.positionState = ElevarmPositionState::ELEVARM_PLAYER;
    } else if (operatorGamepad->GetYButton() || operatorGamepad->DPadUp()){
        if (driverGamepad->rightTriggerActive()) futureState.positionState = ElevarmPositionState::ELEVARM_HIGH;
        else futureState.positionState = ElevarmPositionState::ELEVARM_SNAKE;
    } else if(operatorGamepad->GetBButton() || operatorGamepad->DPadRight()){
        if (driverGamepad->rightTriggerActive()) futureState.positionState = ElevarmPositionState::ELEVARM_MID;
        else futureState.positionState = ElevarmPositionState::ELEVARM_SNAKE;
    } else {
        if (previousState.positionState != ElevarmPositionState::ELEVARM_MANUAL) {
            futureState.positionState = ElevarmPositionState::ELEVARM_STOW;
        } 
    }

    if (operatorGamepad->DPadUp() || operatorGamepad->DPadDown() 
    || operatorGamepad->DPadLeft() || operatorGamepad->DPadRight()){
        futureState.pieceState = ElevarmPieceState::ELEVARM_CUBE;
    } else {
        futureState.pieceState = ElevarmPieceState::ELEVARM_CONE;
    }

    if (driverGamepad->GetLeftBumper()) {
        futureState.directionState = ElevarmDirectionState::ELEVARM_FRONT;
    } else if (driverGamepad->GetRightBumper() || operatorGamepad->GetLeftBumper()) {
        futureState.directionState = ElevarmDirectionState::ELEVARM_BACK;
    } else {
        futureState.directionState = ElevarmDirectionState::ELEVARM_FRONT;
    }

    futureState.deadManEnabled = operatorGamepad->GetRightTriggerAxis();
}

void Elevarm::analyzeDashboard()
{
    manualMaxCarriageSpeed = table->GetNumber("Carriage Max Manual Speed", 1.0);
    manualMaxArmSpeed = table->GetNumber("Arm Rotate Max Manual Speed", 1.0);
    futureState.pitModeEnabled = table->GetBoolean("Pit Mode", false);
    carriageStallPower = table->GetNumber("Carriage Stall Power", P_MIN_CARRIAGE);
}

void Elevarm::assignOutputs()
{    
    if (futureState.directionState != previousState.directionState) {
        futureState.positionState = ElevarmPositionState::ELEVARM_STOW;
    }

    if (futureState.positionState == ElevarmPositionState::ELEVARM_STOW) {
        futureState.targetPose = reverseKinematics(stowPos, ElevarmSolutions::ELEVARM_LEGS, ElevarmDirectionState::ELEVARM_FRONT);
    } else {
            if (futureState.positionState == ElevarmPositionState::ELEVARM_PLAYER || futureState.positionState == ElevarmPositionState::ELEVARM_MID || futureState.positionState == ElevarmPositionState::ELEVARM_SNAKE) {
                futureState.targetPose = reverseKinematics(posMap[futureState.pieceState][futureState.directionState][futureState.positionState], ElevarmSolutions::ELEVARM_ARMS , futureState.directionState);
            } else 
                futureState.targetPose = reverseKinematics(posMap[futureState.pieceState][futureState.directionState][futureState.positionState], ElevarmSolutions::ELEVARM_LEGS, futureState.directionState);
    }

    table->PutBoolean("going from stow to not stow", false);
    

    if ((futureState.deadManEnabled && futureState.pitModeEnabled) || !futureState.pitModeEnabled) {
        if (futureState.positionState == ElevarmPositionState::ELEVARM_MANUAL) {
            auto manualOutputs = detectionBoxManual(futureState.manualCarriage, futureState.manualArm);
            if (manualOutputs.h == 0.0) 
                carriageMotors.setPower(carriageStallPower);
            else 
                carriageMotors.setPower(manualOutputs.h);
            armRotateMotor.setPower(manualOutputs.theta);
            previousState.positionState = ElevarmPositionState::ELEVARM_MANUAL;
        } else {
            if (previousState.positionState == ElevarmPositionState::ELEVARM_STOW && (futureState.positionState != ElevarmPositionState::ELEVARM_STOW )) {
                table->PutBoolean("going from stow to not stow", true);
                if (std::fabs(armRotateMotor.getPosition()) > minAngle() && minFloorAngle()){
                    carriageMotors.setPosition(futureState.targetPose.h);
                } else {
                    carriageMotors.setPower(carriageStallPower);

                }
                armRotateMotor.setPosition(futureState.targetPose.theta);
            } else if (previousState.positionState == ElevarmPositionState::ELEVARM_GROUND && futureState.positionState == ElevarmPositionState::ELEVARM_STOW){
                if (std::fabs(carriageMotors.getPosition() - futureState.targetPose.h) < PREVIOUS_HEIGHT_DEADBAND) {
                    armRotateMotor.setPosition(futureState.targetPose.theta);
                } else {
                    armRotateMotor.setPower(0.0);
                }
                carriageMotors.setPosition(futureState.targetPose.h);
            } else {            
                carriageMotors.setPosition(futureState.targetPose.h);
                armRotateMotor.setPosition(futureState.targetPose.theta);
            }
            
            if (std::fabs(carriageMotors.getPosition() - futureState.targetPose.h) <= PREVIOUS_HEIGHT_DEADBAND) {
                carriageMotors.setPower(carriageStallPower);
            } 

            previousState = ((std::fabs(carriageMotors.getPosition() - futureState.targetPose.h)  <= PREVIOUS_HEIGHT_DEADBAND) && 
            (std::fabs(armRotateMotor.getPosition() - futureState.targetPose.theta) <= PREVIOUS_ROTATION_DEADBAND))  ? futureState : previousState;
            
        }
            
    } else {
        carriageMotors.setPower(carriageStallPower);
        armRotateMotor.setPower(0);
        previousState = ((std::fabs(carriageMotors.getPosition() - futureState.targetPose.h)  <= PREVIOUS_HEIGHT_DEADBAND) && 
        (std::fabs(armRotateMotor.getPosition() - futureState.targetPose.theta) <= PREVIOUS_ROTATION_DEADBAND))  ? futureState : previousState;
    }
}
double Elevarm::minAngle()
{
    double Zt = carriageMotors.getPosition() + Z_CARRIAGE_FLOOR_OFFSET + Z_CARRIAGE_JOINT_OFFSET ;
    double Xt = -X_BUMPER_WIDTH - X_HALF_WIDTH + X_CARRIAGE_OFFSET;
    double Zs = Z_FORK;
    double Xs = armRotateMotor.getPosition() > 0 ? X_CHASSIS_FRONT_BOUND : X_CHASSIS_BACK_BOUND;
    return std::fabs(atan2(Xs - Xt, Zt - Zs) * 180.0 / M_PI);
}

bool Elevarm::minFloorAngle()
{
    return (futureState.resultKinematics.Z().to<double>() - Z_INTAKE_OFFSET) > 0.5;
}

Elevarm::Positions Elevarm::detectionBoxManual(double carriage, double arm) {
    double vertical = X_CARRIAGE_OFFSET - X_BUMPER_WIDTH - X_HALF_WIDTH;
    double currentX = futureState.resultKinematics.X().to<double>();
    double currentZ = futureState.resultKinematics.Z().to<double>();
    // Arm inside front chassis box or in ground
    if (currentX > vertical && ((X_CHASSIS_FRONT_BOUND > currentX && Z_FORK > currentZ) || (Z_GROUND > currentZ))) {
        // Allow only up
        carriage = carriage > 0 ? carriage : 0;
        // Allow only away from chassis (positive)
        arm = arm > 0 ? arm : 0;
    // Arm inside back chassis box or in ground
    } else if (vertical > currentX && ((currentX > X_CHASSIS_BACK_BOUND && Z_FORK > currentZ) || (Z_GROUND > currentZ))) {
        // Allow only up
        carriage = carriage > 0 ? carriage : 0;
        // Allow only away from chassis (negative)
        arm = 0 > arm ? arm : 0;
    }
    return Positions(carriage, arm);
}


Elevarm::Positions Elevarm::reverseKinematics(frc::Pose3d pose, ElevarmSolutions solution, ElevarmDirectionState dir) 
{
    double phi = 0.0;
    double theta = 0.0;
    double height = 0.0;
    double intakeOff = (dir == ElevarmDirectionState::ELEVARM_FRONT) ? X_INTAKE_FORWARD_OFFSET: X_INTAKE_REVERSE_OFFSET;
    double direction = (dir == ElevarmDirectionState::ELEVARM_FRONT) ? 1.0 : -1.0;

    // Arms solution
    if (solution == ElevarmSolutions::ELEVARM_ARMS) {
        phi = std::acos(std::fabs((pose.X().to<double>() + X_BUMPER_WIDTH + X_HALF_WIDTH - X_CARRIAGE_OFFSET - intakeOff) / X_ARM_LENGTH));
        theta = phi + (M_PI / 2.0);
        height = pose.Z().to<double>() - (X_ARM_LENGTH * std::sin(phi));
        theta *= direction;
    // Legs Solution
    } else {
        theta = std::asin((pose.X().to<double>() + X_BUMPER_WIDTH + X_HALF_WIDTH - X_CARRIAGE_OFFSET - intakeOff) / X_ARM_LENGTH);
        height = pose.Z().to<double>() + (X_ARM_LENGTH * std::cos(theta));
    }
    height -= (Z_CARRIAGE_FLOOR_OFFSET + Z_CARRIAGE_JOINT_OFFSET + Z_INTAKE_OFFSET);


    return Positions(height,theta * 180.0 / M_PI);
}

frc::Pose3d Elevarm::forwardKinematics(Elevarm::Positions positions) 
{
    double x, z = 0;
    double theta = positions.theta * M_PI / 180.0;
    // Forward
    if (theta > 0) {
        // Arms
        if (theta > (M_PI / 2.0)) {
            double phi = theta - (M_PI / 2.0);
            z = Z_CARRIAGE_FLOOR_OFFSET + Z_CARRIAGE_JOINT_OFFSET + Z_INTAKE_OFFSET + positions.h + X_ARM_LENGTH * sin(phi);
            x = X_ARM_LENGTH * cos(phi) + X_CARRIAGE_OFFSET - X_BUMPER_WIDTH - X_HALF_WIDTH;
        // Legs
        } else {
            z = Z_CARRIAGE_FLOOR_OFFSET + Z_CARRIAGE_JOINT_OFFSET + Z_INTAKE_OFFSET + positions.h - X_ARM_LENGTH * cos(theta);
            x = X_ARM_LENGTH * sin(theta) + X_CARRIAGE_OFFSET - X_BUMPER_WIDTH - X_HALF_WIDTH;
        }
    // Reverse
    } else {
        // Arms
        if (theta < -(M_PI / 2.0)) {
            double phi = std::fabs(theta + (M_PI / 2.0));
            z = Z_CARRIAGE_FLOOR_OFFSET + Z_CARRIAGE_JOINT_OFFSET + Z_INTAKE_OFFSET + positions.h + X_ARM_LENGTH * sin(phi);
            x = -X_ARM_LENGTH * cos(phi) + X_CARRIAGE_OFFSET - X_BUMPER_WIDTH - X_HALF_WIDTH;
        // Legs
        } else {
            z = Z_CARRIAGE_FLOOR_OFFSET + Z_CARRIAGE_JOINT_OFFSET + Z_INTAKE_OFFSET + positions.h - X_ARM_LENGTH * cos(std::fabs(theta));
            x = -X_ARM_LENGTH * sin(std::fabs(theta)) + X_CARRIAGE_OFFSET - X_BUMPER_WIDTH - X_HALF_WIDTH;
        }
    }
    return frc::Pose3d((units::length::meter_t)x, 0_m, (units::length::meter_t)z, frc::Rotation3d());
}

void Elevarm::InitSendable(wpi::SendableBuilder& builder)
{
    builder.SetSmartDashboardType("Subsystem");

    builder.AddDoubleProperty(
        "Previous State Piece",
        [this] { return previousState.pieceState; },
        nullptr
    );

    builder.AddDoubleProperty(
        "Previous State Position",
        [this] { return previousState.positionState; },
        nullptr
    );

    builder.AddDoubleProperty(
        "Previous State Direction",
        [this] { return previousState.directionState; },
        nullptr
    );

    builder.AddDoubleProperty(
        "Previous State Target Height",
        [this]{ return previousState.targetPose.h; },
        nullptr
    );

        builder.AddDoubleProperty(
        "Previous State Target Theta",
        [this]{ return previousState.targetPose.theta; },
        nullptr
    );

    builder.AddDoubleProperty(
        "Future State Piece",
        [this] { return futureState.pieceState; },
        nullptr
    );

    builder.AddDoubleProperty(
        "Future State Position",
        [this] { return futureState.positionState; },
        nullptr
    );

    builder.AddDoubleProperty(
        "Future State Direction",
        [this] { return futureState.directionState; },
        nullptr
    );

    builder.AddDoubleProperty(
        "Future State Target Height",
        [this]{ return futureState.targetPose.h; },
        nullptr
    );

        builder.AddDoubleProperty(
        "Future State Target Theta",
        [this]{ return futureState.targetPose.theta; },
        nullptr
    );
        builder.AddDoubleProperty(
        "Min Floor Angle",
        [this]{ return minFloorAngle(); },
        nullptr
    );
        builder.AddDoubleProperty(
        "Min Angle",
        [this]{ return minAngle(); },
        nullptr
    );
        builder.AddDoubleProperty(
        "Forward Kinematics Z",
        [this]{ return futureState.resultKinematics.Z().to<double>(); },
        nullptr
    );
        builder.AddDoubleProperty(
        "Forward Kinematics X",
        [this]{ return futureState.resultKinematics.X().to<double>(); },
        nullptr
    );
    builder.AddBooleanProperty(
        "going from stow",
        [this]{ return table->GetBoolean("going from stow to not stow", false); },
        nullptr
    );
        
}

frc2::FunctionalCommand * Elevarm::getAutoCommand(std::string pieceState, std::string directionState, std::string positionState){
    return new frc2::FunctionalCommand(
        // OnInit
        [&]() {
            
            }, 
        //onExecute
        [&, pieceState, directionState, positionState](){
            futureState.pieceState = stringToPieceState(pieceState);
            futureState.directionState = stringToDirectionState(directionState);
            futureState.positionState = stringToPositionState(positionState);
        }, 
        [&](bool){
            previousState = futureState;
        }, // onEnd
        [&](){ //isFinished
            return ((std::fabs(carriageMotors.getPosition() - futureState.targetPose.h)  <= PREVIOUS_HEIGHT_DEADBAND) && 
            (std::fabs(armRotateMotor.getPosition() - futureState.targetPose.theta) <= PREVIOUS_ROTATION_DEADBAND));
        },
        {}
    );
}

