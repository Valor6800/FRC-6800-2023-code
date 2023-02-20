/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
#include "subsystems/Elevarm.h"
#include <iostream>


#define ROTATE_GEAR_RATIO 74.25f
#define CARRIAGE_GEAR_RATIO 4.0f
#define CARRAIAGE_OUTPUT_DIAMETER 0.0364f
#define CARRIAGE_UPPER_LIMIT 0.89f 
#define CARRIAGE_LOWER_LIMIT 0.07f
#define ROTATE_FORWARD_LIMIT 180.0f
#define ROTATE_REVERSE_LIMIT -180.0f

#define CARRIAGE_K_F 0.000156f  
#define CARRIAGE_K_P 0.0f
#define CARRIAGE_K_I 0.0f
#define CARRIAGE_K_D 0.0f
#define CARRIAGE_K_ERROR 0.005f
#define CARRIAGE_K_VEL 2.0f
#define CARRIAGE_K_ACC_MUL 1.0f

#define ROTATE_K_F 0.75f
#define ROTATE_K_P 0.03f
#define ROTATE_K_I 0.0f
#define ROTATE_K_D 0.0f
#define ROTATE_K_ERROR 1.0f
#define ROTATE_K_VEL 120.0f
#define ROTATE_K_ACC_MUL 0.66f
#define ROTATE_K_AFF 0.105f
#define ROTATE_K_AFF_CUBE 0.1275f
#define ROTATE_K_AFF_POS 90.0f

#define PREVIOUS_HEIGHT_DEADBAND 0.02f
#define PREVIOUS_ROTATION_DEADBAND 2.0f

#define X_BUMPER_WIDTH 0.0984f
#define X_HALF_WIDTH 0.2921f
#define X_CARRIAGE_OFFSET 0.121f
#define X_ARM_LENGTH 0.84455f
#define X_INTAKE_FORWARD_OFFSET 0.254f
#define X_INTAKE_REVERSE_OFFSET 0.127f

#define Z_CARRIAGE_JOINT_OFFSET 0.1651f
#define Z_CARRIAGE_FLOOR_OFFSET 0.230f
#define Z_INTAKE_OFFSET 0.1143f

// #define X_CHASSIS_FRONT_BOUND 0.0f
#define X_CHASSIS_FRONT_BOUND 0.2113f
// #define X_CHASSIS_BACK_BOUND  -0.6604f
#define X_CHASSIS_BACK_BOUND  -1.0f
#define Z_FORK 0.465f
#define Z_GROUND 0.1f

#define P_MIN_CARRIAGE 0.0f
#define P_MIN_ARM 0.0f

#define MAN_MAX_CARRIAGE 0.3f
#define MAN_MAX_ROTATE 0.4f

Elevarm::Elevarm(frc::TimedRobot *_robot, Intake *_intake) : ValorSubsystem(_robot, "Elevarm"),                        
                            intake(_intake),
                            carriageMotors(CANIDs::CARRIAGE_MAIN, ValorNeutralMode::Brake, false),
                            armRotateMotor(CANIDs::ARM_ROTATE, ValorNeutralMode::Brake, false, "baseCAN"),
                            manualMaxArmSpeed(MAN_MAX_ROTATE),
                            manualMaxCarriageSpeed(MAN_MAX_CARRIAGE),
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
    rotatePID.acceleration = ROTATE_K_ACC_MUL;
    rotatePID.F = ROTATE_K_F;
    rotatePID.P = ROTATE_K_P;
    rotatePID.I = ROTATE_K_I;
    rotatePID.D = ROTATE_K_D;
    rotatePID.error = ROTATE_K_ERROR; 
    rotatePID.aFF = ROTATE_K_AFF; 
    rotatePID.aFFTarget = ROTATE_K_AFF_POS; 
    
    carriageMotors.setConversion(1.0 / CARRIAGE_GEAR_RATIO * M_PI * CARRAIAGE_OUTPUT_DIAMETER);
    carriageMotors.setForwardLimit(CARRIAGE_UPPER_LIMIT);
    carriageMotors.setReverseLimit(CARRIAGE_LOWER_LIMIT);
    carriageMotors.setPIDF(carriagePID, 0);

    carriageMotors.setupFollower(CANIDs::CARRIAGE_FOLLOW, false);

    armRotateMotor.setConversion(1.0 / ROTATE_GEAR_RATIO * 360.0);
    armRotateMotor.setReverseLimit(ROTATE_REVERSE_LIMIT);
    armRotateMotor.setPIDF(rotatePID, 0);

    stowPos = frc::Pose3d(-0.463_m, 0_m, 0.5722_m, frc::Rotation3d());
    posMap[ElevarmPieceState::ELEVARM_CONE][ElevarmDirectionState::ELEVARM_FRONT][ElevarmPositionState::ELEVARM_GROUND] = frc::Pose3d( 0.31_m,  0.0_m,  0.307_m, frc::Rotation3d() );
    posMap[ElevarmPieceState::ELEVARM_CONE][ElevarmDirectionState::ELEVARM_BACK][ElevarmPositionState::ELEVARM_GROUND] = frc::Pose3d( -1.12_m,  0.0_m,  0.535_m, frc::Rotation3d() );
    posMap[ElevarmPieceState::ELEVARM_CONE][ElevarmDirectionState::ELEVARM_FRONT][ElevarmPositionState::ELEVARM_PLAYER] = frc::Pose3d( 0.19_m,  0.0_m,  1.32_m, frc::Rotation3d() );
    posMap[ElevarmPieceState::ELEVARM_CONE][ElevarmDirectionState::ELEVARM_BACK][ElevarmPositionState::ELEVARM_PLAYER] = frc::Pose3d( -0.88_m,  0.0_m,  1.3_m, frc::Rotation3d() );
    posMap[ElevarmPieceState::ELEVARM_CONE][ElevarmDirectionState::ELEVARM_FRONT][ElevarmPositionState::ELEVARM_MID] = frc::Pose3d( 0.275_m,  0.0_m,  1.22_m, frc::Rotation3d() );
    posMap[ElevarmPieceState::ELEVARM_CONE][ElevarmDirectionState::ELEVARM_BACK][ElevarmPositionState::ELEVARM_MID] = frc::Pose3d( -1.07_m,  0.0_m,  1.2_m, frc::Rotation3d() );
    posMap[ElevarmPieceState::ELEVARM_CONE][ElevarmDirectionState::ELEVARM_FRONT][ElevarmPositionState::ELEVARM_HIGH] = frc::Pose3d( 0.51_m,  0.0_m,  1.55_m, frc::Rotation3d() );
    posMap[ElevarmPieceState::ELEVARM_CONE][ElevarmDirectionState::ELEVARM_BACK][ElevarmPositionState::ELEVARM_HIGH] = frc::Pose3d( -1.055_m,  0.0_m,  1.4_m, frc::Rotation3d() );
    posMap[ElevarmPieceState::ELEVARM_CUBE][ElevarmDirectionState::ELEVARM_FRONT][ElevarmPositionState::ELEVARM_GROUND] = frc::Pose3d( 0.31_m,  0.0_m,  0.307_m, frc::Rotation3d() );
    posMap[ElevarmPieceState::ELEVARM_CUBE][ElevarmDirectionState::ELEVARM_BACK][ElevarmPositionState::ELEVARM_GROUND] = frc::Pose3d( -1.12_m,  0.0_m,  0.535_m, frc::Rotation3d() );
    posMap[ElevarmPieceState::ELEVARM_CUBE][ElevarmDirectionState::ELEVARM_FRONT][ElevarmPositionState::ELEVARM_PLAYER] = frc::Pose3d( 0.19_m,  0.0_m,  1.32_m, frc::Rotation3d() );
    posMap[ElevarmPieceState::ELEVARM_CUBE][ElevarmDirectionState::ELEVARM_BACK][ElevarmPositionState::ELEVARM_PLAYER] = frc::Pose3d( -0.88_m,  0.0_m,  1.3_m, frc::Rotation3d() );
    posMap[ElevarmPieceState::ELEVARM_CUBE][ElevarmDirectionState::ELEVARM_FRONT][ElevarmPositionState::ELEVARM_MID] = frc::Pose3d( 0.255_m,  0.0_m,  1.175_m, frc::Rotation3d() );
    posMap[ElevarmPieceState::ELEVARM_CUBE][ElevarmDirectionState::ELEVARM_BACK][ElevarmPositionState::ELEVARM_MID] = frc::Pose3d( -0.89_m,  0.0_m,  1.08_m, frc::Rotation3d() );
    posMap[ElevarmPieceState::ELEVARM_CUBE][ElevarmDirectionState::ELEVARM_FRONT][ElevarmPositionState::ELEVARM_HIGH] = frc::Pose3d( 0.3_m,  0.0_m,  1.315_m, frc::Rotation3d() );
    posMap[ElevarmPieceState::ELEVARM_CUBE][ElevarmDirectionState::ELEVARM_BACK][ElevarmPositionState::ELEVARM_HIGH] = frc::Pose3d( -1.055_m,  0.0_m,  1.4_m, frc::Rotation3d() );
    posMap[ElevarmPieceState::ELEVARM_CONE][ElevarmDirectionState::ELEVARM_FRONT][ElevarmPositionState::ELEVARM_SNAKE] = frc::Pose3d( -0.07_m,  0.0_m,  1.36_m, frc::Rotation3d() );
    posMap[ElevarmPieceState::ELEVARM_CONE][ElevarmDirectionState::ELEVARM_BACK][ElevarmPositionState::ELEVARM_SNAKE] = frc::Pose3d( -0.56_m,  0.0_m,  1.36_m, frc::Rotation3d() );
    posMap[ElevarmPieceState::ELEVARM_CUBE][ElevarmDirectionState::ELEVARM_FRONT][ElevarmPositionState::ELEVARM_SNAKE] = frc::Pose3d( -0.07_m,  0.0_m,  1.36_m, frc::Rotation3d() );
    posMap[ElevarmPieceState::ELEVARM_CUBE][ElevarmDirectionState::ELEVARM_BACK][ElevarmPositionState::ELEVARM_SNAKE] = frc::Pose3d( -0.56_m,  0.0_m,  1.36_m, frc::Rotation3d() );
    

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
        table->PutNumber("L stick Y", futureState.manualCarriage);
        table->PutNumber("R stick X", futureState.manualArm);
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
    manualMaxCarriageSpeed = table->GetNumber("Carriage Max Manual Speed", MAN_MAX_CARRIAGE);
    manualMaxArmSpeed = table->GetNumber("Arm Rotate Max Manual Speed", MAN_MAX_ROTATE);
    futureState.pitModeEnabled = table->GetBoolean("Pit Mode", false);
    carriageStallPower = table->GetNumber("Carriage Stall Power", P_MIN_CARRIAGE);
    // armStallPower = table->GetNumber("Arm Stall Power", P_MIN_ARM);
    futureState.resultKinematics = forwardKinematics(Elevarm::Positions(carriageMotors.getPosition(), armRotateMotor.getPosition()));
    table->PutNumber("Min angle front", minAngle(true));
    table->PutNumber("min angle back", minAngle(false));
}

void Elevarm::assignOutputs()
{    
    bool inTransition = futureState.directionState != previousState.directionState;

    if (futureState.positionState == ElevarmPositionState::ELEVARM_STOW || inTransition) {
        futureState.targetPose = reverseKinematics(stowPos, ElevarmSolutions::ELEVARM_LEGS, ElevarmDirectionState::ELEVARM_FRONT);
    } else {
            if (futureState.positionState == ElevarmPositionState::ELEVARM_PLAYER || futureState.positionState == ElevarmPositionState::ELEVARM_MID || futureState.positionState == ElevarmPositionState::ELEVARM_SNAKE || futureState.positionState == ElevarmPositionState::ELEVARM_HIGH) {
                futureState.targetPose = reverseKinematics(posMap[futureState.pieceState][futureState.directionState][futureState.positionState], ElevarmSolutions::ELEVARM_ARMS , futureState.directionState);
            } else 
                futureState.targetPose = reverseKinematics(posMap[futureState.pieceState][futureState.directionState][futureState.positionState], ElevarmSolutions::ELEVARM_LEGS, futureState.directionState);
    }

    table->PutBoolean("going from stow to not stow", false);
    bool atCarriage = std::fabs(carriageMotors.getPosition() - futureState.targetPose.h)  <= PREVIOUS_HEIGHT_DEADBAND;
    bool atArm = std::fabs(armRotateMotor.getPosition() - futureState.targetPose.theta) <= PREVIOUS_ROTATION_DEADBAND;

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
            double frontAngle = minAngle(true);
            double backAngle = minAngle(false);

            // Target in triangle
            if (futureState.targetPose.theta >= backAngle && futureState.targetPose.theta <= frontAngle){
                if (atCarriage || std::fabs(armRotateMotor.getPosition()) > 45){
                    armRotateMotor.setPosition(futureState.targetPose.theta);
                }
                else {
                    armRotateMotor.setPower(0.0);
                }
                carriageMotors.setPosition(futureState.targetPose.h);
            // Target oustside triangle
            } else {
                if (armRotateMotor.getPosition() > frontAngle || armRotateMotor.getPosition() < backAngle){
                    carriageMotors.setPosition(futureState.targetPose.h);
                } else {
                    carriageMotors.setPower(carriageStallPower);
                }
                armRotateMotor.setPosition(futureState.targetPose.theta);
            }
            
            if (atCarriage) {
                carriageMotors.setPower(carriageStallPower);
            }

            if (atCarriage && atArm) {
                previousState = futureState;
                if (inTransition)
                    previousState.positionState = ELEVARM_STOW;
            }
        }
            
    } else {
        carriageMotors.setPower(carriageStallPower);
        armRotateMotor.setPower(0);
    }
}
double Elevarm::minAngle(bool isFront)
{
    double Zt = carriageMotors.getPosition() + Z_CARRIAGE_FLOOR_OFFSET + Z_CARRIAGE_JOINT_OFFSET ;
    double Xt = -X_BUMPER_WIDTH - X_HALF_WIDTH + X_CARRIAGE_OFFSET;
    double Zs = Z_FORK;
    if (isFront)
        return atan2(X_CHASSIS_FRONT_BOUND - Xt, Zt - Zs) * 180 / M_PI;
    return atan2(X_CHASSIS_BACK_BOUND - Xt, Zt - Zs) * 180 / M_PI;
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
    table->PutNumber("current X", currentX);
    table->PutNumber("current z", currentZ);
    table->PutNumber("vertial", vertical);
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
            x = -(X_ARM_LENGTH * sin(std::fabs(theta))) + X_CARRIAGE_OFFSET - X_BUMPER_WIDTH - X_HALF_WIDTH;
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
        [this]{ return minAngle(true); },
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
    builder.AddDoubleProperty(
        "Min angle front",
        [this]{ return minAngle(true); },
        nullptr
    );
    builder.AddDoubleProperty(
        "Min angle back",
        [this]{ return minAngle(false); },
        nullptr
    );
}

frc2::FunctionalCommand * Elevarm::getAutoCommand(std::string pieceState, std::string directionState, std::string positionState, bool parallel){
    Elevarm::ElevarmPieceState eaPieceState = stringToPieceState(pieceState);
    Elevarm::ElevarmDirectionState eaDirectionState = stringToDirectionState(directionState);
    Elevarm::ElevarmPositionState eaPositionState = stringToPositionState(positionState);
    return new frc2::FunctionalCommand(
        // OnInit
        [&]() {
            
            }, 
        //onExecute
        [&, eaPieceState, eaDirectionState, eaPositionState](){
            futureState.pieceState = eaPieceState;
            futureState.directionState = eaDirectionState;
            futureState.positionState = eaPositionState;
            table->PutNumber("piece", futureState.pieceState);
            table->PutNumber("dir", futureState.directionState);
            table->PutNumber("pos", futureState.positionState);
        }, 
        [&](bool){
            previousState = futureState;
        }, // onEnd
        [&, eaPieceState, eaDirectionState, eaPositionState, parallel](){ //isFinished
            return parallel || (previousState.directionState == eaDirectionState && previousState.positionState == eaPositionState);
        },
        {}
    );
}

