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

#define PREVIOUS_HEIGHT_DEADBAND 0.1f
#define PREVIOUS_ROTATION_DEADBAND 2.0f

#define X_BUMPER_WIDTH 0.0762f
#define X_HALF_WIDTH 0.254f
#define X_CARRIAGE_OFFSET 0.1778f
#define X_ARM_LENGTH 1.01854f

#define Z_CARRIAGE_JOINT_OFFSET 0.0724154f
#define Z_CARRIAGE_FLOOR_OFFSET 0.2286f
#define Z_INTAKE_OFFSET 0.138f

// #define X_CHASSIS_FRONT_BOUND 0.0f
#define X_CHASSIS_FRONT_BOUND 0.2f
// #define X_CHASSIS_BACK_BOUND  -0.6604f
#define X_CHASSIS_BACK_BOUND  -0.8604f
#define Z_FORK 0.465f
#define Z_GROUND 0.45f

Elevarm::Elevarm(frc::TimedRobot *_robot) : ValorSubsystem(_robot, "Elevarm"),                        
                            carriageMotors(CANIDs::CARRIAGE_MAIN, ValorNeutralMode::Break, false),
                            armRotateMotor(CANIDs::ARM_ROTATE, ValorNeutralMode::Break, false),
                            manualMaxArmSpeed(1.0),
                            manualMaxCarriageSpeed(1.0)
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

    stowPos = frc::Pose3d(-0.54_m, 0_m, 0.55_m, frc::Rotation3d());
    posMap[ElevarmPieceState::ELEVARM_CONE][ElevarmDirectionState::ELEVARM_FRONT][ElevarmPositionState::ELEVARM_GROUND] = frc::Pose3d( 0.408_m,  0.0_m,  0.226_m, frc::Rotation3d() );
    posMap[ElevarmPieceState::ELEVARM_CONE][ElevarmDirectionState::ELEVARM_BACK][ElevarmPositionState::ELEVARM_GROUND] = frc::Pose3d( -0.408_m,  0.0_m,  0.226_m, frc::Rotation3d() );
    posMap[ElevarmPieceState::ELEVARM_CONE][ElevarmDirectionState::ELEVARM_FRONT][ElevarmPositionState::ELEVARM_PLAYER] = frc::Pose3d( 0.5_m,  0.0_m,  0.946150_m, frc::Rotation3d() );
    posMap[ElevarmPieceState::ELEVARM_CONE][ElevarmDirectionState::ELEVARM_BACK][ElevarmPositionState::ELEVARM_PLAYER] = frc::Pose3d( -0.5_m,  0.0_m,  0.946150_m, frc::Rotation3d() );
    posMap[ElevarmPieceState::ELEVARM_CONE][ElevarmDirectionState::ELEVARM_FRONT][ElevarmPositionState::ELEVARM_MID] = frc::Pose3d( 0.576898_m,  0.0_m,  0.862775_m, frc::Rotation3d() );
    posMap[ElevarmPieceState::ELEVARM_CONE][ElevarmDirectionState::ELEVARM_BACK][ElevarmPositionState::ELEVARM_MID] = frc::Pose3d( -0.576898_m,  0.0_m,  0.862775_m, frc::Rotation3d() );
    posMap[ElevarmPieceState::ELEVARM_CONE][ElevarmDirectionState::ELEVARM_FRONT][ElevarmPositionState::ELEVARM_HIGH] = frc::Pose3d( 0.866_m,  0.0_m,  1.5_m, frc::Rotation3d() );
    posMap[ElevarmPieceState::ELEVARM_CONE][ElevarmDirectionState::ELEVARM_BACK][ElevarmPositionState::ELEVARM_HIGH] = frc::Pose3d( -0.866_m,  0.0_m,  1.5_m, frc::Rotation3d() );
    posMap[ElevarmPieceState::ELEVARM_CUBE][ElevarmDirectionState::ELEVARM_FRONT][ElevarmPositionState::ELEVARM_GROUND] = frc::Pose3d( 0.408_m,  0.0_m,  0.226_m, frc::Rotation3d() );
    posMap[ElevarmPieceState::ELEVARM_CUBE][ElevarmDirectionState::ELEVARM_BACK][ElevarmPositionState::ELEVARM_GROUND] = frc::Pose3d( -0.408_m,  0.0_m,  0.226_m, frc::Rotation3d() );
    posMap[ElevarmPieceState::ELEVARM_CUBE][ElevarmDirectionState::ELEVARM_FRONT][ElevarmPositionState::ELEVARM_PLAYER] = frc::Pose3d( 0.5_m,  0.0_m,  0.946150_m, frc::Rotation3d() );
    posMap[ElevarmPieceState::ELEVARM_CUBE][ElevarmDirectionState::ELEVARM_BACK][ElevarmPositionState::ELEVARM_PLAYER] = frc::Pose3d( -0.5_m,  0.0_m,  0.946150_m, frc::Rotation3d() );
    posMap[ElevarmPieceState::ELEVARM_CUBE][ElevarmDirectionState::ELEVARM_FRONT][ElevarmPositionState::ELEVARM_MID] = frc::Pose3d( 0.576898_m,  0.0_m,  0.80_m, frc::Rotation3d() );
    posMap[ElevarmPieceState::ELEVARM_CUBE][ElevarmDirectionState::ELEVARM_BACK][ElevarmPositionState::ELEVARM_MID] = frc::Pose3d( -0.576898_m,  0.0_m,  0.80_m, frc::Rotation3d() );
    posMap[ElevarmPieceState::ELEVARM_CUBE][ElevarmDirectionState::ELEVARM_FRONT][ElevarmPositionState::ELEVARM_HIGH] = frc::Pose3d( 0.866_m,  0.0_m,  1.5_m, frc::Rotation3d() );
    posMap[ElevarmPieceState::ELEVARM_CUBE][ElevarmDirectionState::ELEVARM_BACK][ElevarmPositionState::ELEVARM_HIGH] = frc::Pose3d( -0.866_m,  0.0_m,  1.5_m, frc::Rotation3d() );

    table->PutNumber("Carriage Max Manual Speed", manualMaxCarriageSpeed);
    table->PutNumber("Arm Rotate Max Manual Speed", manualMaxArmSpeed);

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
    } else if (operatorGamepad->GetAButton() || operatorGamepad->DPadDown()){
        futureState.positionState = ElevarmPositionState::ELEVARM_GROUND;
    } else if(operatorGamepad->GetXButton() || operatorGamepad->DPadLeft()){
        futureState.positionState = ElevarmPositionState::ELEVARM_PLAYER;
    } else if(operatorGamepad->GetYButton() || operatorGamepad->DPadUp()){
        futureState.positionState = ElevarmPositionState::ELEVARM_HIGH;
    } else if(operatorGamepad->GetBButton() || operatorGamepad->DPadRight()){
        futureState.positionState = ElevarmPositionState::ELEVARM_MID;
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

    if (operatorGamepad->GetLeftBumper()){
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
    table->PutNumber("Carriage Position", carriageMotors.getPosition());
    table->PutNumber("Arm Position", armRotateMotor.getPosition());

    table->PutNumber("EA position state", futureState.positionState);
    table->PutNumber("EA piece state", futureState.pieceState);
    table->PutNumber("EA direction state", futureState.directionState);
    table->PutNumber("target height", futureState.targetPose.h);
    table->PutNumber("target rotation", futureState.targetPose.theta);
    futureState.resultKinematics = forwardKinematics(Positions(carriageMotors.getPosition(), armRotateMotor.getPosition()));
    table->PutNumber("forward kin. x", futureState.resultKinematics.X().to<double>());
    table->PutNumber("forward kin. z", futureState.resultKinematics.Z().to<double>());
}

void Elevarm::assignOutputs()
{    
    if (futureState.positionState == ElevarmPositionState::ELEVARM_STOW) {
        futureState.targetPose = reverseKinematics(stowPos, ElevarmSolutions::ELEVARM_LEGS);
    } else {
        if (futureState.directionState != previousState.directionState) {
        futureState.targetPose = reverseKinematics(stowPos, ElevarmSolutions::ELEVARM_LEGS);
        } else {
            if (futureState.positionState == ElevarmPositionState::ELEVARM_PLAYER || futureState.positionState == ElevarmPositionState::ELEVARM_MID) {
                futureState.targetPose = reverseKinematics(posMap[futureState.pieceState][futureState.directionState][futureState.positionState], ElevarmSolutions::ELEVARM_ARMS);
            } else 
                futureState.targetPose = reverseKinematics(posMap[futureState.pieceState][futureState.directionState][futureState.positionState], ElevarmSolutions::ELEVARM_LEGS);
        }
    }

    

    if (futureState.deadManEnabled) {
        if (futureState.positionState == ElevarmPositionState::ELEVARM_MANUAL) {
            auto manualOutputs = detectionBoxManual(futureState.manualCarriage, futureState.manualArm);
            if (manualOutputs.h == 0.0) 
                carriageMotors.setPower(0.0125);
            else 
                carriageMotors.setPower(manualOutputs.h);
            armRotateMotor.setPower(manualOutputs.theta);
            previousState.positionState = ElevarmPositionState::ELEVARM_MANUAL;
        } else {
            if (previousState.positionState == ElevarmPositionState::ELEVARM_STOW && futureState.positionState == ElevarmPositionState::ELEVARM_GROUND) {
                if (std::fabs(armRotateMotor.getPosition()) > futureState.targetPose.theta - 2.0){
                    carriageMotors.setPosition(futureState.targetPose.h);
                } else {
                    carriageMotors.setPower(0.0125);
                }
                armRotateMotor.setPosition(futureState.targetPose.theta);
            } else if (previousState.positionState == ElevarmPositionState::ELEVARM_GROUND && futureState.positionState == ElevarmPositionState::ELEVARM_STOW){
                if (std::fabs(carriageMotors.getPosition() - futureState.targetPose.h) < PREVIOUS_HEIGHT_DEADBAND) {
                    armRotateMotor.setPosition(futureState.targetPose.theta);
                } else {
                    armRotateMotor.setPower(0.0);
                }
                carriageMotors.setPosition(futureState.targetPose.h);
            } else if (previousState.positionState == ElevarmPositionState::ELEVARM_STOW && ( futureState.positionState == ElevarmPositionState::ELEVARM_MID || futureState.positionState == ElevarmPositionState::ELEVARM_PLAYER)) {
                if (std::fabs(armRotateMotor.getPosition()) > 90.0){
                    carriageMotors.setPosition(futureState.targetPose.h);
                } else {
                    carriageMotors.setPower(0.0125);
                }
                armRotateMotor.setPosition(futureState.targetPose.theta);
            } else {            
                carriageMotors.setPosition(futureState.targetPose.h);
                armRotateMotor.setPosition(futureState.targetPose.theta);
            }
           
            previousState = ((std::abs(carriageMotors.getPosition() - futureState.targetPose.h)  <= PREVIOUS_HEIGHT_DEADBAND) && 
            (std::abs(armRotateMotor.getPosition() - futureState.targetPose.theta) <= PREVIOUS_ROTATION_DEADBAND))  ? futureState : previousState;
        }
    } else {
        carriageMotors.setPower(0.0125);
        armRotateMotor.setPower(0);
        previousState = ((std::abs(carriageMotors.getPosition() - futureState.targetPose.h)  <= PREVIOUS_HEIGHT_DEADBAND) && 
        (std::abs(armRotateMotor.getPosition() - futureState.targetPose.theta) <= PREVIOUS_ROTATION_DEADBAND))  ? futureState : previousState;
    }
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

Elevarm::Positions Elevarm::detectionBoxAuto() {
    double vertical = X_CARRIAGE_OFFSET - X_BUMPER_WIDTH - X_HALF_WIDTH;
    double currentX = futureState.resultKinematics.X().to<double>();
    double currentZ = futureState.resultKinematics.Z().to<double>();
    double carriagePos = futureState.targetPose.h;
    double armPos = futureState.targetPose.h;
    // Arm inside front chassis box or in ground
    if (currentX >= vertical &&  // checks front includes middle line
      ((X_CHASSIS_FRONT_BOUND > currentX && Z_FORK > currentZ) ||  //inside chassis
       (Z_GROUND > currentZ))) {  //inside ground
        // Allow only up
        carriagePos = carriageMotors.getPosition() > futureState.targetPose.h ? carriageMotors.getPosition() : futureState.targetPose.h;
        // Allow only away from chassis (positive)
        armPos = armRotateMotor.getPosition() > futureState.targetPose.theta ? armRotateMotor.getPosition() : futureState.targetPose.theta;
    // Arm inside back chassis box or in ground
    } else if (vertical > currentX && // checks back
             ((currentX > X_CHASSIS_BACK_BOUND && Z_FORK > currentZ) || // inside chassis
              (Z_GROUND > currentZ))) {  //inside ground
        // Allow only up
        carriagePos = carriageMotors.getPosition() > futureState.targetPose.h ? carriageMotors.getPosition() : futureState.targetPose.h;
        // Allow only away from chassis (negative)
        armPos = futureState.targetPose.theta > armRotateMotor.getPosition() ? armRotateMotor.getPosition() : futureState.targetPose.theta;
    }
    return Positions(carriagePos, armPos);
}


Elevarm::Positions Elevarm::reverseKinematics(frc::Pose3d pose, ElevarmSolutions solution) 
{
    double phi = 0.0;
    double theta = 0.0;
    double height = 0.0;

    table->PutNumber("target Xt", pose.X().to<double>());
    table->PutNumber("target Zt", pose.Z().to<double>());

    // Arms solution
    if (solution == ElevarmSolutions::ELEVARM_ARMS) {
        phi = std::acos((pose.X().to<double>() + X_BUMPER_WIDTH + X_HALF_WIDTH - X_CARRIAGE_OFFSET ) / X_ARM_LENGTH);
        theta = phi + M_PI / 2.0;
        height = pose.Z().to<double>() - (X_ARM_LENGTH * std::sin(phi)) + Z_INTAKE_OFFSET;

    // Legs Solution
    } else {
        theta = std::asin((pose.X().to<double>() + X_BUMPER_WIDTH + X_HALF_WIDTH - X_CARRIAGE_OFFSET ) / X_ARM_LENGTH);
        height = pose.Z().to<double>() + (X_ARM_LENGTH * std::cos(theta)) - Z_INTAKE_OFFSET;
    }
    height -= (Z_CARRIAGE_FLOOR_OFFSET + Z_CARRIAGE_JOINT_OFFSET);

    return Positions(height,theta * 180.0 / M_PI);
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
            // z = 0.2286f + 0.0724154f + 0.138f  - ( 1.01854f * 0.866025 )

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
