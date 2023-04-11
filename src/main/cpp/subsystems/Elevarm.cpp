/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
#include "subsystems/Elevarm.h"
#include <iostream>

#define IS_COMP

#ifdef IS_COMP
#define ROTATE_GEAR_RATIO 101.25f
#define WRIST_GEAR_RATIO 24.3f

#define ARM_CANCODER_OFFSET  64.0f
#define WRIST_CANCODER_OFFSET 57.83f
#else
#define ROTATE_GEAR_RATIO 83.53f
#define WRIST_GEAR_RATIO 24.3f

#define ARM_CANCODER_OFFSET  300.925f
#define WRIST_CANCODER_OFFSET 122.81f
#endif

#define CARRIAGE_GEAR_RATIO 4.0f
#define ARM_CANCODER_GEAR_RATIO 1.0f
#define WRIST_CANCODER_GEAR_RATIO 1.0f
#define CARRAIAGE_OUTPUT_DIAMETER 0.0364f

#define CARRIAGE_UPPER_LIMIT 0.86f 
#define CARRIAGE_LOWER_LIMIT 0.0f
#define ROTATE_FORWARD_LIMIT 180.0f
#define ROTATE_REVERSE_LIMIT -250.0f
#define WRIST_FORWARD_LIMIT 325.0f
#define WRIST_REVERSE_LIMIT -325.0f

#define INITIAL_HEIGHT_OFFSET 0.0f //cm
#define STOW_HEIGHT_OFFSET 0.0f //cm

#define CARRIAGE_K_F 0.000156f  
#define CARRIAGE_K_P 1.5e-4f
#define CARRIAGE_K_I 0.0f
#define CARRIAGE_K_D 0.0f
#define CARRIAGE_K_ERROR 0.005f
#define CARRIAGE_K_VEL 4.0f
#define CARRIAGE_K_ACC_MUL 0.15f

#define ROTATE_K_F 0.75f
#define ROTATE_K_P 0.04f
#define ROTATE_K_I 0.0f
#define ROTATE_K_D 0.0f
#define ROTATE_K_ERROR 0.5f
#define ROTATE_K_VEL 140.0f
#define ROTATE_K_ACC_MUL 0.9f
#define ROTATE_K_AFF 0.14f
#define ROTATE_K_AFF_CUBE 0.11f
#define ROTATE_K_AFF_POS 90.0f

#define AUTO_ROTATE_K_F 0.75f
#define AUTO_ROTATE_K_P 0.045f
#define AUTO_ROTATE_K_I 0.0f
#define AUTO_ROTATE_K_D 0.0f
#define AUTO_ROTATE_K_ERROR 0.5f
#define AUTO_ROTATE_K_VEL 120.0f
#define AUTO_ROTATE_K_ACC_MUL 0.66f //2.0f //1.5 = 2deg shift
#define AUTO_ROTATE_K_AFF 0.115f
#define AUTO_ROTATE_K_AFF_CUBE 0.11f
#define AUTO_ROTATE_K_AFF_POS 90.0f

#define ROTATE_S_CURVE_STRENGTH 3

#define WRIST_K_F 0.75f
#define WRIST_K_P 0.18f
#define WRIST_K_I 0.0f
#define WRIST_K_D 0.0f
#define WRIST_K_ERROR 0.25f
#define WRIST_K_VEL 750.0f
#define WRIST_K_ACC_MUL 0.425f
#define WRIST_S_CURVE_STRENGTH 3

#define PREVIOUS_WRIST_DEADBAND 1.01f
#define PREVIOUS_HEIGHT_DEADBAND 0.03f
#define PREVIOUS_ROTATION_DEADBAND 3.5f

#define X_BUMPER_WIDTH 0.0984f
#define X_HALF_WIDTH 0.2921f
#define X_CARRIAGE_OFFSET 0.121f
#define X_ARM_LENGTH 0.857f

#define Z_CARRIAGE_JOINT_OFFSET 0.1651f
#define Z_CARRIAGE_FLOOR_OFFSET 0.230f

// #define X_CHASSIS_FRONT_BOUND 0.0f
// #define X_CHASSIS_FRONT_BOUND 0.2113f
#define X_CHASSIS_FRONT_BOUND 0.1016f
// #define X_CHASSIS_BACK_BOUND  -0.6604f
#define X_CHASSIS_BACK_BOUND  -1.0f
#define Z_FORK 0.42f
#define Z_GROUND 0.1f

#define P_MIN_CARRIAGE 0.01f
#define P_MIN_ARM 0.0f

#define MAN_MAX_CARRIAGE 0.3f
#define MAN_MAX_ROTATE 0.4f

Elevarm::Elevarm(frc::TimedRobot *_robot, Intake *_intake) : ValorSubsystem(_robot, "Elevarm"),                        
                            intake(_intake),
                            carriageMotors(CANIDs::CARRIAGE_MAIN, ValorNeutralMode::Brake, false),
                            armRotateMotor(CANIDs::ARM_ROTATE, ValorNeutralMode::Brake, false, "baseCAN"),
                            armCANcoder(CANIDs::ARM_CANCODER, "baseCAN"),
                            wristMotor(CANIDs::WRIST, ValorNeutralMode::Brake, false, "baseCAN"),
                            wristCANcoder(CANIDs::WRIST_CANCODER, "baseCAN"),
                            candle(_robot, 286, CANIDs::CANDLE, "baseCAN"),
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
    setFuturePiece(Piece::CONE);
    futureState.directionState = Direction::FRONT;
    futureState.positionState = Position::STOW;
    futureState.manualCarriage = 0;
    futureState.manualArm = 0;
    futureState.deadManEnabled = false;
    futureState.pitModeEnabled = false;
    zeroArm = false;
    coastMode = false;
    previousState = futureState;
    setPrevPiece(intake->getFuturePiece());

}

void Elevarm::init()
{ 
    carriagePID.velocity = CARRIAGE_K_VEL;
    carriagePID.acceleration = CARRIAGE_K_ACC_MUL;
    carriagePID.F = CARRIAGE_K_F;
    carriagePID.P = CARRIAGE_K_P;
    carriagePID.I = CARRIAGE_K_I;
    carriagePID.D = CARRIAGE_K_D;
    carriagePID.error = CARRIAGE_K_ERROR;

    rotatePID.velocity = ROTATE_K_VEL;
    rotatePID.acceleration = ROTATE_K_ACC_MUL;
    rotatePID.F = ROTATE_K_F;
    rotatePID.P = ROTATE_K_P;
    rotatePID.I = ROTATE_K_I;
    rotatePID.D = ROTATE_K_D;
    rotatePID.error = ROTATE_K_ERROR; 
    rotatePID.aFF = ROTATE_K_AFF; 
    rotatePID.aFFTarget = ROTATE_K_AFF_POS;
    rotatePID.sCurveStrength = ROTATE_S_CURVE_STRENGTH;

    autoRotatePID.velocity = AUTO_ROTATE_K_VEL;
    autoRotatePID.acceleration = AUTO_ROTATE_K_ACC_MUL;
    autoRotatePID.F = AUTO_ROTATE_K_F;
    autoRotatePID.P = AUTO_ROTATE_K_P;
    autoRotatePID.I = AUTO_ROTATE_K_I;
    autoRotatePID.D = AUTO_ROTATE_K_D;
    autoRotatePID.error = AUTO_ROTATE_K_ERROR; 
    autoRotatePID.aFF = AUTO_ROTATE_K_AFF; 
    autoRotatePID.aFFTarget = AUTO_ROTATE_K_AFF_POS; 
    autoRotatePID.sCurveStrength = ROTATE_S_CURVE_STRENGTH;
    
    wristPID.velocity = WRIST_K_VEL;
    wristPID.acceleration = WRIST_K_ACC_MUL;
    wristPID.F = WRIST_K_F;
    wristPID.P = WRIST_K_P;
    wristPID.I = WRIST_K_I;
    wristPID.D = WRIST_K_D;
    wristPID.error = WRIST_K_ERROR; 
    wristPID.sCurveStrength = WRIST_S_CURVE_STRENGTH;
        
    carriageMotors.setConversion(1.0 / CARRIAGE_GEAR_RATIO * M_PI * CARRAIAGE_OUTPUT_DIAMETER);
    carriageMotors.setForwardLimit(CARRIAGE_UPPER_LIMIT);
    carriageMotors.setReverseLimit(CARRIAGE_LOWER_LIMIT);
    carriageMotors.setPIDF(carriagePID, 0);

    carriageMotors.setupFollower(CANIDs::CARRIAGE_FOLLOW, false);

    armRotateMotor.setVoltageCompensation(12.0);
    armRotateMotor.setConversion(1.0 / ROTATE_GEAR_RATIO * 360.0);
    armRotateMotor.setForwardLimit(ROTATE_FORWARD_LIMIT);
    armRotateMotor.setReverseLimit(ROTATE_REVERSE_LIMIT);
    armRotateMotor.setPIDF(rotatePID, 0);

    armCANcoder.ConfigAbsoluteSensorRange(AbsoluteSensorRange::Unsigned_0_to_360);
    armCANcoder.SetPositionToAbsolute();
    wristCANcoder.ConfigAbsoluteSensorRange(AbsoluteSensorRange::Unsigned_0_to_360);
    wristCANcoder.SetPositionToAbsolute();
    wristCANcoder.ConfigSensorDirection(true);

    wristMotor.setVoltageCompensation(12.0);
    wristMotor.setConversion((1.0 / WRIST_GEAR_RATIO) * 360.0);
    wristMotor.setForwardLimit(WRIST_FORWARD_LIMIT);
    wristMotor.setReverseLimit(WRIST_REVERSE_LIMIT);
    wristMotor.setPIDF(wristPID, 0);

    // STOW POSITION
    stowPos = frc::Pose2d(-0.4185_m, 0.354_m, -15.0_deg);
    
    // FRONT CONE
    posMap[Piece::CONE][Direction::FRONT][Position::GROUND] =frc::Pose2d(0.266_m, 0.483_m, 134.03_deg);
    posMap[Piece::CONE][Direction::FRONT][Position::GROUND_TOPPLE] =frc::Pose2d(0.151_m, 0.190_m, 141.4_deg);
    posMap[Piece::CONE][Direction::FRONT][Position::GROUND_SCORE] =frc::Pose2d(0.112_m, 0.471_m, 165.0_deg);
    posMap[Piece::CONE][Direction::FRONT][Position::PLAYER] =frc::Pose2d(-0.033_m, 1.428_m, 303.3_deg); // new points
    posMap[Piece::CONE][Direction::FRONT][Position::MID] =frc::Pose2d(0.142_m, 1.253_m, -118.16_deg);
    posMap[Piece::CONE][Direction::FRONT][Position::HIGH] =frc::Pose2d(0.576_m, 1.356_m, -187.76_deg);
    posMap[Piece::CONE][Direction::FRONT][Position::SNAKE] =frc::Pose2d(-0.288_m, 1.25_m, 0.0_deg);
    posMap[Piece::CONE][Direction::FRONT][Position::HIGH_AUTO] =frc::Pose2d(0.516_m, 1.53_m, -140.0_deg);

    // FRONT CUBE
    posMap[Piece::CUBE][Direction::FRONT][Position::GROUND] =frc::Pose2d(0.147_m, 0.375_m, 197.47_deg);
    posMap[Piece::CUBE][Direction::FRONT][Position::GROUND_TOPPLE] =frc::Pose2d(0.151_m, 0.190_m, 141.4_deg);
    posMap[Piece::CUBE][Direction::FRONT][Position::GROUND_SCORE] =frc::Pose2d(0.112_m, 0.521_m, 165.0_deg);
    posMap[Piece::CUBE][Direction::FRONT][Position::PLAYER] =frc::Pose2d(-0.037_m, 1.34_m, 318.0_deg);
    posMap[Piece::CUBE][Direction::FRONT][Position::MID] =frc::Pose2d(0.274_m, 1.104_m, -62.93_deg);
    posMap[Piece::CUBE][Direction::FRONT][Position::HIGH] =frc::Pose2d(0.576_m, 1.305_m, -137.65_deg);
    posMap[Piece::CUBE][Direction::FRONT][Position::SNAKE] =frc::Pose2d(-0.288_m, 1.25_m, 0.0_deg);
    posMap[Piece::CUBE][Direction::FRONT][Position::POOPFULL] =frc::Pose2d(0.05_m, 0.436_m, 81.0_deg);

    // BACK CONE   
    // posMap[Piece::CONE][Direction::BACK][Position::GROUND] =frc::Pose2d(-0.99_m, 0.5_m, -208.5_deg);
    posMap[Piece::CONE][Direction::BACK][Position::GROUND] =frc::Pose2d(-0.288_m, 1.25_m, 0.0_deg); //snake pos
    posMap[Piece::CONE][Direction::BACK][Position::GROUND_TOPPLE] =frc::Pose2d(0.151_m, 0.150_m, 141.4_deg);
    posMap[Piece::CONE][Direction::BACK][Position::GROUND_SCORE] =frc::Pose2d(-0.888_m, 0.541_m, -165.0_deg);
    posMap[Piece::CONE][Direction::BACK][Position::PLAYER] =frc::Pose2d(-0.8605_m, 1.5425_m, 40.7_deg);
    posMap[Piece::CONE][Direction::BACK][Position::MID] =frc::Pose2d(-0.904_m, 1.03_m, -180.0_deg);
    posMap[Piece::CONE][Direction::BACK][Position::HIGH] =frc::Pose2d(-0.904_m, 1.03_m, -180.0_deg);
    posMap[Piece::CONE][Direction::BACK][Position::SNAKE] =frc::Pose2d(-0.288_m, 1.25_m, -60.0_deg);
    // BACK CUBE
    posMap[Piece::CUBE][Direction::BACK][Position::GROUND] =frc::Pose2d(-0.914_m, 0.222_m, -140.0_deg);
    posMap[Piece::CUBE][Direction::BACK][Position::GROUND_TOPPLE] =frc::Pose2d(0.151_m, 0.09_m, 141.4_deg);
    posMap[Piece::CUBE][Direction::BACK][Position::GROUND_SCORE] =frc::Pose2d(-0.888_m, 0.541_m, -165.0_deg);
    posMap[Piece::CUBE][Direction::BACK][Position::PLAYER] =frc::Pose2d(-0.823_m, 1.32_m, 78.7_deg);
    posMap[Piece::CUBE][Direction::BACK][Position::MID] =frc::Pose2d(-0.849_m, 1.042_m, -221.0_deg);
    posMap[Piece::CUBE][Direction::BACK][Position::HIGH] =frc::Pose2d(-0.849_m, 1.042_m, -164.0_deg);
    posMap[Piece::CUBE][Direction::BACK][Position::SNAKE] =frc::Pose2d(-0.288_m, 1.25_m, 0.0_deg);
    posMap[Piece::CUBE][Direction::BACK][Position::POOPFULL] =frc::Pose2d(0.05_m, 0.436_m, 81.0_deg);


    table->PutNumber("Carriage Max Manual Speed", MAN_MAX_CARRIAGE);
    table->PutNumber("Arm Rotate Max Manual Speed", MAN_MAX_ROTATE);
    table->PutBoolean("Pit Mode", futureState.pitModeEnabled);
    table->PutNumber("Carriage Stall Power", P_MIN_CARRIAGE);
    table->PutNumber("Carraige Offset", INITIAL_HEIGHT_OFFSET);
    table->PutBoolean("Enable Carraige Offset", false);
    table->PutBoolean("Arm In Range", false);
    table->PutBoolean("Zero Arm", zeroArm);
    table->PutBoolean("Coast Mode", coastMode);

    resetState();
    armRotateMotor.setEncoderPosition((ARM_CANCODER_OFFSET - armCANcoder.GetAbsolutePosition() ) / ARM_CANCODER_GEAR_RATIO - 180.0);
    carriageMotors.setEncoderPosition(0.0);
    wristMotor.setEncoderPosition((wristCANcoder.GetAbsolutePosition() - WRIST_CANCODER_OFFSET) / WRIST_CANCODER_GEAR_RATIO);
    wristCANcoder.SetStatusFramePeriod(CANCoderStatusFrame_SensorData, 50, 1000); // changes the period of the sensor data frame to 50ms
    //armCANcoder.SetStatusFramePeriod(CANCoderStatusFrame_SensorData, 50); // changes the period of the sensor data frame to 50ms

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
        futureState.positionState = Position::MANUAL;
    } else if (driverGamepad->GetLeftBumper() || driverGamepad->GetRightBumper()){
        if (intake->state.intakeState == Intake::IntakeStates::SPIKED) futureState.positionState = Position::STOW;
        else futureState.positionState = Position::GROUND;
    } else if (operatorGamepad->DPadDown()) {
        if (driverGamepad->leftTriggerActive()) futureState.positionState = Position::GROUND_SCORE;
        else futureState.positionState = Position::STOW;
    } else if(operatorGamepad->DPadLeft()){
        if (driverGamepad->leftTriggerActive()) futureState.positionState = Position::PLAYER;
        else if (intake->getFuturePiece() == Piece::CONE) futureState.positionState = Position::SNAKE;
        else futureState.positionState = Position::STOW;
    } else if (operatorGamepad->DPadUp()){
        if (driverGamepad->leftTriggerActive()) futureState.positionState = Position::HIGH;
        else futureState.positionState = Position::SNAKE;
    } else if(operatorGamepad->DPadRight()){
        if (driverGamepad->leftTriggerActive()) futureState.positionState = Position::MID;
        else futureState.positionState = Position::SNAKE;
    } else if (operatorGamepad->GetAButton()){
        if (driverGamepad->leftTriggerActive()) futureState.positionState = Position::POOPFULL;
        else futureState.positionState = Position::STOW;
    } else {
        if (previousState.positionState != Position::MANUAL) {
            if (intake->getFuturePiece() == Piece::CONE) {
                futureState.positionState = Position::SNAKE; 
            } else {
                futureState.positionState = Position::STOW;
            }
        } 
    }
    
    if(operatorGamepad->GetBButton()){
        setFuturePiece(Piece::CONE);
    }

    if(operatorGamepad->GetYButton() || driverGamepad->GetYButton()){
        setFuturePiece(Piece::CUBE);
    }
    
    if (driverGamepad->GetRightBumper() || operatorGamepad->GetLeftBumper()) {
        futureState.directionState = Direction::BACK;
    } else {
        futureState.directionState = Direction::FRONT;
    }

    futureState.deadManEnabled = operatorGamepad->GetRightBumper();
}

void Elevarm::analyzeDashboard()
{
    manualMaxCarriageSpeed = table->GetNumber("Carriage Max Manual Speed", MAN_MAX_CARRIAGE);
    manualMaxArmSpeed = table->GetNumber("Arm Rotate Max Manual Speed", MAN_MAX_ROTATE);
    futureState.pitModeEnabled = table->GetBoolean("Pit Mode", false);
    carriageStallPower = table->GetNumber("Carriage Stall Power", P_MIN_CARRIAGE);
    zeroArm = table->GetBoolean("Zero Arm", false);
    coastMode = table->GetBoolean("Coast Mode", false);

    if (zeroArm) {
        armRotateMotor.setEncoderPosition(180.0);
    }

    if(coastMode && armRotateMotor.getNeutralMode() == ValorNeutralMode::Brake){
        
        armRotateMotor.setNeutralMode(ValorNeutralMode::Coast);
    } else if(armRotateMotor.getNeutralMode() == ValorNeutralMode::Coast && !coastMode){
        armRotateMotor.setNeutralMode(ValorNeutralMode::Brake);
    }

    if (table->GetBoolean("Enable Carraige Offset", false)) {
        futureState.carraigeOffset = table->GetNumber("Carraige Offset", INITIAL_HEIGHT_OFFSET);
    } else {
        futureState.carraigeOffset = INITIAL_HEIGHT_OFFSET;
    }
    // armStallPower = table->GetNumber("Arm Stall Power", P_MIN_ARM);
    futureState.resultKinematics = forwardKinematics(Elevarm::Positions(carriageMotors.getPosition(), armRotateMotor.getPosition(), wristMotor.getPosition()));
    futureState.frontMinAngle = minAngle(true);
    futureState.backMinAngle = minAngle(false);

    futureState.atCarriage = std::fabs(carriageMotors.getPosition() - futureState.targetPose.h)  <= PREVIOUS_HEIGHT_DEADBAND;
    futureState.atArm = std::fabs(armRotateMotor.getPosition() - futureState.targetPose.theta) <= PREVIOUS_ROTATION_DEADBAND;
    futureState.atWrist = std::fabs(wristMotor.getPosition() - futureState.targetPose.wrist) <= PREVIOUS_WRIST_DEADBAND;

    bool armInRange = armCANcoder.GetAbsolutePosition() > (ARM_CANCODER_OFFSET - 5) &&
                      armCANcoder.GetAbsolutePosition() < (ARM_CANCODER_OFFSET + 8);
    table->PutBoolean("Arm In Range", armInRange);

    bool wristInRange = wristCANcoder.GetAbsolutePosition() > (WRIST_CANCODER_OFFSET - 50) &&
                        wristCANcoder.GetAbsolutePosition() < (WRIST_CANCODER_OFFSET + 50);
    table->PutBoolean("Wrist In Range", wristInRange);

    if (intake && intake->state.intakeState == Intake::IntakeStates::SPIKED) {
        candle.setColor(255,0,0);
    } else if (robot->IsDisabled()) {
        if (armInRange) candle.setColor(0,255,0);
        else candle.setColor(255,0,0);
    } else 
    if (robot->IsAutonomous()) {
        if (futureState.atCarriage && futureState.atArm && futureState.atWrist)
            candle.setColor(0,255,0);
        else
            candle.setColor(0,0,255);
    } else {
        if (intake->getFuturePiece() == Piece::CUBE) {
            candle.setColor(156,0,255);
        } else {
            candle.setColor(255,196,0);
        }
    }

    intake->state.elevarmGround = futureState.positionState == Position::GROUND_SCORE;
    intake->state.elevarmPoopFull = futureState.positionState == Position::POOPFULL;
}

void Elevarm::assignOutputs()
{
    if (futureState.positionState == Position::SNAKE) futureState.directionState = Direction::BACK;
    
    Positions stowPose = reverseKinematics(stowPos, ElevarmSolutions::ELEVARM_LEGS, Direction::FRONT);;
    if (futureState.positionState == Position::STOW) {
        futureState.targetPose = stowPose;
    } else {
        if (futureState.positionState == Position::PLAYER || futureState.positionState == Position::MID || futureState.positionState == Position::SNAKE || futureState.positionState == Position::HIGH || futureState.positionState == Position::HIGH_AUTO)
            futureState.targetPose = reverseKinematics(posMap[intake->getFuturePiece()][futureState.directionState][futureState.positionState], ElevarmSolutions::ELEVARM_ARMS , futureState.directionState);
        else 
            futureState.targetPose = reverseKinematics(posMap[intake->getFuturePiece()][futureState.directionState][futureState.positionState], ElevarmSolutions::ELEVARM_LEGS, futureState.directionState);
    }
    table->PutBoolean("going from stow to not stow", false);

    if ((futureState.positionState == Position::HIGH || futureState.positionState == Position::MID) && futureState.directionState == Direction::FRONT) {
        futureState.targetPose.theta -= 360.0;
    }

    if ((futureState.deadManEnabled && futureState.pitModeEnabled) || !futureState.pitModeEnabled) {
        if (futureState.positionState == Position::MANUAL) {
            auto manualOutputs = detectionBoxManual(futureState.manualCarriage, futureState.manualArm);
            if (manualOutputs.h == 0.0) 
                carriageMotors.setPower(carriageStallPower);
             else 
                 carriageMotors.setPower(manualOutputs.h);
            armRotateMotor.setPower(manualOutputs.theta);
            wristMotor.setPosition(stowPos.Rotation().Degrees().to<double>());
            previousState.positionState = Position::MANUAL;
        } else { 
            if ((futureState.directionState == Direction::FRONT && armRotateMotor.getPosition() > 6.0) ||
                (futureState.directionState == Direction::BACK && armRotateMotor.getPosition() < -26.0) ||
                 (armRotateMotor.getPosition() > 100.0) ||
                 (armRotateMotor.getPosition() < -90.0)) {

                armRotateMotor.setPosition(futureState.targetPose.theta);
                
                if ((futureState.positionState == Position::HIGH && armRotateMotor.getPosition() > -200.0) 
                || futureState.atCarriage) {
                    carriageMotors.setPower(carriageStallPower);
                } else 
                    carriageMotors.setPosition(futureState.targetPose.h);
                
            } else {
                if (intake->getFuturePiece() == Piece::CUBE && futureState.positionState == Position::STOW && previousState.positionState == Position::STOW && futureState.atCarriage) {
                    carriageMotors.setPower(carriageStallPower);
                } else
                    carriageMotors.setPosition(stowPose.h);
                if (carriageMotors.getPosition() >= (stowPose.h - 0.03)) {
                    armRotateMotor.setPosition(futureState.targetPose.theta);
                } else {
                    armRotateMotor.setPower(0);
                }
            }
            
            if ((futureState.directionState == Direction::FRONT && armRotateMotor.getPosition() > 6.0) ||
                (futureState.directionState == Direction::BACK && armRotateMotor.getPosition() < -26.0) ||
                (futureState.targetPose.theta < -135.0 && armRotateMotor.getPosition() < -135.0)) {
                wristMotor.setPosition(futureState.targetPose.wrist);
            } else {
                wristMotor.setPosition(stowPos.Rotation().Degrees().to<double>());
            }

            if (futureState.atCarriage && futureState.atArm && futureState.atWrist) {
                previousState = futureState;
                setPrevPiece(intake->getFuturePiece());
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
        return (atan2(X_CHASSIS_FRONT_BOUND - Xt, Zt - Zs) * 180 / M_PI);
    return atan2(X_CHASSIS_BACK_BOUND - Xt, Zt - Zs) * 180 / M_PI;
}

bool Elevarm::minFloorAngle()
{
    return (futureState.resultKinematics.Y().to<double>()) > 0.5;
}

Elevarm::Positions Elevarm::detectionBoxManual(double carriage, double arm) {
    double vertical = X_CARRIAGE_OFFSET - X_BUMPER_WIDTH - X_HALF_WIDTH;
    double currentX = futureState.resultKinematics.X().to<double>();
    double currentZ = futureState.resultKinematics.Y().to<double>();
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
    return Positions(carriage, arm, 0);
}


Elevarm::Positions Elevarm::reverseKinematics(frc::Pose2d pose, ElevarmSolutions solution, Direction dir) 
{
    double phi = 0.0;
    double theta = 0.0;
    double height = 0.0;
    double direction = (dir == Direction::FRONT) ? 1.0 : -1.0;

    // Arms solution
    if (solution == ElevarmSolutions::ELEVARM_ARMS) {
        phi = std::acos(std::fabs((pose.X().to<double>() + X_BUMPER_WIDTH + X_HALF_WIDTH - X_CARRIAGE_OFFSET) / X_ARM_LENGTH));
        theta = phi + (M_PI / 2.0);
        height = pose.Y().to<double>() - (X_ARM_LENGTH * std::sin(phi));
        theta *= direction;
    // Legs Solution
    } else {
        theta = std::asin((pose.X().to<double>() + X_BUMPER_WIDTH + X_HALF_WIDTH - X_CARRIAGE_OFFSET) / X_ARM_LENGTH);
        height = pose.Y().to<double>() + (X_ARM_LENGTH * std::cos(theta));
    }
    height -= (Z_CARRIAGE_FLOOR_OFFSET + Z_CARRIAGE_JOINT_OFFSET);
    height += futureState.carraigeOffset / 100.0; //convert fron cm on dashboard to m in logic


    return Positions(height,theta * 180.0 / M_PI, pose.Rotation().Degrees().to<double>());
}

frc::Pose2d Elevarm::forwardKinematics(Elevarm::Positions positions) 
{
    double x, z = 0;
    double theta = positions.theta * M_PI / 180.0;
    double w = wristMotor.getPosition();

    // Forward
    if (theta > 0) {
        // Arms
        if (theta > (M_PI / 2.0)) {
            double phi = theta - (M_PI / 2.0);
            z = Z_CARRIAGE_FLOOR_OFFSET + Z_CARRIAGE_JOINT_OFFSET + positions.h + X_ARM_LENGTH * sin(phi);
            x = X_ARM_LENGTH * cos(phi) + X_CARRIAGE_OFFSET - X_BUMPER_WIDTH - X_HALF_WIDTH;
        // Legs
        } else {
            z = Z_CARRIAGE_FLOOR_OFFSET + Z_CARRIAGE_JOINT_OFFSET + positions.h - X_ARM_LENGTH * cos(theta);
            x = X_ARM_LENGTH * sin(theta) + X_CARRIAGE_OFFSET - X_BUMPER_WIDTH - X_HALF_WIDTH;
        }
    // Reverse
    } else {
        // Arms
        if (theta < -(M_PI / 2.0)) {
            double phi = std::fabs(theta + (M_PI / 2.0));
            z = Z_CARRIAGE_FLOOR_OFFSET + Z_CARRIAGE_JOINT_OFFSET + positions.h + X_ARM_LENGTH * sin(phi);
            x = -X_ARM_LENGTH * cos(phi) + X_CARRIAGE_OFFSET - X_BUMPER_WIDTH - X_HALF_WIDTH;
        // Legs
        } else {
            z = Z_CARRIAGE_FLOOR_OFFSET + Z_CARRIAGE_JOINT_OFFSET + positions.h - X_ARM_LENGTH * cos(std::fabs(theta));
            x = -(X_ARM_LENGTH * sin(std::fabs(theta))) + X_CARRIAGE_OFFSET - X_BUMPER_WIDTH - X_HALF_WIDTH;
        }
    }
    return frc::Pose2d((units::length::meter_t)x,(units::length::meter_t)z,(units::angle::degree_t)w);
}

void Elevarm::InitSendable(wpi::SendableBuilder& builder)
{
    builder.SetSmartDashboardType("Subsystem");

    builder.AddDoubleProperty(
        "Previous State Piece",
        [this] { return intake->getPrevPiece(); },
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
        [this] { return intake->getFuturePiece(); },
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
        "Future State Target Wrist",
        [this]{ return futureState.targetPose.wrist; },
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
        "Forward Kinematics Y",
        [this]{ return futureState.resultKinematics.Y().to<double>(); },
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
        [this]{ return futureState.frontMinAngle; },
        nullptr
    );
    builder.AddDoubleProperty(
        "Min angle back",
        [this]{ return futureState.backMinAngle; },
        nullptr
    );
    builder.AddDoubleProperty(
        "Arm CANcoder",
        [this]{ return armCANcoder.GetAbsolutePosition(); },
        nullptr
    );
    builder.AddDoubleProperty(
        "Wrist CANcoder",
        [this]{ return wristCANcoder.GetAbsolutePosition(); },
        nullptr
    );
    builder.AddBooleanProperty(
        "atWrist",
        [this]{ return futureState.atWrist; },
        nullptr
    );
    builder.AddBooleanProperty(
        "atArm",
        [this]{ return futureState.atArm; },
        nullptr
    );
    builder.AddBooleanProperty(
        "atCarriage",
        [this]{ return futureState.atCarriage; },
        nullptr
    );
}

frc2::FunctionalCommand * Elevarm::getAutoCommand(std::string pieceState, std::string directionState, std::string positionState){
    Piece eaPieceState = stringToPieceState(pieceState);
    Direction eaDirectionState = stringToDirectionState(directionState);
    Position eaPositionState = stringToPositionState(positionState);
    return new frc2::FunctionalCommand(
        // OnInit
        [&, eaPieceState, eaDirectionState, eaPositionState](){
            setFuturePiece(eaPieceState);
            futureState.directionState = eaDirectionState;
            futureState.positionState = eaPositionState;
            table->PutNumber("piece", intake->getFuturePiece());
            table->PutNumber("dir", futureState.directionState);
            table->PutNumber("pos", futureState.positionState);
        },
        //onExecute
        [](){
           
        }, 
        [&](bool){
            previousState = futureState;
            setPrevPiece(intake->getFuturePiece());
        }, // onEnd
        [&, eaPieceState, eaDirectionState, eaPositionState](){ //isFinished
            return (previousState.directionState == eaDirectionState && previousState.positionState == eaPositionState) || futureState.pitModeEnabled;
        },
        {}
    );
}

void Elevarm::setArmPIDF(bool isAuto) {
    isAuto ? armRotateMotor.setPIDF(autoRotatePID,0) : armRotateMotor.setPIDF(rotatePID,0);
}


void Elevarm::setPrevPiece(Piece piece){
     intake->prevState.pieceState = piece;
}
void Elevarm::setFuturePiece(Piece piece){
    intake->state.pieceState = piece;
}

frc2::FunctionalCommand * Elevarm::getRotatePIDSetterCommand(bool isAuto){
    return new frc2::FunctionalCommand(
        // OnInit
        [&]() {}, 
        //onExecute
        [&, isAuto](){
            setArmPIDF(isAuto);
        }, 
        [&](bool){}, // onEnd
        [&](){
            return true;
        }, //isFinished
        {}
    );
}