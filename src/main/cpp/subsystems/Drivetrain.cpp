/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
#include "subsystems/Drivetrain.h"
#include <frc/DriverStation.h>
#include <iostream>
#include <math.h>

#define TXRANGE  30.0f
#define KPIGEON 2.0f
#define KLIMELIGHT -29.8f
// #define KP_LOCK 0.2f
#define KP_LIMELIGHT 0.7f

#define KPX 60.0f //50
#define KIX 0.0f //0
#define KDX 0.0f //.1
#define KFX 0.0f

#define KPY 60.0f //65
#define KIY 0.0f //0
#define KDY 0.0f //.1
#define KFY 0.0f

#define KPT 15.0f
#define KIT 0.0f
#define KDT 0.0f
#define KFT 0.0f

#define AZIMUTH_K_P 0.00001f
#define AZIMUTH_K_I 0.0f
#define AZIMUTH_K_D 0.0f
#define AZIMUTH_K_E 0.0027f

#define AZIMUTH_K_VEL 10.0f
#define AZIMUTH_K_ACC_MUL 0.05f

#define DRIVE_K_P 0.001f
#define DRIVE_K_I 0.0f
#define DRIVE_K_D 0.0f
#define DRIVE_K_E 0.0027f

#define DRIVE_K_VEL 6.0f
#define DRIVE_K_ACC_MUL 0.05f

#define MOTOR_FREE_SPEED 6380.0f
#define WHEEL_DIAMETER_M 0.0973f //0.1016
#define DRIVE_GEAR_RATIO 5.51f
#define AZIMUTH_GEAR_RATIO 13.37f
#define AUTO_MAX_SPEED 10.0f
#define AUTO_MAX_ACCEL_SECONDS 5.33f //5.33
#define ROT_SPEED_MUL 2.0f

#define AUTO_VISION_THRESHOLD 4.0f //meters
#define FIELD_LENGTH 16.5f

#define MODULE_DIFF 0.206375f

#define X_TIME 214.85f

#define MODULE_DIFF_XS {1, 1, -1, -1}
#define MODULE_DIFF_YS {1, -1, -1, 1}

#define DRIVETRAIN_CAN_BUS ""
#define PIGEON_CAN_BUS "baseCAN"

Drivetrain::Drivetrain(frc::TimedRobot *_robot) : ValorSubsystem(_robot, "Drivetrain"),
                        driveMaxSpeed(MOTOR_FREE_SPEED / 60.0 / DRIVE_GEAR_RATIO * WHEEL_DIAMETER_M * M_PI),
                        swerveModuleDiff(units::meter_t(MODULE_DIFF)),
                        rotMaxSpeed(ROT_SPEED_MUL * 2 * M_PI),
                        autoMaxSpeed(AUTO_MAX_SPEED),
                        autoMaxAccel(AUTO_MAX_SPEED/AUTO_MAX_ACCEL_SECONDS),
                        rotMaxAccel(rotMaxSpeed * 0.5),
                        pigeon(CANIDs::PIGEON_CAN, PIGEON_CAN_BUS),
                        motorLocations(wpi::empty_array),
                        initPositions(wpi::empty_array),
                        kinematics(NULL),
                        estimator(NULL),
                        config(NULL),
                        swerveNoError(true)
{
    frc2::CommandScheduler::GetInstance().RegisterSubsystem(this);
    init();
}

Drivetrain::~Drivetrain()
{
    for (int i = 0; i < SWERVE_COUNT; i++)
    {
        delete azimuthControllers[i];
        delete driveControllers[i];
        delete swerveModules[i];
    }

    delete kinematics;
    delete estimator;
    delete config;
}

void Drivetrain::configSwerveModule(int i)
{

   int MDX[] = MODULE_DIFF_XS;
   int MDY[] = MODULE_DIFF_YS;

    motorLocations[i] = frc::Translation2d{swerveModuleDiff * MDX[i],
                                           swerveModuleDiff * MDY[i]};

    ValorPIDF azimuthPID;
    azimuthPID.velocity = AZIMUTH_K_VEL;
    azimuthPID.acceleration = AZIMUTH_K_ACC_MUL;
    azimuthPID.P = AZIMUTH_K_P;
    azimuthPID.I = AZIMUTH_K_I;
    azimuthPID.D = AZIMUTH_K_D;
    azimuthPID.error = AZIMUTH_K_E;

    azimuthControllers.push_back(new SwerveAzimuthMotor(CANIDs::AZIMUTH_CANS[i],
                                                      ValorNeutralMode::Brake,
                                                      true,
                                                      DRIVETRAIN_CAN_BUS));
    azimuthControllers[i]->setConversion(1.0 / AZIMUTH_GEAR_RATIO);
    azimuthControllers[i]->setPIDF(azimuthPID, 0);

    ValorPIDF drivePID;
    drivePID.velocity = DRIVE_K_VEL;
    drivePID.acceleration = DRIVE_K_ACC_MUL;
    drivePID.P = DRIVE_K_P;
    drivePID.I = DRIVE_K_I;
    drivePID.D = DRIVE_K_D;
    drivePID.error = DRIVE_K_E;

    driveControllers.push_back(new SwerveDriveMotor(CANIDs::DRIVE_CANS[i],
                                                    ValorNeutralMode::Coast,
                                                    false,
                                                    DRIVETRAIN_CAN_BUS));
    driveControllers[i]->setConversion(1.0 / DRIVE_GEAR_RATIO * M_PI * WHEEL_DIAMETER_M);
    driveControllers[i]->setPIDF(drivePID, 0);

    swerveModules.push_back(new ValorSwerve<SwerveAzimuthMotor, SwerveDriveMotor>(azimuthControllers[i], driveControllers[i], motorLocations[i]));
    swerveModules[i]->setMaxSpeed(driveMaxSpeed);
    

}

void Drivetrain::resetState()
{
    resetDriveEncoders();
    pullSwerveModuleZeroReference();
    resetOdometry(frc::Pose2d{0_m, 0_m, 0_rad});
}

void Drivetrain::init()
{
    limeTable = nt::NetworkTableInstance::GetDefault().GetTable("limelight");
    pigeon.Calibrate();    

    initPositions.fill(frc::SwerveModulePosition{0_m, frc::Rotation2d(0_rad)});

    for (int i = 0; i < SWERVE_COUNT; i++)
    {
        configSwerveModule(i);
    }

    kinematics = new frc::SwerveDriveKinematics<SWERVE_COUNT>(motorLocations);
    estimator = new frc::SwerveDrivePoseEstimator<SWERVE_COUNT>(*kinematics, pigeon.GetRotation2d(), initPositions, frc::Pose2d{0_m, 0_m, 0_rad});
    config = new frc::TrajectoryConfig(units::velocity::meters_per_second_t{autoMaxSpeed}, units::acceleration::meters_per_second_squared_t{autoMaxAccel});

    xPIDF.P = KPX;
    xPIDF.I = KIX;
    xPIDF.D = KDX;
    xPIDF.F = KFX;

    yPIDF.P = KPY;
    yPIDF.I = KIY;
    yPIDF.D = KDY;
    yPIDF.F = KFY;

    thetaPIDF.P = KPT;
    thetaPIDF.I = KIT;
    thetaPIDF.D = KDT;
    thetaPIDF.F = KFT;

    table->PutNumber("Vision Std", 3.0);
    table->PutBoolean("Load Swerve Mag Encoder", false);

    table->PutNumber("KPLIMELIGHT", KP_LIMELIGHT);

    limeTable->PutNumber("pipeline", LimelightPipes::TAPE_HIGH);    
    limeTable->PutNumber("ledMode", 1);

    state.lock = false;

    resetState();
}

std::vector<ValorSwerve<Drivetrain::SwerveAzimuthMotor, Drivetrain::SwerveDriveMotor> *> Drivetrain::getSwerveModules()
{
    return swerveModules;
}

double Drivetrain::angleWrap(double degrees)
{
    double zeroTo360 = degrees + 180;
    double start = fmod(zeroTo360, 360); //will work for positive angles

    //angle is (-360, 0), add 360 to make (0, 360)
    if (start < 0)
    {
        start += 360;
    }

    //bring it back to (-180, 180)
    return start - 180;
}

void Drivetrain::assessInputs()
{
    if (!driverGamepad) return;

    if (driverGamepad->GetBackButtonPressed()) {
        resetGyro();
    }

    state.adas = driverGamepad->GetAButton();
    state.topTape = operatorGamepad->DPadUp();
    state.bottomTape = operatorGamepad->DPadRight();
    state.lock = state.adas || driverGamepad->GetBButton();

    state.xSpeed = driverGamepad->leftStickY(2);
    state.ySpeed = driverGamepad->leftStickX(2);
    if (!state.lock){
    state.rot = driverGamepad->rightStickX(3);
    }

    state.xPose = driverGamepad->GetXButton();
}

void Drivetrain::analyzeDashboard()
{
    if (table->GetBoolean("Load Swerve Mag Encoder",false))
        pullSwerveModuleZeroReference();

    estimator->UpdateWithTime(frc::Timer::GetFPGATimestamp(),
                            getPigeon(),
                            {
                                swerveModules[0]->getModulePosition(),
                                swerveModules[1]->getModulePosition(),
                                swerveModules[2]->getModulePosition(),
                                swerveModules[3]->getModulePosition()
                            });

    if (limeTable->GetNumber("tv", 0) == 1.0) {
        
        std::vector<double> poseArray;
        if (frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kBlue) {
            poseArray = limeTable->GetNumberArray("botpose_wpiblue", std::span<const double>());
        } else {
            poseArray = limeTable->GetNumberArray("botpose_wpired", std::span<const double>());
        }

        if (poseArray.size() >= 6){
            double x = poseArray[0], y = poseArray[1], angle = poseArray[5];
            
            frc::Pose2d botpose = frc::Pose2d{units::meter_t(x), units::meter_t(y), units::degree_t(angle)};
            state.prevVisionPose = state.visionPose;
            state.visionPose = frc::Pose2d{botpose.X(), botpose.Y(), getPose_m().Rotation()};

            state.visionOdomDiff = (botpose - getPose_m()).Translation().Norm().to<double>();
            double visionStd = table->GetNumber("Vision Std", 3.0);

            if (((x < AUTO_VISION_THRESHOLD && x > 0) || 
                (x > (FIELD_LENGTH - AUTO_VISION_THRESHOLD) && x < FIELD_LENGTH)) &&
                (state.visionPose - state.prevVisionPose).Translation().Norm().to<double>() < 1.0)
            {
                // estimator->AddVisionMeasurement(
                //     state.visionPose,  
                //     frc::Timer::GetFPGATimestamp(),
                //     {visionStd, visionStd, visionStd}
                // ); 
            }
            
            
            if (driverGamepad->GetStartButton()){
                resetOdometry(botpose);
            }
        }
    }
}

void Drivetrain::assignOutputs()
{    
    if (state.lock){angleLock();}
    state.xSpeedMPS = units::velocity::meters_per_second_t{state.xSpeed * driveMaxSpeed};
    state.ySpeedMPS = units::velocity::meters_per_second_t{state.ySpeed * driveMaxSpeed};
    state.rotRPS = units::angular_velocity::radians_per_second_t{state.rot * rotMaxSpeed};

    if (state.xPose){
        setXMode();
    } else if (state.adas){
        limeTable->PutNumber("ledMode", 3);
        if (state.bottomTape) {
            adas(LimelightPipes::TAPE_MID);
        } else {
            adas(LimelightPipes::TAPE_HIGH);
        }
        drive(state.xSpeedMPS, state.ySpeedMPS, state.rotRPS, true);
    } 
    else {
        setDriveMotorNeutralMode(ValorNeutralMode::Coast);
        if (robot->IsTeleop()){
            limeTable->PutNumber("pipeline", LimelightPipes::TAPE_HIGH);    
            limeTable->PutNumber("ledMode", 1);
        }
        drive(state.xSpeedMPS, state.ySpeedMPS, state.rotRPS, true);
    }
}

void Drivetrain::pullSwerveModuleZeroReference(){
    swerveNoError = true;
    for (size_t i = 0; i < swerveModules.size(); i++) {
        swerveNoError &= swerveModules[i]->loadAndSetAzimuthZeroReference();
    }
}

frc::SwerveDriveKinematics<SWERVE_COUNT>* Drivetrain::getKinematics()
{
    return kinematics;
}

frc::Pose2d Drivetrain::getPose_m()
{
    return estimator->GetEstimatedPosition();
}

frc::Pose2d Drivetrain::getVisionPose(){
    if (limeTable->GetNumber("tv", 0) != 1.0)
        return frc::Pose2d{0_m, 0_m, 0_deg};

    std::vector<double> poseArray;
    if (frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kBlue) {
        poseArray = limeTable->GetNumberArray("botpose_wpiblue", std::span<const double>());
    } else {
        poseArray = limeTable->GetNumberArray("botpose_wpired", std::span<const double>());
    }

    if (poseArray.size() < 6)
        return frc::Pose2d{0_m, 0_m, 0_deg}; 

    return frc::Pose2d{
        units::meter_t{poseArray[0]},
        units::meter_t{poseArray[1]},
        units::degree_t{poseArray[5]}
    };
}

void Drivetrain::addVisionMeasurement(frc::Pose2d visionPose, double doubt=1){
    if (limeTable->GetNumber("tv", 0) == 1.0)   
        estimator->AddVisionMeasurement(
            visionPose,  
            frc::Timer::GetFPGATimestamp(),
            {doubt, 999999, 999999}
        ); 
}

void Drivetrain::resetGyro(){
    frc::Pose2d initialPose = getPose_m();
    frc::Pose2d desiredPose = frc::Pose2d(initialPose.X(), initialPose.Y(), frc::Rotation2d(0_deg));
    resetOdometry(desiredPose);
}

void Drivetrain::resetOdometry(frc::Pose2d pose)
{

    limeTable->PutNumber("pipeline", LimelightPipes::APRIL_TAGS);

    wpi::array<frc::SwerveModulePosition, SWERVE_COUNT> modulePositions = wpi::array<frc::SwerveModulePosition, SWERVE_COUNT>(wpi::empty_array);

    for (size_t i = 0; i < swerveModules.size(); i++)
    {
        modulePositions[i] = swerveModules[i]->getModulePosition();
    }

    estimator->ResetPosition(getPigeon(), modulePositions, pose);
}

frc::Rotation2d Drivetrain::getPigeon() 
{
    return pigeon.GetRotation2d();
}

units::degree_t Drivetrain::getGlobalPitch(){
    double pitch = pigeon.GetPitch() / 180 * M_PI, yaw = getPose_m().Rotation().Degrees().to<double>() / 180 * M_PI, roll = pigeon.GetRoll() / 180 * M_PI;
    double globalPitch = (std::cos(yaw) * pitch + std::sin(yaw) * roll) * 180 / M_PI; 
    return units::degree_t{globalPitch};
}

void Drivetrain::resetDriveEncoders()
{
    for (size_t i = 0; i < swerveModules.size(); i++)
    {
        swerveModules[i]->resetDriveEncoder();
    }
}

void Drivetrain::drive(units::velocity::meters_per_second_t vx_mps, units::velocity::meters_per_second_t vy_mps, units::angular_velocity::radians_per_second_t omega_radps, bool isFOC)
{
    auto states = getModuleStates(vx_mps,
                                  vy_mps,
                                  omega_radps,
                                  isFOC);
    for (size_t i = 0; i < swerveModules.size(); i++)
    {
        swerveModules[i]->setDesiredState(states[i], true);
    }
}

wpi::array<frc::SwerveModuleState, SWERVE_COUNT> Drivetrain::getModuleStates(units::velocity::meters_per_second_t vx_mps,
                                                                  units::velocity::meters_per_second_t vy_mps,
                                                                  units::angular_velocity::radians_per_second_t omega_radps,
                                                                  bool isFOC)
{
    frc::ChassisSpeeds chassisSpeeds = isFOC ? frc::ChassisSpeeds::FromFieldRelativeSpeeds(vx_mps,
                                                                                           vy_mps,
                                                                                           omega_radps,
                                                                                           estimator->GetEstimatedPosition().Rotation())
                                             : frc::ChassisSpeeds{vx_mps, vy_mps, omega_radps};
    auto states = kinematics->ToSwerveModuleStates(chassisSpeeds);
    kinematics->DesaturateWheelSpeeds(&states, units::velocity::meters_per_second_t{driveMaxSpeed});
    return states;
}

void Drivetrain::setModuleStates(wpi::array<frc::SwerveModuleState, SWERVE_COUNT> desiredStates)
{ 
    kinematics->DesaturateWheelSpeeds(&desiredStates, units::velocity::meters_per_second_t{autoMaxSpeed});
    for (int i = 0; i < SWERVE_COUNT; i++)
    {
        swerveModules[i]->setDesiredState(desiredStates[i], false);
    }
}

void Drivetrain::angleLock(){
    if (0 > getPose_m().Rotation().Degrees().to<double>()){
        state.rot = (-getPose_m().Rotation().Degrees().to<double>()/180) - 1.0;
    } else{
        state.rot = (-getPose_m().Rotation().Degrees().to<double>()/180) + 1.0;
    }
    
}

void Drivetrain::adas(LimelightPipes pipe){
    limeTable->PutNumber("pipeline", pipe);
    if (limeTable->GetNumber("pipeline", -1) == LimelightPipes::TAPE_HIGH || limeTable->GetNumber("pipeline", -1) == LimelightPipes::TAPE_MID){
        if (limeTable->GetNumber("tv",0) == 1){
            double tx = limeTable->GetNumber("tx",0);
            double normalizedTx = tx / KLIMELIGHT;
            double kPlimeLight = table->GetNumber("KPLIMELIGHT", KP_LIMELIGHT);
            state.ySpeedMPS = units::velocity::meters_per_second_t(((std::fabs(normalizedTx) <= 1 ? normalizedTx : std::copysignf(1.0, normalizedTx) ) * kPlimeLight * -driveMaxSpeed));
        }
    }
}

void Drivetrain::setLimelightPipeline(LimelightPipes pipeline){
    limeTable->PutNumber("pipeline", pipeline);
}

frc2::FunctionalCommand* Drivetrain::getResetOdom() {
    return new frc2::FunctionalCommand(
        [&]{ // onBegin
            limeTable->PutNumber("pipeline", 0);
            state.startTimestamp = frc::Timer::GetFPGATimestamp();
        },
        [&]{ // continuously running
            frc::Pose2d visionPose = getVisionPose();
            table->PutNumber("resetting maybe", true);
            if (limeTable->GetNumber("tv", 0.0) == 1.0 && (visionPose.X() > 0_m && visionPose.Y() > 0_m)){
                table->PutNumber("resetting odom", table->GetNumber("resetting odom", 0) + 1);
                addVisionMeasurement(visionPose, 1.0);
                table->PutBoolean("resetting", true);
            }
            else {
                table->PutBoolean("resetting", false);
            }
        },
        [&](bool){ // onEnd
                
        },
        [&]{ // isFinished
            return (frc::Timer::GetFPGATimestamp() - state.startTimestamp) > 1.0_s;
        },
        {}
    );
}

frc2::FunctionalCommand* Drivetrain::getVisionAutoLevel(){
    return new frc2::FunctionalCommand(
        [&](){
            state.abovePitchThreshold = false;
            limeTable->PutNumber("pipeline", LimelightPipes::APRIL_TAGS);
            state.prevPose = getPose_m();
        },
        [&](){
            limeTable->PutNumber("pipeline", LimelightPipes::APRIL_TAGS);
            if (!state.abovePitchThreshold && std::fabs(getGlobalPitch().to<double>()) > 20)
                state.abovePitchThreshold = true;

            if (!state.abovePitchThreshold){
                state.prevPose = getPose_m();
                state.xSpeed = -0.5;
            } else
                state.xSpeed = -0.3;
                

            frc::Pose2d visionPose = getVisionPose();
            if (std::fabs(state.visionPose.X().to<double>() - state.prevPose.X().to<double>()) < 1.00)
                state.prevPose = state.visionPose;
        },
        [&](bool){
            state.xSpeed = 0.0;
            state.xPose = true;
        }, // onEnd
        [&](){
            return (state.prevPose.X() < 4.05_m) || (frc::Timer::GetFPGATimestamp().to<double>() - state.matchStart > X_TIME) || (state.abovePitchThreshold && getGlobalPitch().to<double>() > 0);
        },//isFinished
        {}
    );
}

frc2::FunctionalCommand* Drivetrain::getAutoLevel(){
    return new frc2::FunctionalCommand(
        [&](){
            state.stage = 0;
        },
        [&](){
            double pitch = pigeon.GetPitch() / 180 * M_PI, yaw = getPose_m().Rotation().Degrees().to<double>() / 180 * M_PI, roll = pigeon.GetRoll() / 180 * M_PI;
            double globalPitch = (std::cos(yaw) * pitch + std::sin(yaw) * roll) * 180 / M_PI;
            switch (state.stage){
                // There should be a stage for each change in slope between positive and negative, otherwise the stage change might happen at unexpected places
                case 0: // 
                    state.xSpeed = -0.5;
                    if (globalPitch < -20.0)
                        state.stage++;
                    break;
                case 1:
                    state.xSpeed = -0.4;
                    if (globalPitch > -14.7)
                        state.stage++;
                    break;
                case 2:
                    state.xSpeed = -0.2;
                    if (globalPitch < -14.6)
                        state.stage++;
                    break;
                case 3:
                    state.xSpeed = -0.2;
                    if (globalPitch > -14.6)
                        state.stage++;
                    break;
                case 4:
                    state.xSpeed = +0.02;
                    break;
            }
        },
        [&](bool){
            state.xSpeed = 0.0;
            state.xPose = true;
        }, // onEnd
        [&](){
            return state.stage == 4 || (frc::Timer::GetFPGATimestamp().to<double>() - state.matchStart > X_TIME);
        },//isFinished
        {}
    );
}

frc2::FunctionalCommand* Drivetrain::getAutoLevelReversed(){
    return new frc2::FunctionalCommand(
        [&](){
            state.stage = 0;
        },
        [&](){
            double pitch = pigeon.GetPitch() / 180 * M_PI, yaw = getPose_m().Rotation().Degrees().to<double>() / 180 * M_PI, roll = pigeon.GetRoll() / 180 * M_PI;
            double globalPitch = (std::cos(yaw) * pitch + std::sin(yaw) * roll) * 180 / M_PI;
            switch (state.stage){
                // There should be a stage for each change in slope between positive and negative, otherwise the stage change might happen at unexpected places
                case 0: // 
                    state.xSpeed = 0.5;
                    if (globalPitch > 20.0)
                        state.stage++;
                    break;
                case 1:
                    state.xSpeed = 0.4;
                    if (globalPitch < 14.7)
                        state.stage++;
                    break;
                case 2:
                    state.xSpeed = 0.2;
                    if (globalPitch > 14.6)
                        state.stage++;
                    break;
                case 3:
                    state.xSpeed = 0.2;
                    if (globalPitch < 14.6)
                        state.stage++;
                    break;
                case 4:
                    state.xSpeed = -0.02;
                    break;
            }
        },
        [&](bool){
            state.xSpeed = 0.0;
            state.xPose = true;
        }, // onEnd
        [&](){
            return state.stage == 4 || (frc::Timer::GetFPGATimestamp().to<double>() - state.matchStart > X_TIME);
        },//isFinished
        {}
    );
}

frc2::FunctionalCommand* Drivetrain::getAutoClimbOver(){
    return new frc2::FunctionalCommand(
        [&](){
            state.stage = 0;
        },
        [&](){
            switch (state.stage){
                case 0: // On the ground before we've started climbing
                    state.xSpeed = 0.5;
                    if (pigeon.GetPitch() < -16.0)
                        state.stage = 1;
                    break;
                case 1: // Began climbing up -> became balanced -> starting to move down
                    state.xSpeed = 0.4;
                    if (pigeon.GetPitch() > 8.0)
                        state.stage = 2;
                    break;
                case 2: // The climb down
                    state.xSpeed = 0.1;
                    if (pigeon.GetPitch() < 5.0)
                        state.stage = 3;
                    break;
                case 3: // Reached flat ground
                    state.xSpeed = 0.05;
            }
        },
        [&](bool){
            state.xSpeed = 0.05;
        }, // onEnd
        [&](){
            return state.stage == 3;
        },//isFinished
        {}
    );
}

//OLD BALANCE
frc2::FunctionalCommand* Drivetrain::getOLDAutoLevel(){
    return new frc2::FunctionalCommand(
        [&](){
            state.abovePitchThreshold = false;
            state.isLeveled = false;
        }, // OnInit
        [&](){
            if (pigeon.GetPitch() > 16.0) {
                state.abovePitchThreshold = true;
                state.xSpeed = -0.4;
            } else if (state.abovePitchThreshold) {
                if (pigeon.GetPitch() < 11.5 ){
                    state.isLeveled = true;
                    state.xSpeed = 0.02;
                }else if(pigeon.GetPitch() < 16){
                    state.xSpeed = -0.15;
                }else{
                    state.xSpeed = -0.3;
                }
            } else {
                state.xSpeed = -0.3;
            }
        }, //onExecute
        [&](bool){
            state.xSpeed = 0.0;
            state.xPose = true;
        }, // onEnd
        [&](){
            return state.isLeveled;
        },//isFinished
        {}
    );
}

//OLD BALANCE
frc2::FunctionalCommand* Drivetrain::getOLDAutoLevelReversed(){
    return new frc2::FunctionalCommand(
        [&](){
            state.abovePitchThreshold = false;
            state.isLeveled = false;
        }, // OnInit
        [&](){
            if (pigeon.GetPitch() < -16.0) {
                state.abovePitchThreshold = true;
                state.xSpeed = 0.4;
            } else if (state.abovePitchThreshold) {
                if (pigeon.GetPitch() > -11.5 ){
                    state.isLeveled = true;
                    state.xSpeed = -0.02;
                }else if(pigeon.GetPitch() > -16){
                    state.xSpeed = 0.15;
                }else{
                    state.xSpeed = 0.3;
                }
            } else {
                state.xSpeed = 0.3;
            }
        }, //onExecute
        [&](bool){
            state.xSpeed = 0.0;
            state.xPose = true;
        }, // onEnd
        [&](){
            return state.isLeveled;
        },//isFinished
        {}
    );
}

double Drivetrain::getDriveMaxSpeed() {
    return driveMaxSpeed;
}

double Drivetrain::getAutoMaxSpeed() {
    return autoMaxSpeed;
}

double Drivetrain::getAutoMaxAcceleration() {
    return autoMaxAccel;
}


double Drivetrain::getRotationMaxSpeed() {
    return rotMaxSpeed;
}

double Drivetrain::getRotationMaxAcceleration() {
    return rotMaxAccel;
}

frc::TrajectoryConfig & Drivetrain::getTrajectoryConfig() {    
    return *config;
}

ValorPIDF Drivetrain::getXPIDF() {
    return xPIDF;
}

ValorPIDF Drivetrain::getYPIDF() {
    return yPIDF;
}

ValorPIDF Drivetrain::getThetaPIDF() {
    return thetaPIDF;
}

void Drivetrain::setAutoMaxAcceleration(double acceleration, double multiplier)  {
    // autoMaxAccel = multiplier * (acceleration == NULL ? AUTO_MAX_ACCEL_SECONDS : acceleration);
    // if (config == NULL) {
    //     delete config;
    // }
    // config = new frc::TrajectoryConfig(units::velocity::meters_per_second_t{autoMaxSpeed}, units::acceleration::meters_per_second_squared_t{autoMaxAccel});
}

void Drivetrain::setXMode(){
    drive(static_cast<units::velocity::meters_per_second_t>(0),static_cast<units::velocity::meters_per_second_t>(0),static_cast<units::angular_velocity::radians_per_second_t>(0),true);
    azimuthControllers[0]->setPosition(std::round(azimuthControllers[0]->getPosition()) + 0.125);
    azimuthControllers[1]->setPosition(std::round(azimuthControllers[1]->getPosition()) + 0.375);
    azimuthControllers[2]->setPosition(std::round(azimuthControllers[2]->getPosition()) - 0.375);
    azimuthControllers[3]->setPosition(std::round(azimuthControllers[3]->getPosition()) - 0.125);
    setDriveMotorNeutralMode(ValorNeutralMode::Brake);
}

void Drivetrain::setDriveMotorNeutralMode(ValorNeutralMode mode) {
    for (int i = 0; i < SWERVE_COUNT; i++)
    {
        driveControllers[i]->setNeutralMode(mode);
    }
}

frc2::InstantCommand* Drivetrain::getSetXMode(){
    frc2::InstantCommand* cmd_XMode = new frc2::InstantCommand( [&] {
        setXMode();
    });
     return cmd_XMode;
}

void Drivetrain::InitSendable(wpi::SendableBuilder& builder)
    {
        builder.SetSmartDashboardType("Subsystem");

        builder.AddDoubleProperty(
            "diffVisionOdom",
            [this] { return state.visionOdomDiff; },
            nullptr
        );

        builder.AddDoubleProperty(
            "xSpeed",
            [this] { return state.xSpeed; },
            nullptr
        );
        builder.AddDoubleProperty(
            "ySpeed",
            [this] { return state.ySpeed; },
            nullptr
        );
        builder.AddDoubleProperty(
            "rotSpeed",
            [this] { return state.rot; },
            nullptr
        );

        builder.AddDoubleProperty(
            "xSpeedMPS",
            [this] { return state.xSpeedMPS.to<double>(); },
            nullptr
        );
        builder.AddDoubleProperty(
            "ySpeedMPS",
            [this] { return state.ySpeedMPS.to<double>(); },
            nullptr
        );
        builder.AddDoubleProperty(
            "rotSpeedMPS",
            [this] { return state.rotRPS.to<double>(); },
            nullptr
        );
        builder.AddDoubleProperty(
            "x",
            [this] { return getPose_m().X().to<double>(); },
            nullptr
        );
        builder.AddDoubleProperty(
            "y",
            [this] { return getPose_m().Y().to<double>(); },
            nullptr
        );
        builder.AddDoubleProperty(
            "theta",
            [this] { return getPose_m().Rotation().Degrees().to<double>(); },
            nullptr
        );
        builder.AddBooleanProperty(
            "swerveGood",
            [this] { return swerveNoError; },
            nullptr
        );
        builder.AddDoubleProperty(
            "visionX",
            [this] { return state.visionPose.X().to<double>(); },
            nullptr
        );
        builder.AddDoubleProperty(
            "visionY",
            [this] { return state.visionPose.Y().to<double>(); },
            nullptr
        );
        builder.AddDoubleArrayProperty(
            "pose",
            [this] 
            { 
                std::vector<double> pose;
                pose.push_back(getPose_m().X().to<double>());
                pose.push_back(getPose_m().Y().to<double>());
                pose.push_back(getPose_m().Rotation().Degrees().to<double>());
                return pose;
            },
            nullptr
        );
        builder.AddDoubleArrayProperty(
            "visionPose",
            [this] 
            { 
                std::vector<double> pose;
                pose.push_back(state.visionPose.X().to<double>());
                pose.push_back(state.visionPose.Y().to<double>());
                pose.push_back(state.visionPose.Rotation().Degrees().to<double>());
                return pose;
            },
            nullptr
        );
        builder.AddDoubleArrayProperty(
            "prevPose",
            [this] 
            { 
                std::vector<double> pose;
                pose.push_back(state.prevPose.X().to<double>());
                pose.push_back(state.prevPose.Y().to<double>());
                pose.push_back(state.prevPose.Rotation().Degrees().to<double>());
                return pose;
            },
            nullptr
        );
        builder.AddDoubleProperty(
            "pigeonPitch",
            [this]
            {
                return pigeon.GetPitch();
            },
            nullptr
        );
        builder.AddDoubleProperty(
            "pigeoYaw",
            [this]
            {
                return pigeon.GetYaw();
            },
            nullptr
        );
        builder.AddDoubleProperty(
            "pigeonRoll",
            [this]
            {
                return pigeon.GetRoll();
            },
            nullptr
        );
        builder.AddIntegerProperty(
            "stage",
            [this]
            {
                return state.stage;
            },
            nullptr
        );
        builder.AddDoubleArrayProperty(
            "swerve states",
            [this] 
            { 
                std::vector<double> states;
                states.push_back(swerveModules[0]->getState().angle.Degrees().to<double>());
                states.push_back(swerveModules[0]->getState().speed.to<double>());
                states.push_back(swerveModules[1]->getState().angle.Degrees().to<double>());
                states.push_back(swerveModules[1]->getState().speed.to<double>());
                states.push_back(swerveModules[2]->getState().angle.Degrees().to<double>());
                states.push_back(swerveModules[2]->getState().speed.to<double>());
                states.push_back(swerveModules[3]->getState().angle.Degrees().to<double>());
                states.push_back(swerveModules[3]->getState().speed.to<double>());
                return states;
            },
            nullptr
        );
        builder.AddDoubleProperty(
            "globalPitch",
            [this]
            {
                double pitch = pigeon.GetPitch() / 180 * M_PI, yaw = getPose_m().Rotation().Degrees().to<double>() / 180 * M_PI, roll = pigeon.GetRoll() / 180 * M_PI;
                double globalPitch = (std::cos(yaw) * pitch + std::sin(yaw) * roll) * 180 / M_PI;
                return globalPitch;
            },
            nullptr
        );

        builder.AddBooleanProperty(
            "abovePitchThreshold",
            [this]{
                return state.abovePitchThreshold;
            },
            nullptr
        );
    }