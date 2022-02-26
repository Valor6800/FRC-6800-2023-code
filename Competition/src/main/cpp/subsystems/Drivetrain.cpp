/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

//start button pulls swerve 0 positions from file
//dashboard button pushes current swerve positions to file


#include "subsystems/Drivetrain.h"
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <iostream>

Drivetrain::Drivetrain() : ValorSubsystem(),
                           driverController(NULL),
                           pigeon(DriveConstants::PIGEON_CAN),
                           kinematics(motorLocations[0], motorLocations[1], motorLocations[2], motorLocations[3]),
                           odometry(kinematics, frc::Rotation2d{units::radian_t{0}}),
                           config(units::velocity::meters_per_second_t{SwerveConstants::AUTO_MAX_SPEED_MPS}, units::acceleration::meters_per_second_squared_t{SwerveConstants::AUTO_MAX_ACCEL_MPSS}),
                           reverseConfig(units::velocity::meters_per_second_t{SwerveConstants::AUTO_MAX_SPEED_MPS}, units::acceleration::meters_per_second_squared_t{SwerveConstants::AUTO_MAX_ACCEL_MPSS}),
                           thetaController{DriveConstants::KPT, DriveConstants::KIT, DriveConstants::KDT, frc::ProfiledPIDController<units::radians>::Constraints(units::angular_velocity::radians_per_second_t{SwerveConstants::AUTO_MAX_ROTATION_RPS}, units::angular_acceleration::radians_per_second_squared_t{SwerveConstants::AUTO_MAX_ROTATION_ACCEL_RPSS})}
{
    frc2::CommandScheduler::GetInstance().RegisterSubsystem(this);
    init();
}

Drivetrain::~Drivetrain()
{
    for (int i = 0; i < 4; i++)
    {
        delete azimuthMotors[i];
        delete driveMotors[i];
        delete swerveModules[i];
        delete magEncoders[i];
    }
}

void Drivetrain::configSwerveModule(int i)
{
    azimuthMotors.push_back(new WPI_TalonFX(DriveConstants::AZIMUTH_CANS[i]));
    azimuthMotors[i]->ConfigFactoryDefault();
    azimuthMotors[i]->SetInverted(false);
    azimuthMotors[i]->ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor, 0, 10);
    azimuthMotors[i]->ConfigAllowableClosedloopError(0, 0);
    azimuthMotors[i]->Config_IntegralZone(0, 0);
    azimuthMotors[i]->Config_kF(0, SwerveConstants::KF);
    azimuthMotors[i]->Config_kD(0, SwerveConstants::KD);
    azimuthMotors[i]->Config_kI(0, SwerveConstants::KI);
    azimuthMotors[i]->Config_kP(0, SwerveConstants::KP);
    azimuthMotors[i]->ConfigMotionAcceleration(SwerveConstants::MOTION_ACCELERATION);
    azimuthMotors[i]->ConfigMotionCruiseVelocity(SwerveConstants::MOTION_CRUISE_VELOCITY);
    azimuthMotors[i]->SetNeutralMode(NeutralMode::Brake);

    driveMotors.push_back(new WPI_TalonFX(DriveConstants::DRIVE_CANS[i]));
    driveMotors[i]->ConfigFactoryDefault();
    driveMotors[i]->SetInverted(false);
    driveMotors[i]->ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor, 0, 10);
    driveMotors[i]->SetNeutralMode(NeutralMode::Coast);

    magEncoders.push_back(new frc::DutyCycleEncoder(DriveConstants::MAG_ENCODER_PORTS[i]));
    magEncoders[i]->SetDistancePerRotation(4096.0);

    swerveModules.push_back(new ValorSwerve(azimuthMotors[i], driveMotors[i], magEncoders[i], motorLocations[i]));
}

void Drivetrain::init()
{
    limeTable = nt::NetworkTableInstance::GetDefault().GetTable("limelight");
    initTable("Drivetrain");
    pigeon.Calibrate();

    thetaController.EnableContinuousInput(units::radian_t(-wpi::numbers::pi), units::radian_t(wpi::numbers::pi));
    config.SetKinematics(getKinematics());
    reverseConfig.SetKinematics(getKinematics());
    reverseConfig.SetReversed(true);
    

    for (int i = 0; i < 4; i++)
    {
        configSwerveModule(i);
    }
    table->PutBoolean("Save Swerve Mag Encoder", false);
    state.saveToFileDebouncer = false;

    // for (size_t i = 0; i < swerveModules.size(); i++) {
    //     swerveModules[i]->loadAndSetAzimuthZeroReference();
    // }

    resetState();
    std::cout <<"init drivetrain" << std::endl;
}

void Drivetrain::setController(frc::XboxController *controller)
{
    driverController = controller;
}

std::vector<ValorSwerve *> Drivetrain::getSwerveModules()
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
    if (!driverController)
    {
        return;
    }

    // driver inputs
    state.leftStickX = driverController->GetLeftX();
    state.leftStickY = driverController->GetLeftY();
    state.rightStickX = driverController->GetRightX();
    state.rightStickY = driverController->GetRightY();

    state.bButtonPressed = driverController->GetBButton();
    state.aButtonPressed = driverController->GetAButton();
    state.xButtonPressed = driverController->GetXButton();
    state.yButtonPressed = driverController->GetYButton();

    state.startButtonPressed = driverController->GetStartButtonPressed();

    state.stickPressed = std::abs(state.leftStickY) > 0.05 || 
    std::abs(state.leftStickX) > 0.05 ||
    std::abs(state.rightStickX) > 0.05;

    //state.dPadDownPressed = driverController->GetPOV(frc::GenericHID::)

    state.tracking = driverController->GetRightTriggerAxis() > 0.25;
}

void Drivetrain::analyzeDashboard()
{
    state.backButtonPressed = driverController->GetBackButtonPressed();


    table->PutNumber("Robot X", getPose_m().X().to<double>());
    table->PutNumber("Robot Y", getPose_m().Y().to<double>());
    table->PutNumber("Robot Theta", getPose_m().Rotation().Degrees().to<double>());
    table->PutNumber("Pigeon Theta", getPigeon().Degrees().to<double>());


    table->PutNumber("left stick x", driverController->GetLeftX());
    table->PutNumber("left stick Y", driverController->GetLeftY());
    table->PutNumber("right stick x", driverController->GetRightX());
    table->PutNumber("right stick Y", driverController->GetRightY());

    for (int i = 0; i < 4; i++)
    {
        table->PutNumber("Wheel " + std::to_string(i) + " angle", swerveModules[i]->getAzimuthRotation2d().Degrees().to<double>());
        table->PutNumber("Wheel " + std::to_string(i) + " velocity", swerveModules[i]->getDriveSpeed_mps().to<double>());

        table->PutNumber("Wheel " + std::to_string(i) + " mag encoder", swerveModules[i]->getMagEncoderCount());
        table->PutNumber("Wheel " + std::to_string(i) + " mag encoder converted", swerveModules[i]->convertMagEncoderToAzimuthEncoder(swerveModules[i]->getMagEncoderCount()));
        table->PutNumber("Wheel " + std::to_string(i) + " azimuth encoder", swerveModules[i]->getAzimuthEncoderCount());
    }

    // Only save to file once. Wait until switch is toggled to run again
    if (table->GetBoolean("Save Swerve Mag Encoder",false) && !state.saveToFileDebouncer) {
        for (ValorSwerve *module : swerveModules)
        {
            module->storeAzimuthZeroReference();
        }
        state.saveToFileDebouncer = true;
    } else if (!table->GetBoolean("Save Swerve Mag Encoder",false)) {
        state.saveToFileDebouncer = false;
    }

    odometry.Update(getPigeon(),
                    swerveModules[0]->getState(),
                    swerveModules[1]->getState(),
                    swerveModules[2]->getState(),
                    swerveModules[3]->getState());

    if (state.backButtonPressed){
        resetGyro();
    }
}

void Drivetrain::setMotorMode(bool enabled){
    for (int i = 0; i < 4; i++){
        if (enabled){
            driveMotors[i]->SetNeutralMode(NeutralMode::Brake);
        }
        else{
            driveMotors[i]->SetNeutralMode(NeutralMode::Coast);
        }
    } 
}

void Drivetrain::assignOutputs()
{
    // Get the x speed. We are inverting this because Xbox controllers return
    // negative values when we push forward.
    double xSpeed = std::abs(state.leftStickY) > DriveConstants::kDeadbandY ? fabs(state.leftStickY) * -state.leftStickY : 0 ;

    // Get the y speed or sideways/strafe speed. We are inverting this because
    // we want a positive value when we pull to the left. Xbox controllers
    // return positive values when you pull to the right by default.
    double ySpeed = std::abs(state.leftStickX) > DriveConstants::kDeadbandX ? fabs(state.leftStickX) * -state.leftStickX : 0;

    // Get the rate of angular rotation. We are inverting this because we want a
    // positive value when we pull to the left (remember, CCW is positive in
    // mathematics). Xbox controllers return positive values when you pull to
    // the right by default.
    double rot = std::abs(state.rightStickX) > DriveConstants::kDeadbandX ? fabs(state.rightStickX) * -state.rightStickX : 0;

    units::meters_per_second_t xSpeedMPS = units::meters_per_second_t{xSpeed * SwerveConstants::DRIVE_MAX_SPEED_MPS};
    units::meters_per_second_t ySpeedMPS = units::meters_per_second_t{ySpeed * SwerveConstants::DRIVE_MAX_SPEED_MPS};
    units::radians_per_second_t rotRPS = units::radians_per_second_t{rot * SwerveConstants::ROTATION_MAX_SPEED_RPS};

    double heading = getPose_m().Rotation().Degrees().to<double>();
    if (state.bButtonPressed) {
        rotRPS = units::radians_per_second_t{angleWrap(heading - 90) * DriveConstants::TURN_KP};
    } else if (state.aButtonPressed) {
        rotRPS = units::radians_per_second_t{angleWrap(heading - 180) * DriveConstants::TURN_KP};
    } else if (state.xButtonPressed) {
        rotRPS = units::radians_per_second_t{angleWrap(heading - 270) * DriveConstants::TURN_KP};
    } else if (state.yButtonPressed) {
        rotRPS = units::radians_per_second_t{angleWrap(heading) * DriveConstants::TURN_KP};
    }

    // Limelight Tracking
    if (state.tracking) {
        rotRPS = units::radians_per_second_t{-limeTable->GetNumber("tx", 0.0) * DriveConstants::LIMELIGHT_KP * limeTable->GetNumber("tv", 0) * SwerveConstants::ROTATION_MAX_SPEED_RPS};
    }

    if (state.startButtonPressed){
        x0y0 = frc::Pose2d(8.514_m, 1.771_m, frc::Rotation2d(182.1_deg));

        goZeroZero = frc::TrajectoryGenerator::GenerateTrajectory(
            getPose_m(),
            {},
            x0y0,
            reverseConfig);

        cmd_go_zero_zero = new frc2::SwerveControllerCommand<4>(
            goZeroZero,
            [&] () { return getPose_m(); },
            getKinematics(),
            frc2::PIDController(DriveConstants::KPX, DriveConstants::KIX, DriveConstants::KDX),
            frc2::PIDController(DriveConstants::KPY, DriveConstants::KIY, DriveConstants::KDY),
            thetaController,
            [this] (auto states) { setModuleStates(states); },
            {this} //used to be "drivetrain"
        );

        // cmd_go_zero_zero->Schedule();
    }
    if (state.stickPressed){
        cmd_go_zero_zero->Cancel();
    }
    drive(xSpeedMPS, ySpeedMPS, rotRPS, true);
}

//hack fix is removing reseting swerve modules from reset state
//TODO: clean up reset state logic across all subsystems and in robot.cpp in general
void Drivetrain::resetState()
{
    state.tracking = false;

    // resetOdometry(frc::Pose2d{0_m, 0_m, 0_rad});
    for (size_t i = 0; i < swerveModules.size(); i++) {
        swerveModules[i]->loadAndSetAzimuthZeroReference();
    }
}

frc::SwerveDriveKinematics<4>& Drivetrain::getKinematics()
{
    return kinematics;
}

frc::Pose2d Drivetrain::getPose_m()
{
    return odometry.GetPose();
}

void Drivetrain::resetGyro(){
    frc::Pose2d initialPose = getPose_m();
    frc::Pose2d desiredPose = frc::Pose2d(initialPose.X(), initialPose.Y(), frc::Rotation2d(0_deg));
    resetOdometry(desiredPose);
}



void Drivetrain::resetOdometry(frc::Pose2d pose)
{
    odometry.ResetPosition(pose, getPigeon());
}

frc::Rotation2d Drivetrain::getPigeon() 
{
    return pigeon.GetRotation2d();
}


void Drivetrain::resetDriveEncoders()
{
    for (size_t i = 0; i < swerveModules.size(); i++)
    {
        swerveModules[i]->resetDriveEncoder();
    }
}



void Drivetrain::drive(units::meters_per_second_t vx_mps, units::meters_per_second_t vy_mps, units::radians_per_second_t omega_radps, bool isFOC)
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

wpi::array<frc::SwerveModuleState, 4> Drivetrain::getModuleStates(units::meters_per_second_t vx_mps,
                                                                  units::meters_per_second_t vy_mps,
                                                                  units::radians_per_second_t omega_radps,
                                                                  bool isFOC)
{
    frc::ChassisSpeeds chassisSpeeds = isFOC ? frc::ChassisSpeeds::FromFieldRelativeSpeeds(vx_mps,
                                                                                           vy_mps,
                                                                                           omega_radps,
                                                                                           odometry.GetPose().Rotation())
                                             : frc::ChassisSpeeds{vx_mps, vy_mps, omega_radps};
    auto states = kinematics.ToSwerveModuleStates(chassisSpeeds);
    kinematics.DesaturateWheelSpeeds(&states, units::meters_per_second_t{SwerveConstants::DRIVE_MAX_SPEED_MPS});
    return states;
}

void Drivetrain::setModuleStates(wpi::array<frc::SwerveModuleState, 4> desiredStates)
{
    kinematics.DesaturateWheelSpeeds(&desiredStates, units::meters_per_second_t{SwerveConstants::DRIVE_MAX_SPEED_MPS});
    for (int i = 0; i < 4; i++)
    {
        swerveModules[i]->setDesiredState(desiredStates[i], true);
    }
}