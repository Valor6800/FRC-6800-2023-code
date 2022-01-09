/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "subsystems/Drivetrain.h"
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/ChassisSpeeds.h>

Drivetrain::Drivetrain() : ValorSubsystem(),
                           driverController(NULL),
                           navX(frc::SerialPort::Port::kMXP),
                           hasGyroOffset(false),
                           kinematics(motorLocations[0], motorLocations[1], motorLocations[2], motorLocations[3]),
                           odometry(kinematics, frc::Rotation2d{units::radian_t{0}})
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
    azimuthMotors[i]->SetInverted(true);
    azimuthMotors[i]->ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor, 0, 10);
    azimuthMotors[i]->ConfigAllowableClosedloopError(0, 0);
    azimuthMotors[i]->Config_IntegralZone(0, 0);
    azimuthMotors[i]->Config_kF(0, 0);
    azimuthMotors[i]->Config_kD(0, SwerveConstants::KD);
    azimuthMotors[i]->Config_kI(0, SwerveConstants::KI);
    azimuthMotors[i]->Config_kP(0, SwerveConstants::KP);
    azimuthMotors[i]->ConfigMotionAcceleration(SwerveConstants::MOTION_ACCELERATION);
    azimuthMotors[i]->ConfigMotionCruiseVelocity(SwerveConstants::MOTION_CRUISE_VELOCITY);

    driveMotors.push_back(new WPI_TalonFX(DriveConstants::DRIVE_CANS[i]));
    driveMotors[i]->ConfigFactoryDefault();
    driveMotors[i]->SetInverted(true);
    driveMotors[i]->ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor, 0, 10);
    driveMotors[i]->SetNeutralMode(NeutralMode::Brake);

    magEncoders.push_back(new frc::DutyCycleEncoder(DriveConstants::MAG_ENCODER_PORTS[i]));
    magEncoders[i]->SetDistancePerRotation(4096.0);

    swerveModules.push_back(new ValorSwerve(azimuthMotors[i], driveMotors[i], magEncoders[i], motorLocations[i]));
}

void Drivetrain::init()
{
    limeTable = nt::NetworkTableInstance::GetDefault().GetTable("limelight");
    initTable("Drivetrain");
    navX.Calibrate();

    for (int i = 0; i < 4; i++)
    {
        configSwerveModule(i);
    }
    //resetState();
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

    //state.dPadDownPressed = driverController->GetPOV(frc::GenericHID::)

    state.tracking = driverController->GetRightBumper();
}

void Drivetrain::analyzeDashboard()
{
    table->PutNumber("Robot X", getPose_m().X().to<double>());
    table->PutNumber("Robot Y", getPose_m().Y().to<double>());

    table->PutNumber("Gyro fused", getHeading().Degrees().to<double>());

    for (int i = 0; i < 4; i++)
    {
        table->PutNumber("Wheel " + std::to_string(i) + " angle", swerveModules[i]->getAzimuthRotation2d().Degrees().to<double>());
        table->PutNumber("Wheel " + std::to_string(i) + " azimuth", swerveModules[i]->getAzimuthAbsoluteEncoderCounts());
        table->PutNumber("Wheel " + std::to_string(i) + " relative", swerveModules[i]->getAzimuthRelativeEncoderCounts());
    }

    if (driverController->GetBackButtonPressed())
    {
        for (ValorSwerve *module : swerveModules)
        {
            module->storeAzimuthZeroReference();
        }
    }

    if (driverController->GetStartButtonPressed())
    {
        resetState();
    }

    odometry.Update(getHeading(),
                    swerveModules[0]->getState(),
                    swerveModules[1]->getState(),
                    swerveModules[2]->getState(),
                    swerveModules[3]->getState());
}

void Drivetrain::assignOutputs()
{
    // Get the x speed. We are inverting this because Xbox controllers return
    // negative values when we push forward.
    double xSpeed = std::abs(state.leftStickY) > DriveConstants::kDeadbandY ? std::fabs(state.leftStickY) * state.leftStickY : 0 ;

    // Get the y speed or sideways/strafe speed. We are inverting this because
    // we want a positive value when we pull to the left. Xbox controllers
    // return positive values when you pull to the right by default.
    double ySpeed = std::abs(state.leftStickX) > DriveConstants::kDeadbandX ? -fabs(state.leftStickX) * state.leftStickX : 0;

    // Get the rate of angular rotation. We are inverting this because we want a
    // positive value when we pull to the left (remember, CCW is positive in
    // mathematics). Xbox controllers return positive values when you pull to
    // the right by default.
    double rot = std::abs(state.rightStickX) > DriveConstants::kDeadbandX ? -fabs(state.rightStickX) * state.rightStickX : 0;

    units::meters_per_second_t xSpeedMPS = units::meters_per_second_t{xSpeed * SwerveConstants::DRIVE_MAX_SPEED_MPS};
    units::meters_per_second_t ySpeedMPS = units::meters_per_second_t{ySpeed * SwerveConstants::DRIVE_MAX_SPEED_MPS};
    units::radians_per_second_t rotRPS = units::radians_per_second_t{rot * SwerveConstants::ROTATION_MAX_SPEED_RPS};

    double heading = getHeading().Degrees().to<double>();
    if (state.xButtonPressed) {
        rotRPS = units::radians_per_second_t{angleWrap(90.0 - heading) * DriveConstants::TURN_KP};
    } else if (state.aButtonPressed) {
        rotRPS = units::radians_per_second_t{angleWrap(-180.0 - heading) * DriveConstants::TURN_KP};
    } else if (state.bButtonPressed) {
        rotRPS = units::radians_per_second_t{angleWrap(-90.0 - heading) * DriveConstants::TURN_KP};
    } else if (state.yButtonPressed) {
        rotRPS = units::radians_per_second_t{angleWrap(0.0 - heading) * DriveConstants::TURN_KP};
    }

    // Limelight Tracking
    int led_mode = limeTable->GetNumber("ledMode", LimelightConstants::LED_MODE_OFF);
    if (state.tracking) {
        if (led_mode != LimelightConstants::LED_MODE_ON) {
            limeTable->PutNumber("ledMode", LimelightConstants::LED_MODE_ON);
            limeTable->PutNumber("camMode", LimelightConstants::TRACK_MODE_ON);
        }
        rotRPS = units::radians_per_second_t{-limeTable->GetNumber("tx", 0.0) * DriveConstants::LIMELIGHT_KP};

    // Manual Control
    } else {
        if (led_mode != LimelightConstants::LED_MODE_OFF) {
            limeTable->PutNumber("ledMode", LimelightConstants::LED_MODE_OFF);
            limeTable->PutNumber("camMode", LimelightConstants::TRACK_MODE_OFF);
        }
    }

    drive(xSpeedMPS, ySpeedMPS, rotRPS, true);
}

void Drivetrain::resetState()
{
    state.tracking = false;

    resetGyro();
    resetOdometry(frc::Pose2d{0_m, 0_m, 0_rad});
    for (int i = 0; i < swerveModules.size(); i++) {
        swerveModules[i]->loadAndSetAzimuthZeroReference();
    }
}

frc::SwerveDriveKinematics<4> Drivetrain::getKinematics()
{
    return kinematics;
}

frc::Pose2d Drivetrain::getPose_m()
{
    return odometry.GetPose();
}

frc::Rotation2d Drivetrain::getHeading()
{
    frc::Rotation2d rot = frc::Rotation2d(units::radian_t(navX.GetFusedHeading() * MathConstants::toRadians));
    return hasGyroOffset ? rot.RotateBy(gyroOffset) : rot;
}

double Drivetrain::getGyroRate()
{
    return navX.GetRate();
}

frc::Rotation2d Drivetrain::getGyroOffset()
{
    return gyroOffset;
}

void Drivetrain::setGyroOffset(frc::Rotation2d offset)
{
    if (gyroOffset == offset)
        return;
    gyroOffset = offset;
    hasGyroOffset = true;
}

void Drivetrain::resetOdometry(frc::Pose2d pose)
{
    odometry.ResetPosition(pose, navX.GetRotation2d().RotateBy(gyroOffset));
}

void Drivetrain::resetDriveEncoders()
{
    for (int i = 0; i < 4; i++)
    {
        swerveModules[i]->resetDriveEncoder();
    }
}

void Drivetrain::resetGyro()
{
    //navX.ZeroYaw(); can't reset fused heading
    setGyroOffset(frc::Rotation2d(units::radian_t(-navX.GetFusedHeading() * MathConstants::toRadians)));
}

void Drivetrain::drive(units::meters_per_second_t vx_mps, units::meters_per_second_t vy_mps, units::radians_per_second_t omega_radps, bool isFOC)
{
    auto states = getModuleStates(vx_mps,
                                  vy_mps,
                                  omega_radps,
                                  isFOC);
    for (int i = 0; i < 4; i++)
    {
        swerveModules[i]->setDesiredState(states[i], true);
    }
}

void Drivetrain::move(double vx_mps, double vy_mps, double omega_radps, bool isFOC)
{
    auto states = getModuleStates(units::meters_per_second_t{vx_mps},
                                  units::meters_per_second_t{vy_mps},
                                  units::radians_per_second_t{omega_radps},
                                  isFOC);
    for (int i = 0; i < 4; i++)
    {
        swerveModules[i]->setDesiredState(states[i], false);
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
                                                                                           -getHeading())
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