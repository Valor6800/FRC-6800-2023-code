/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
#include "subsystems/Drivetrain.h"
#include <iostream>

#define MOTOR_FREE_SPEED 6380.0
#define WHEEL_DIAMETER_M 0.1016
#define DRIVE_GEAR_RATIO 12.8
#define AZIMUTH_GEAR_RATIO 5.14
#define AUTO_SPEED_MUL 0.75
#define ROT_SPEED_MUL 1
#define ROT_SPEED_SLOW_MUL 0.5

#define AZIMUTH_K_P 0.2
#define AZIMUTH_K_I 0.0
#define AZIMUTH_K_D 0.1
#define AZIMUTH_K_F 0.05

#define AZIMUTH_K_VEL 17000.0
#define AZIMUTH_K_ACC_MUL 20.0

#define DRIVETRAIN_CAN_BUS "baseCAN"

Drivetrain::Drivetrain(frc::TimedRobot *_robot) : ValorSubsystem(_robot, "Drivetrain"),
                        driveMaxSpeed(MOTOR_FREE_SPEED / 60.0 * DRIVE_GEAR_RATIO * WHEEL_DIAMETER_M * M_PI),
                        rotMaxSpeed(ROT_SPEED_MUL * 2 * M_PI),
                        autoMaxSpeed(driveMaxSpeed),
                        autoMaxAccel(autoMaxSpeed * AUTO_SPEED_MUL),
                        pigeon(CANIDs::PIGEON_CAN, DRIVETRAIN_CAN_BUS),
                        kinematics(motorLocations[0], motorLocations[1], motorLocations[2], motorLocations[3]),
                        odometry(kinematics, pigeon.GetRotation2d(), initPositions, frc::Pose2d{0_m, 0_m, 0_rad})
{
    frc2::CommandScheduler::GetInstance().RegisterSubsystem(this);
    init();
}

Drivetrain::~Drivetrain()
{
    for (int i = 0; i < 4; i++)
    {
        delete magEncoders[i];
        delete azimuthControllers[i];
        delete driveControllers[i];
        delete swerveModules[i];
    }
}

void Drivetrain::configSwerveModule(int i)
{
    ValorPIDF azimuthPID;
    azimuthPID.velocity = AZIMUTH_K_VEL;
    azimuthPID.acceleration = azimuthPID.velocity * AZIMUTH_K_ACC_MUL;
    azimuthPID.F = AZIMUTH_K_F;
    azimuthPID.P = AZIMUTH_K_P;
    azimuthPID.I = AZIMUTH_K_I;
    azimuthPID.D = AZIMUTH_K_D;

    azimuthControllers.push_back(new SwerveAzimuthMotor(CANIDs::AZIMUTH_CANS[i],
                                                      NeutralMode::Brake,
                                                      false,
                                                      DRIVETRAIN_CAN_BUS));
    azimuthControllers[i]->setPIDF(azimuthPID, 0);
    azimuthControllers[i]->setConversion(1 / AZIMUTH_GEAR_RATIO);

    driveControllers.push_back(new SwerveDriveMotor(CANIDs::DRIVE_CANS[i],
                                                    NeutralMode::Coast,
                                                    false,
                                                    DRIVETRAIN_CAN_BUS));
    driveControllers[i]->setConversion(1 / DRIVE_GEAR_RATIO * M_PI * WHEEL_DIAMETER_M);

    magEncoders.push_back(new frc::DutyCycleEncoder(DIOPorts::MAG_ENCODER_PORTS[i]));
    magEncoders[i]->SetDistancePerRotation(4096.0);

    swerveModules.push_back(new ValorSwerve<SwerveAzimuthMotor, SwerveDriveMotor>(azimuthControllers[i], driveControllers[i], magEncoders[i], motorLocations[i]));
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
    pigeon.Calibrate();    

    for (int i = 0; i < 4; i++)
    {
        configSwerveModule(i);
    }

    table->PutBoolean("Save Swerve Mag Encoder", false);
    state.saveToFileDebouncer = false;

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

    state.xSpeed = driverGamepad->leftStickY(2);
    state.ySpeed = driverGamepad->leftStickX(2);
    state.rot = driverGamepad->rightStickX(3);
    state.slowDown = driverGamepad->GetAButton();
    state.startButton = driverGamepad->GetStartButtonPressed();
}

void Drivetrain::analyzeDashboard()
{
    table->PutNumber("Robot X", getPose_m().X().to<double>());
    table->PutNumber("Robot Y", getPose_m().Y().to<double>());
    table->PutNumber("Robot Theta", getPose_m().Rotation().Degrees().to<double>());
    table->PutNumber("Pigeon Theta", getPigeon().Degrees().to<double>());

    // Only save to file once. Wait until switch is toggled to run again
    if (table->GetBoolean("Save Swerve Mag Encoder",false) && !state.saveToFileDebouncer) {
        for (ValorSwerve<SwerveAzimuthMotor, SwerveDriveMotor> *module : swerveModules)
        {
            module->storeAzimuthZeroReference();
        }
        state.saveToFileDebouncer = true;
    } else if (!table->GetBoolean("Save Swerve Mag Encoder",false)) {
        state.saveToFileDebouncer = false;
    }

    odometry.Update(getPigeon(),
                    {
                        swerveModules[0]->getModulePosition(),
                        swerveModules[1]->getModulePosition(),
                        swerveModules[2]->getModulePosition(),
                        swerveModules[3]->getModulePosition()
                    });
}

void Drivetrain::assignOutputs()
{    
    units::meters_per_second_t xSpeedMPS = units::meters_per_second_t{state.xSpeed * driveMaxSpeed};
    units::meters_per_second_t ySpeedMPS = units::meters_per_second_t{state.ySpeed * driveMaxSpeed};
    units::radians_per_second_t rotRPS = units::radians_per_second_t{state.rot * rotMaxSpeed};

    if (state.slowDown) {
        double magnitude = std::sqrt(std::pow(state.xSpeed, 2) + std::pow(state.ySpeed, 2));
        double x = state.xSpeed / magnitude;
        double y = state.ySpeed / magnitude;
        xSpeedMPS = units::meters_per_second_t{x};
        ySpeedMPS = units::meters_per_second_t{y};
        if(state.rot != 0){
            int sign = std::signbit(state.rot) == 0 ? 1 : -1;
            rotRPS = units::radians_per_second_t{state.rot * rotMaxSpeed * ROT_SPEED_SLOW_MUL};
        }
    }

    if (state.startButton) {
        pullSwerveModuleZeroReference();
    }
    drive(xSpeedMPS, ySpeedMPS, rotRPS, true);
}

void Drivetrain::pullSwerveModuleZeroReference(){
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
    odometry.ResetPosition(getPigeon(),
                            { 
                                swerveModules[0]->getModulePosition(),
                                swerveModules[1]->getModulePosition(),
                                swerveModules[2]->getModulePosition(),
                                swerveModules[3]->getModulePosition()
                            },
                            pose);
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
    kinematics.DesaturateWheelSpeeds(&states, units::meters_per_second_t{driveMaxSpeed});
    return states;
}

void Drivetrain::setModuleStates(wpi::array<frc::SwerveModuleState, 4> desiredStates)
{
    kinematics.DesaturateWheelSpeeds(&desiredStates, units::meters_per_second_t{driveMaxSpeed});
    for (int i = 0; i < 4; i++)
    {
        swerveModules[i]->setDesiredState(desiredStates[i], true);
    }
}