#include "subsystems/ValorSwerve.h"
#include "Constants.h"
#include <fstream>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#define K100MSPERSECOND 10

double ValorSwerve::getMaxSpeed_mps()
{
    return FalconSwerveConstants::DRIVE_MAX_SPEED_MPS;
}

frc::Translation2d ValorSwerve::getWheelLocation_m()
{
    return wheelLocation_m;
}

frc::SwerveModuleState ValorSwerve::getState()
{
    double speed_mps = getDriveSpeed_mps();
    frc::Rotation2d angle = getAzimuthRotation2d();
    return frc::SwerveModuleState{units::meters_per_second_t(speed_mps), angle};
}

void ValorSwerve::setDesiredState(frc::SwerveModuleState desiredState, bool isDriveOpenLoop)
{

    // Deadband
    if (desiredState.speed < FalconSwerveConstants::DRIVE_DEADBAND_MPS) {
        desiredState = frc::SwerveModuleState{units::meters_per_second_t{0.0}, previousAngle};
    }
    
    // Cache previous angle
    previousAngle = desiredState.angle;

    // Get current angle, optimize drive state
    frc::Rotation2d currentAngle = getAzimuthRotation2d();
    frc::SwerveModuleState optimizedState = frc::SwerveModuleState::Optimize(desiredState, currentAngle);

    // Output optimized rotation and speed
    setAzimuthRotation2d(optimizedState.angle);
    if (isDriveOpenLoop)
        setDriveOpenLoop_mps(optimizedState.speed.to<double>());
    else
        setDriveClosedLoop_mps(optimizedState.speed.to<double>());
}

void ValorSwerve::resetDriveEncoder()
{
    driveFalcon->SetSelectedSensorPosition(0);
}

void ValorSwerve::storeAzimuthZeroReference()
{
    int index = getWheelIndex();
    int position = getAzimuthAbsoluteEncoderCounts();
    std::ofstream ofs;
    ofs.open("SwerveModule.wheel." + std::to_string(index) + ".txt", std::ofstream::out);
    ofs << std::to_string(position);
    ofs.close();
}

void ValorSwerve::loadAndSetAzimuthZeroReference()
{
    int index = getWheelIndex();
    std::ifstream infile("SwerveModule.wheel." + std::to_string(index) + ".txt");
    if (!infile.good())
        return;

    std::string line;
    std::getline(infile, line);
    int reference = atoi(line.c_str());

    int azimuthAbsoluteCounts = getAzimuthAbsoluteEncoderCounts();
    int azimuthSetpoint = azimuthAbsoluteCounts - reference;
    azimuthFalcon->SetSelectedSensorPosition(azimuthSetpoint, 0, 10);

    azimuthFalcon->Set(ControlMode::MotionMagic, azimuthSetpoint);
}

WPI_TalonFX* ValorSwerve::getAzimuthFalcon()
{
    return azimuthFalcon;
}

WPI_TalonFX* ValorSwerve::getDriveFalcon()
{
    return driveFalcon;
}

int ValorSwerve::getAzimuthAbsoluteEncoderCounts()
{
    return 0;
    //@TODO implement this
}

frc::Rotation2d ValorSwerve::getAzimuthRotation2d()
{
    double azimuthCounts = azimuthFalcon->GetSelectedSensorPosition();
    double radians = 2.0 * M_PI * azimuthCounts / FalconSwerveConstants::AZIMUTH_COUNTS_PER_REV;
    return frc::Rotation2d{units::radian_t{radians}};
}

void ValorSwerve::setAzimuthRotation2d(frc::Rotation2d angle)
{
    double countsBefore = azimuthFalcon->GetSelectedSensorPosition();
    double countsFromAngle = angle.Radians().to<double>() / (2.0 * M_PI) * FalconSwerveConstants::AZIMUTH_COUNTS_PER_REV;
    double countsDelta = fmod(countsFromAngle - countsBefore, FalconSwerveConstants::AZIMUTH_COUNTS_PER_REV);
    azimuthFalcon->Set(ControlMode::MotionMagic, countsBefore + countsDelta);
}

double ValorSwerve::getDriveSpeed_mps()
{
    double encoderCountsPer100ms = driveFalcon->GetSelectedSensorVelocity();
    double motorRotationsPer100ms = encoderCountsPer100ms / FalconSwerveConstants::DRIVE_COUNTS_PER_REV;
    double wheelRotationsPer100ms = motorRotationsPer100ms * FalconSwerveConstants::DRIVE_GEAR_RATIO;
    double metersPer100ms = wheelRotationsPer100ms * FalconSwerveConstants::WHEEL_CIRCUMFERENCE_M;
    return metersPer100ms * K100MSPERSECOND;
}

void ValorSwerve::setDriveOpenLoop_mps(double mps)
{
    driveFalcon->Set(ControlMode::PercentOutput, mps / FalconSwerveConstants::DRIVE_MAX_SPEED_MPS);
}

void ValorSwerve::setDriveClosedLoop_mps(double mps)
{
    double wheelRotationsPerSecond = mps / FalconSwerveConstants::WHEEL_CIRCUMFERENCE_M;
    double motorRotationsPerSecond = wheelRotationsPerSecond / FalconSwerveConstants::DRIVE_GEAR_RATIO;
    double encoderCountsPerSecond = motorRotationsPerSecond * FalconSwerveConstants::DRIVE_COUNTS_PER_REV;
    driveFalcon->Set(ControlMode::Velocity, encoderCountsPerSecond / K100MSPERSECOND);
}

int ValorSwerve::getWheelIndex()
{
    if (wheelLocation_m.X() > units::meter_t{0} && wheelLocation_m.Y() > units::meter_t{0}) return 0;
    if (wheelLocation_m.X() > units::meter_t{0} && wheelLocation_m.Y() < units::meter_t{0}) return 1;
    if (wheelLocation_m.X() < units::meter_t{0} && wheelLocation_m.Y() > units::meter_t{0}) return 2;
    return 3;
}