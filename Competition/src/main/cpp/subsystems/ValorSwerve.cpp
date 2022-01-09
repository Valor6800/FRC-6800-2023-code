#include "subsystems/ValorSwerve.h"
#include "Constants.h"
#include <fstream>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#define K100MSPERSECOND 10

ValorSwerve::ValorSwerve(WPI_TalonFX* _azimuthFalcon,
                         WPI_TalonFX* _driveFalcon,
                         frc::DutyCycleEncoder* _magEncoder,
                         frc::Translation2d _wheelLocation) :
    azimuthFalcon(_azimuthFalcon),
    driveFalcon(_driveFalcon),
    magEncoder(_magEncoder),
    wheelLocation_m(_wheelLocation)
{

}

double ValorSwerve::getMaxSpeed_mps()
{
    return SwerveConstants::DRIVE_MAX_SPEED_MPS;
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
    if (desiredState.speed < SwerveConstants::DRIVE_DEADBAND_MPS) {
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
    std::stringstream stream;
    //wpi::SmallVectorImpl<char> result;
    //frc::filesystem::GetDeployDirectory(&result);
    //stream << result;
    stream << "/home/lvuser/SwerveModule.wheel.";
    stream << std::to_string(index);
    stream << ".txt";
    ofs.open(stream.str(), std::ofstream::out);
    ofs << std::to_string(position);
    ofs.close();
}

void ValorSwerve::loadAndSetAzimuthZeroReference()
{
    int index = getWheelIndex();
    std::ifstream infile("/home/lvuser/SwerveModule.wheel." + std::to_string(index) + ".txt");
    if (!infile.good())
        return;

    std::string line;
    std::getline(infile, line);
    int reference = atoi(line.c_str());

    int azimuthAbsoluteCounts = getAzimuthAbsoluteEncoderCounts();
    int azimuthSetpoint = fmod(azimuthAbsoluteCounts - reference, 2048.0 / SwerveConstants::AZIMUTH_GEAR_RATIO);
    azimuthFalcon->SetSelectedSensorPosition(-azimuthSetpoint, 0, 10);

    //azimuthFalcon->Set(ControlMode::MotionMagic, 0); //was initially azimuthSetpoint
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
    return magEncoder->GetDistance() * 0.5 / SwerveConstants::AZIMUTH_GEAR_RATIO;
}

int ValorSwerve::getAzimuthRelativeEncoderCounts()
{
    return azimuthFalcon->GetSelectedSensorPosition();
}

frc::Rotation2d ValorSwerve::getAzimuthRotation2d()
{
    double azimuthCounts = azimuthFalcon->GetSelectedSensorPosition();
    double radians = 2.0 * M_PI * azimuthCounts / SwerveConstants::AZIMUTH_COUNTS_PER_REV * SwerveConstants::AZIMUTH_GEAR_RATIO;
    return frc::Rotation2d{units::radian_t{radians}};
}

void ValorSwerve::setAzimuthRotation2d(frc::Rotation2d angle)
{
    double countsBefore = azimuthFalcon->GetSelectedSensorPosition();
    double countsFromAngle = angle.Radians().to<double>() / (2.0 * M_PI * SwerveConstants::AZIMUTH_GEAR_RATIO) * SwerveConstants::AZIMUTH_COUNTS_PER_REV;
    double countsDelta = fmod(countsFromAngle - countsBefore, SwerveConstants::AZIMUTH_COUNTS_PER_REV / SwerveConstants::AZIMUTH_GEAR_RATIO);
    azimuthFalcon->Set(ControlMode::MotionMagic, countsBefore + countsDelta);
}

double ValorSwerve::getDriveSpeed_mps()
{
    double encoderCountsPer100ms = driveFalcon->GetSelectedSensorVelocity();
    double motorRotationsPer100ms = encoderCountsPer100ms / SwerveConstants::DRIVE_COUNTS_PER_REV;
    double wheelRotationsPer100ms = motorRotationsPer100ms * SwerveConstants::DRIVE_GEAR_RATIO;
    double metersPer100ms = wheelRotationsPer100ms * SwerveConstants::WHEEL_CIRCUMFERENCE_M;
    return metersPer100ms * K100MSPERSECOND;
}

void ValorSwerve::setDriveOpenLoop_mps(double mps)
{
    driveFalcon->Set(ControlMode::PercentOutput, mps / SwerveConstants::DRIVE_MAX_SPEED_MPS);
}

void ValorSwerve::setDriveClosedLoop_mps(double mps)
{
    double wheelRotationsPerSecond = mps / SwerveConstants::WHEEL_CIRCUMFERENCE_M;
    double motorRotationsPerSecond = wheelRotationsPerSecond / SwerveConstants::DRIVE_GEAR_RATIO;
    double encoderCountsPerSecond = motorRotationsPerSecond * SwerveConstants::DRIVE_COUNTS_PER_REV;
    driveFalcon->Set(ControlMode::Velocity, encoderCountsPerSecond / K100MSPERSECOND);
}

int ValorSwerve::getWheelIndex()
{
    if (wheelLocation_m.X() > units::meter_t{0} && wheelLocation_m.Y() > units::meter_t{0}) return 0;
    if (wheelLocation_m.X() > units::meter_t{0} && wheelLocation_m.Y() < units::meter_t{0}) return 1;
    if (wheelLocation_m.X() < units::meter_t{0} && wheelLocation_m.Y() > units::meter_t{0}) return 2;
    return 3;
}