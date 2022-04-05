#include "subsystems/ValorSwerve.h"
#include "Constants.h"
#include <fstream>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <iostream>

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
    return frc::SwerveModuleState{getDriveSpeed_mps(), getAzimuthRotation2d()};
}

void ValorSwerve::setDesiredState(frc::SwerveModuleState desiredState, bool isDriveOpenLoop)
{

    // Deadband
    if (desiredState.speed < SwerveConstants::DRIVE_DEADBAND_MPS) {
        setDriveOpenLoop_mps(0);
        return;
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

double ValorSwerve::getAzimuthCurrent(){
    return azimuthFalcon->GetSupplyCurrent();
}

double ValorSwerve::getDriveCurrent(){
    return driveFalcon->GetSupplyCurrent();
}

void ValorSwerve::storeAzimuthZeroReference()
{
    int index = getWheelIndex();

    // Encoder ticks via the mag encoder
    int position = getMagEncoderCount();

    std::ofstream ofs;
    std::stringstream stream;
    stream << "/home/lvuser/SwerveModule.wheel.";
    stream << std::to_string(index);
    stream << ".txt";
    ofs.open(stream.str(), std::ofstream::out);
    ofs << std::to_string(position);
    ofs.close();
    //std::cout << "stored position in file" << std::endl;
}

void ValorSwerve::loadAndSetAzimuthZeroReference()
{
    int index = getWheelIndex();
    std::ifstream infile("/home/lvuser/SwerveModule.wheel." + std::to_string(index) + ".txt");
    if (!infile.good())
        return;

    std::string line;
    std::getline(infile, line);

    // Encoder position for the mag encoder read from storage
    int storedMagEncoderTicks = atoi(line.c_str());

    // Difference in mag encoder ticks to re-zero the module
    int delta = getMagEncoderCount() - storedMagEncoderTicks;

    // Mod division by 2048 (resolution of the mag encoder) to get remainder.
    // Then, convert mag encoder to azimuth ticks
    int azimuthSetpoint = convertMagEncoderToAzimuthEncoder(fmod(delta, SwerveConstants::MAG_COUNTS_PER_REV));
    // 1500 % 
    //issue: mag encoder tics don't keep increasing forever, they "wrap around" and reset back to 0
    //so, modding doesn't really help anything because the falcon encoder could have been spun around 10 times
    //line below might fix burn out issue
    //azimuthSetpoint = fmod(azimuthSetpoint, SwerveConstants::AZIMUTH_COUNTS_PER_REV / SwerveConstants::AZIMUTH_GEAR_RATIO);

    // Set the azimuth offset to the calculated setpoint (which will take over in teleop)
    azimuthFalcon->SetSelectedSensorPosition(azimuthSetpoint, 0, 10);
    //std::cout << "pulled pospition from file" << std::endl;
}

WPI_TalonFX* ValorSwerve::getAzimuthFalcon()
{
    return azimuthFalcon;
}

WPI_TalonFX* ValorSwerve::getDriveFalcon()
{
    return driveFalcon;
}

int ValorSwerve::convertMagEncoderToAzimuthEncoder(float magTicks)
{
    //GetDistance is 4096 ticks per rotation
    //Azimuth ticks per rotation is 2048
    //Therefore divide mag encoder by 2 to sync azimuth and mag encoder
    //The mag encoder is not geared. Therefore take the gear ratio into account to match the azimuth gearing
    return (magTicks * 0.5f / SwerveConstants::AZIMUTH_GEAR_RATIO);
}

int ValorSwerve::getAzimuthEncoderCount()
{
    return azimuthFalcon->GetSelectedSensorPosition();
}

int ValorSwerve::getMagEncoderCount()
{
    return magEncoder->GetDistance();
}

frc::Rotation2d ValorSwerve::getAzimuthRotation2d()
{
    double radians = getAzimuthEncoderCount() * MathConstants::ticksToRads;
    return frc::Rotation2d{units::radian_t{radians}};
}

// The angle coming in is an optimized angle. No further calcs should be done on 'angle'
void ValorSwerve::setAzimuthRotation2d(frc::Rotation2d angle)
{
    frc::Rotation2d currentAngle = getAzimuthRotation2d();
    frc::Rotation2d deltaAngle = angle - currentAngle;

    double countsBefore = getAzimuthEncoderCount();
    double revolutionsWheel = deltaAngle.Radians().to<double>() / (2.0 * M_PI);
    if (revolutionsWheel > 1)
    {
        revolutionsWheel = fmod(revolutionsWheel, 1.0);
        if (revolutionsWheel > .5)
        {
            revolutionsWheel -= 1;
        }
    }
    else if (revolutionsWheel < -1)
    {
        revolutionsWheel = fmod(revolutionsWheel, -1.0);
        if (revolutionsWheel < -.5)
        {
            revolutionsWheel += 1;
        }
    }
    double revolutionsFalon = revolutionsWheel / SwerveConstants::AZIMUTH_GEAR_RATIO;
    double countsFromAngle = revolutionsFalon * SwerveConstants::AZIMUTH_COUNTS_PER_REV;
    //double countsDelta = fmod(countsFromAngle - countsBefore, SwerveConstants::AZIMUTH_COUNTS_PER_REV / SwerveConstants::AZIMUTH_GEAR_RATIO);
    azimuthFalcon->Set(ControlMode::MotionMagic, countsBefore + countsFromAngle);
}

units::meters_per_second_t ValorSwerve::getDriveSpeed_mps()
{
    double encoderCountsPer100ms = driveFalcon->GetSelectedSensorVelocity();
    if (encoderCountsPer100ms < 0) 
    encoderCountsPer100ms*= 0.94;
    double motorRotationsPer100ms = encoderCountsPer100ms / SwerveConstants::DRIVE_COUNTS_PER_REV;
    double wheelRotationsPer100ms = motorRotationsPer100ms * SwerveConstants::DRIVE_GEAR_RATIO;
    double metersPer100ms = wheelRotationsPer100ms * SwerveConstants::WHEEL_CIRCUMFERENCE_M;
    return units::meters_per_second_t{metersPer100ms * K100MSPERSECOND};
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