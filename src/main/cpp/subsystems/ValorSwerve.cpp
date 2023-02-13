#include "subsystems/ValorSwerve.h"
#include "Constants.h"
#include <fstream>
#include <stdio.h>
#include <stdlib.h>
#include <cmath>
#include <iostream>
#include <string>

#define DRIVE_DEADBAND 0.05f
#define MAG_ENCODER_TICKS_PER_REV 4096.0f

// Explicit template instantiation
// This is needed for linking
template class ValorSwerve<ValorFalconController, ValorFalconController>;
template class ValorSwerve<ValorNeoController, ValorNeoController>;
template class ValorSwerve<ValorFalconController, ValorNeoController>;
template class ValorSwerve<ValorNeoController, ValorFalconController>;

template<class AzimuthMotor, class DriveMotor>
ValorSwerve<AzimuthMotor, DriveMotor>::ValorSwerve(AzimuthMotor* _azimuthMotor,
                                                    DriveMotor* _driveMotor,
                                                    frc::Translation2d _wheelLocation) :
    azimuthMotor(_azimuthMotor),
    driveMotor(_driveMotor)
{
    if (_wheelLocation.X() > units::meter_t{0} && _wheelLocation.Y() > units::meter_t{0}) wheelIdx = 0;
    else if (_wheelLocation.X() > units::meter_t{0} && _wheelLocation.Y() < units::meter_t{0}) wheelIdx = 1;
    else if (_wheelLocation.X() < units::meter_t{0} && _wheelLocation.Y() > units::meter_t{0}) wheelIdx = 3;
    else wheelIdx = 2;

    wpi::SendableRegistry::AddLW(this, "Swerve", "Module " + std::to_string(wheelIdx));
}
    
template<class AzimuthMotor, class DriveMotor>
frc::SwerveModulePosition ValorSwerve<AzimuthMotor, DriveMotor>::getModulePosition()
{
    return { units::meter_t{ driveMotor->getPosition() },
             getAzimuthPosition()
    };
}

template<class AzimuthMotor, class DriveMotor>
frc::SwerveModuleState ValorSwerve<AzimuthMotor, DriveMotor>::getState()
{
    return frc::SwerveModuleState{units::velocity::meters_per_second_t{driveMotor->getSpeed()}, getAzimuthPosition()};
}

template<class AzimuthMotor, class DriveMotor>
void ValorSwerve<AzimuthMotor, DriveMotor>::setDesiredState(frc::SwerveModuleState desiredState, bool isDriveOpenLoop)
{

    // Deadband
    if (desiredState.speed < units::velocity::meters_per_second_t{DRIVE_DEADBAND}) {
        setDriveOpenLoop(0);
        return;
    }

    // Get current angle, optimize drive state
    frc::Rotation2d currentAngle = getAzimuthPosition();
    frc::SwerveModuleState optimizedState = frc::SwerveModuleState::Optimize(desiredState, currentAngle);

    // Output optimized rotation and speed
    setAzimuthPosition(optimizedState.angle);
    if (isDriveOpenLoop)
        setDriveOpenLoop(optimizedState.speed.to<double>());
    else
        setDriveClosedLoop(optimizedState.speed.to<double>());
}

template<class AzimuthMotor, class DriveMotor>
void ValorSwerve<AzimuthMotor, DriveMotor>::resetDriveEncoder()
{
    driveMotor->reset();
}

template<class AzimuthMotor, class DriveMotor>
void ValorSwerve<AzimuthMotor, DriveMotor>::storeAzimuthZeroReference()
{
    // Encoder position in rotations via the mag encoder
    double position = getMagEncoderCount();

    std::ofstream ofs;
    std::stringstream stream;
    stream << "/home/lvuser/SwerveModule.wheel.";
    stream << std::to_string(wheelIdx);
    stream << ".txt";
    ofs.open(stream.str(), std::ofstream::out);
    ofs << std::to_string(position);
    ofs.close();
}

template<class AzimuthMotor, class DriveMotor>
bool ValorSwerve<AzimuthMotor, DriveMotor>::loadAndSetAzimuthZeroReference()
{
    // Read the encoder position. If the encoder position isn't returned, set the position to what the wheels
    //   are currently. The pit crew sets the wheels straight in pre-match setup. They should be close enough
    //   if the mag encoders aren't working.
    //   Protects against issues as seen in: https://www.youtube.com/watch?v=MGxpWNcv-VM
    double currPos = getMagEncoderCount();
    if (currPos == 0) {
        return false;
    }

    std::ifstream infile("/home/lvuser/SwerveModule.wheel." + std::to_string(wheelIdx) + ".txt");
    if (!infile.good()) {
        return false;
    }

    // Encoder position for the mag encoder read from storage
    std::string line;
    std::getline(infile, line);
    double storedPos = atof(line.c_str());
    infile.close();

    // Get the remainder of the delta so the encoder can wrap
    azimuthMotor->setEncoderPosition(currPos - storedPos);
    return true;
}

template<class AzimuthMotor, class DriveMotor>
double ValorSwerve<AzimuthMotor, DriveMotor>::getMagEncoderCount()
{
    return azimuthMotor->getAbsEncoderPosition();
}

template<class AzimuthMotor, class DriveMotor>
frc::Rotation2d ValorSwerve<AzimuthMotor, DriveMotor>::getAzimuthPosition()
{
    double radians = azimuthMotor->getPosition() * (2.0 * M_PI);
    return frc::Rotation2d{units::radian_t{radians}};
}

// The angle coming in is an optimized angle. No further calcs should be done on 'angle'
template<class AzimuthMotor, class DriveMotor>
void ValorSwerve<AzimuthMotor, DriveMotor>::setAzimuthPosition(frc::Rotation2d desiredAngle)
{
    frc::Rotation2d currentAngle = getAzimuthPosition();
    double currentRotations = currentAngle.Radians().to<double>() / (2.0 * M_PI);

    frc::Rotation2d deltaAngle = desiredAngle - currentAngle;
    double deltaRotations = deltaAngle.Radians().to<double>() / (2.0 * M_PI);
    if (deltaRotations > 1)
    {
        deltaRotations = fmod(deltaRotations, 1.0);
        if (deltaRotations > .5)
        {
            deltaRotations -= 1;
        }
    }
    else if (deltaRotations < -1)
    {
        deltaRotations = fmod(deltaRotations, -1.0);
        if (deltaRotations < -.5)
        {
            deltaRotations += 1;
        }
    }
    azimuthMotor->setPosition(currentRotations + deltaRotations);
}

template<class AzimuthMotor, class DriveMotor>
void ValorSwerve<AzimuthMotor, DriveMotor>::setDriveOpenLoop(double mps)
{
    driveMotor->setPower(mps / maxSpeed);
}

template<class AzimuthMotor, class DriveMotor>
void ValorSwerve<AzimuthMotor, DriveMotor>::setDriveClosedLoop(double mps)
{
    driveMotor->setSpeed(mps);
}

template<class AzimuthMotor, class DriveMotor>
void ValorSwerve<AzimuthMotor, DriveMotor>::InitSendable(wpi::SendableBuilder& builder)
{
    builder.SetSmartDashboardType("Subsystem");
    builder.AddDoubleProperty(
        "magEncoderRotatons",
        [this] { return getMagEncoderCount(); },
        nullptr
    );
    builder.AddDoubleProperty
    (
        "state: angle",
        [this] { return getState().angle.Degrees().template to<double>(); },
        nullptr
    );
    builder.AddDoubleProperty
    (
        "state: speed",
        [this] { return getState().speed.template to<double>(); },
        nullptr
    );
    builder.AddDoubleProperty
    (
        "position: angle",
        [this] { return getModulePosition().angle.Degrees().template to<double>(); },
        nullptr
    );
    builder.AddDoubleProperty
    (
        "position: distance",
        [this] { return getModulePosition().distance.template to<double>(); },
        nullptr
    );
}
