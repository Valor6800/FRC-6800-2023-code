#include <frc/Filesystem.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <ctre/Phoenix.h>
#include <frc/DutyCycleEncoder.h>

class ValorSwerve
{
public:

    ValorSwerve(WPI_TalonFX* _azimuthFalcon,
                WPI_TalonFX* _driveFalcon,
                frc::DutyCycleEncoder* _magEncoder,
                frc::Translation2d _wheelLocation);

    /**
     * Get the maximum attainable speed of the drive
     * @return max speed in meters per second
     */
    double getMaxSpeed_mps();

    /**
     * Get the wheel location as Translation2d
     * @return wheel location in meters relative to center of robot
     */
    frc::Translation2d getWheelLocation_m();

    /**
     * Get the current state of the swerve module
     * @return current state of the swerve module
     */
    frc::SwerveModuleState getState();

    /**
     * Command the swerve module motors to the desired state
     * @param desiredState the desired swerve module speed and angle
     * @param isDriveOpen true if drive should set speed using closed-loop velocity control
     */
    void setDesiredState(frc::SwerveModuleState desiredState, bool isDriveOpenLoop);

    /**
     * Command the swerve module motors to the desired state using closed-loop drive speed control
     * @param desiredState the desired swerve module speed and angle
     */
    void setDesiredState(frc::SwerveModuleState desiredState)
    {
        setDesiredState(desiredState, false);
    }

    /**
     * Resets the drive encoders to currently read a position of 0
     */
    void resetDriveEncoder();

    /**
     * Save the current azimuth absolute encoder reference position in NetworkTables preferences.
     * Call this method following physical alignment of the module wheel in its zeroed position.
     * Used during module instantiation to initialize the relative encoder.
     */
    void storeAzimuthZeroReference();

    /**
     * Loads the current azimuth absolute encoder reference position and sets selected sensor encoder
     */
    void loadAndSetAzimuthZeroReference();

    frc::Rotation2d getAzimuthRotation2d();

    void setAzimuthRotation2d(frc::Rotation2d angle);

    WPI_TalonFX *getAzimuthFalcon();

    WPI_TalonFX *getDriveFalcon();

    int getAzimuthAbsoluteEncoderCounts();

    int getAzimuthRelativeEncoderCounts();

private:
    double getDriveSpeed_mps();

    void setDriveOpenLoop_mps(double mps);

    void setDriveClosedLoop_mps(double mps);

    int getWheelIndex();

    WPI_TalonFX* azimuthFalcon;
    WPI_TalonFX* driveFalcon;
    frc::DutyCycleEncoder* magEncoder;

    frc::Translation2d wheelLocation_m;
    frc::Rotation2d previousAngle;
};