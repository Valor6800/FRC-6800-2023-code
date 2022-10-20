#include "controllers/ValorNeoController.h"

ValorNeoController::ValorNeoController(int canID,
                                       rev::CANSparkMax::IdleMode _mode,
                                       bool _inverted,
                                       std::string canbus) :
    ValorController(new rev::CANSparkMax(canID, rev::CANSparkMax::MotorType::kBrushless)),
    pidController(motor->GetPIDController()),
    encoder(motor->GetEncoder()),
    mode(_mode),
    inverted(_inverted)
{
    // motor = new rev::CANSparkMax{};
    currentPidSlot = 0;
    init();
}

void ValorNeoController::init()
{
    motor->RestoreFactoryDefaults();
    motor->SetInverted(inverted);
    motor->SetIdleMode(mode);
    setRange(0,-1,1);
    ValorPIDF motionPIDF;
    setPIDF(motionPIDF, 0);
    reset();
}

void ValorNeoController::reset()
{
    encoder.SetPosition(0);
}

void ValorNeoController::setupFollower(int canID)
{
    followerMotor = new rev::CANSparkMax(canID, rev::CANSparkMax::MotorType::kBrushless);
    followerMotor->Follow(*motor);
    followerMotor->SetIdleMode(mode);
}

void ValorNeoController::setLimits(int reverse, int forward)
{
    motor->EnableSoftLimit(rev::CANSparkMax::SoftLimitDirection::kForward, true);
    motor->SetSoftLimit(rev::CANSparkMax::SoftLimitDirection::kForward, forward);
    motor->EnableSoftLimit(rev::CANSparkMax::SoftLimitDirection::kReverse, true);
    motor->SetSoftLimit(rev::CANSparkMax::SoftLimitDirection::kReverse, reverse);
}

void ValorNeoController::setForwardLimit(int forward)
{
    motor->EnableSoftLimit(rev::CANSparkMax::SoftLimitDirection::kForward, true);
    motor->SetSoftLimit(rev::CANSparkMax::SoftLimitDirection::kForward, forward);
}

void ValorNeoController::setReverseLimit(int reverse)
{
    motor->EnableSoftLimit(rev::CANSparkMax::SoftLimitDirection::kReverse, true);
    motor->SetSoftLimit(rev::CANSparkMax::SoftLimitDirection::kReverse, reverse);
}

void ValorNeoController::setPIDF(ValorPIDF pidf, int slot)
{
    pidController.SetP(pidf.P, slot);
    pidController.SetI(pidf.I, slot);
    pidController.SetD(pidf.D, slot);
    pidController.SetFF(pidf.F, slot);
    pidController.SetIZone(0, slot);

    pidController.SetSmartMotionMaxVelocity(pidf.velocity, slot);
    pidController.SetSmartMotionMaxAccel(pidf.acceleration, slot);
    pidController.SetSmartMotionAllowedClosedLoopError(pidf.error, slot);
}

/**
 * Set the conversion rate.
 * Converts between your desired units and rotations of the neo motor shaft (includes gear ratio)
 * @param conversion Conversion rate for position
 */
void ValorNeoController::setConversion(double conversion)
{
    encoder.SetPositionConversionFactor(conversion);
    // convert from minutes to seconds for velocity
    encoder.SetVelocityConversionFactor(conversion / 60.0);
}

void ValorNeoController::setRange(int slot, double min, double max)
{
    pidController.SetOutputRange(min, max, slot);
}

double ValorNeoController::getCurrent()
{
    return motor->GetOutputCurrent();
}

/**
 * Get the position in units (specified by conversion)
 */
double ValorNeoController::getPosition()
{
    return encoder.GetPosition();
}

/**
 * Get the speed in units per second (specified by conversion)
 */
double ValorNeoController::getSpeed()
{
    return encoder.GetVelocity();
}

/**
 * Set the position in units (specified by conversion). Example: inches
 */
void ValorNeoController::setPosition(double position)
{
    pidController.SetReference(position, rev::CANSparkMax::ControlType::kSmartMotion);
}

void ValorNeoController::setProfile(int profile)
{
    currentPidSlot = profile;
}

/**
 * Set the speed in units per second (specified by conversion). Example: inches per second
 */
void ValorNeoController::setSpeed(double speed)
{
    pidController.SetReference(speed, rev::ControlType::kSmartVelocity, currentPidSlot);
}

void ValorNeoController::setPower(double speed)
{
    motor->Set(speed);
}