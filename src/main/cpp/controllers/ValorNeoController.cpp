#include "controllers/ValorNeoController.h"

ValorNeoController::ValorNeoController(int canID,
                                       ValorNeutralMode _mode,
                                       bool _inverted,
                                       std::string canbus) :
    ValorController(new rev::CANSparkMax(canID, rev::CANSparkMax::MotorType::kBrushless), _inverted, _mode),
    pidController(motor->GetPIDController()),
    encoder(motor->GetEncoder()),
    extEncoder(motor->GetAbsoluteEncoder(rev::SparkMaxAbsoluteEncoder::Type::kDutyCycle)),
    currentPidSlot(0)
{
    init();
}

void ValorNeoController::init()
{
    motor->RestoreFactoryDefaults();
    motor->SetInverted(inverted);
    setNeutralMode(neutralMode);
    setRange(0,-1,1);
    ValorPIDF motionPIDF;
    setPIDF(motionPIDF, 0);
    reset();

    wpi::SendableRegistry::AddLW(this, "ValorNeoController", "ID " + std::to_string(motor->GetDeviceId()));
}

void ValorNeoController::reset()
{
    encoder.SetPosition(0);
}

void ValorNeoController::setupFollower(int canID, bool followerInverted)
{
    followerMotor = new rev::CANSparkMax(canID, rev::CANSparkMax::MotorType::kBrushless);
    followerMotor->Follow(*motor, followerInverted);
    setNeutralMode(ValorController::neutralMode);
}

void ValorNeoController::setForwardLimit(double forward)
{
    motor->EnableSoftLimit(rev::CANSparkMax::SoftLimitDirection::kForward, true);
    motor->SetSoftLimit(rev::CANSparkMax::SoftLimitDirection::kForward, forward);
}

void ValorNeoController::setReverseLimit(double reverse)
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

    pidController.SetSmartMotionMaxVelocity(pidf.velocity * 60.0 / conversion, slot);
    pidController.SetSmartMotionMaxAccel(pidf.acceleration * 60.0 / conversion, slot);
    pidController.SetSmartMotionAllowedClosedLoopError(pidf.error, slot);
}

/**
 * Set the conversion rate.
 * Converts between your desired units and rotations of the neo motor shaft (includes gear ratio)
 * @param conversion Conversion rate for position
 */
void ValorNeoController::setConversion(double _conversion)
{
    conversion = _conversion;
    encoder.SetPositionConversionFactor(conversion);
    // convert from minutes to seconds for velocity
    encoder.SetVelocityConversionFactor(_conversion / 60.0);
   
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
 * Get the PIDF profile number of the motor
*/
int ValorNeoController::getProfile()
{
    return currentPidSlot;
}

/**
 * Get the speed in units per second (specified by conversion)
 */
double ValorNeoController::getSpeed()
{
    return encoder.GetVelocity();
}

void ValorNeoController::setEncoderPosition(double position)
{
    encoder.SetPosition(position);
}

double ValorNeoController::getAbsEncoderPosition()
{
    return extEncoder.GetPosition();
}

/**
 * Set the position in units (specified by conversion). Example: inches
 */
void ValorNeoController::setPosition(double position)
{
    pidController.SetReference(position, rev::CANSparkMax::ControlType::kSmartMotion, currentPidSlot);
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
    pidController.SetReference(speed * 60.0, rev::CANSparkMax::ControlType::kVelocity, currentPidSlot);
}

void ValorNeoController::setPower(double power)
{
    motor->Set(power);
}

void ValorNeoController::setNeutralMode(ValorNeutralMode mode){  
    motor->SetIdleMode(mode == ValorNeutralMode::Brake ? rev::CANSparkMax::IdleMode::kBrake : rev::CANSparkMax::IdleMode::kCoast);
    neutralMode = mode;
}

void ValorNeoController::InitSendable(wpi::SendableBuilder& builder)
{
    builder.SetSmartDashboardType("Subsystem");
    builder.AddDoubleProperty(
        "Amps", 
        [this] { return getCurrent(); },
        nullptr);
    builder.AddDoubleProperty(
        "Position", 
        [this] { return getPosition(); },
        nullptr);
    builder.AddDoubleProperty(
        "Speed", 
        [this] { return getSpeed(); },
        nullptr);
    builder.AddBooleanProperty(
        "Inverted", 
        [this] { return inverted; },
        nullptr);
}