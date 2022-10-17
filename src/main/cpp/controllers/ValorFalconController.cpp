#include "controllers/ValorFalconController.h"

#define FALCON_TICKS_PER_REV 2048

ValorFalconController::ValorFalconController(int canID,
                                             NeutralMode _mode,
                                             bool _inverted,
                                             const std::__cxx11::string &canbus) :
    ValorController(_mode, _inverted),
    conversion(1)
{
    motor = new WPI_TalonFX{canID, canbus};
    init();
}

void ValorFalconController::init()
{
    motor->ConfigFactoryDefault();
    motor->SetInverted(inverted);
    motor->SetNeutralMode(mode);

    // @TODO should this be enabled for all falcons going forward?
    motor->EnableVoltageCompensation(true);
    motor->ConfigVoltageCompSaturation(10);
    motor->ConfigSupplyCurrentLimit(SupplyCurrentLimitConfiguration(true, 60, 80, .75)); //potentially could do 40 60

    motor->ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor, 0, 10);
    motor->ConfigAllowableClosedloopError(0, 0);
    motor->Config_IntegralZone(0, 0);
    
    setPIDF(0, motionPIDF);
    reset();
}

void ValorFalconController::reset()
{
    motor->SetSelectedSensorPosition(0);
}

void ValorFalconController::setupFollower(int canID)
{
    followerMotor = new WPI_TalonFX(canID);
    followerMotor->Follow(*motor);
    followerMotor->SetNeutralMode(mode);
}

void ValorFalconController::setLimits(int reverse, int forward)
{
    motor->ConfigForwardSoftLimitThreshold(reverse);
    motor->ConfigReverseSoftLimitThreshold(forward);
    motor->ConfigForwardSoftLimitEnable(true);
    motor->ConfigReverseSoftLimitEnable(true);
}

void ValorFalconController::setPIDF(int slot, PIDF pidf)
{
    ValorController::setPIDF(pidf);
    motor->Config_kP(slot, motionPIDF.P);
    motor->Config_kI(slot, motionPIDF.I);
    motor->Config_kD(slot, motionPIDF.D);
    motor->Config_kF(slot, motionPIDF.F);
    motor->ConfigMotionCruiseVelocity(motionPIDF.velocity);
    motor->ConfigMotionAcceleration(motionPIDF.acceleration);
}

void ValorFalconController::setConversion(double _conversion)
{
    conversion = _conversion;
}

/**
 * Get the position in units (specified by conversion)
 */
double ValorFalconController::getPosition()
{
    return motor->GetSelectedSensorPosition() * conversion / FALCON_TICKS_PER_REV;
}

/**
 * Get the speed in units per second (specified by conversion)
 */
double ValorFalconController::getSpeed()
{
    return motor->GetSelectedSensorVelocity() * 10 * conversion / FALCON_TICKS_PER_REV;
}

void ValorFalconController::setRange(int slot, double min, double max)
{
    
}

/**
 * Set the position in units (specified by conversion). Example: inches
 */
void ValorFalconController::setPosition(double position)
{
    motor->Set(ControlMode::MotionMagic, position / conversion * FALCON_TICKS_PER_REV);
}

/**
 * Set the speed in units per second (specified by conversion). Example: inches per second
 */
void ValorFalconController::setSpeed(double speed)
{
    motor->Set(ControlMode::Velocity, speed / 10 / conversion * FALCON_TICKS_PER_REV);
}

void ValorFalconController::setPower(double speed)
{
    motor->Set(speed);
}

void ValorFalconController::setProfile(int profile)
{
    motor->SelectProfileSlot(profile, 0);
}

void ValorFalconController::preventBackwards()
{
    motor->ConfigPeakOutputReverse(0);
}