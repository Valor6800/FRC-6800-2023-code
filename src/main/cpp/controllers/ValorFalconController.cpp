#include "controllers/ValorFalconController.h"

#define FALCON_TICKS_PER_REV 2048

ValorFalconController::ValorFalconController(int canID,
                                             NeutralMode mode,
                                             bool inverted,
                                             std::string canbus) :
    ValorController(canID, mode, inverted, canbus),
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
    
    ValorPIDF motionPIDF;
    setPIDF(motionPIDF, 0);
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
    //@TODO this is wrong! Needs to be converted from units to raw units
    motor->ConfigForwardSoftLimitThreshold(reverse);
    motor->ConfigReverseSoftLimitThreshold(forward);
    motor->ConfigForwardSoftLimitEnable(true);
    motor->ConfigReverseSoftLimitEnable(true);
}

void ValorFalconController::setPIDF(ValorPIDF pidf, int slot)
{
    motor->Config_kP(slot, pidf.P);
    motor->Config_kI(slot, pidf.I);
    motor->Config_kD(slot, pidf.D);
    motor->Config_kF(slot, pidf.F);
    motor->ConfigMotionCruiseVelocity(pidf.velocity);
    motor->ConfigMotionAcceleration(pidf.acceleration);
}

void ValorFalconController::setConversion(double _conversion)
{
    conversion = _conversion;
}

double ValorFalconController::getCurrent()
{
    return motor->GetOutputCurrent();
}

/**
 * Get the position in units (specified by conversion)
 */
double ValorFalconController::getPosition()
{
    return motor->GetSelectedSensorPosition() * conversion / FALCON_TICKS_PER_REV;
}

double ValorFalconController::getSpeed()
{
    return motor->GetSelectedSensorVelocity() * 10 * conversion / FALCON_TICKS_PER_REV;
}

void ValorFalconController::setRange(int slot, double min, double max)
{
    
}

void ValorFalconController::setPosition(double position)
{
    motor->Set(ControlMode::MotionMagic, position / conversion * FALCON_TICKS_PER_REV);
}

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