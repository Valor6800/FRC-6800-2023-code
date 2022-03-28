#include "commands/LiftRotateOut.h"

LiftRotateOut::LiftRotateOut(Lift *lift) : m_lift(lift) {}

void LiftRotateOut::Initialize()
{
    m_lift->state.liftstateRotate = Lift::LiftRotateState::LIFT_ROTATE_TOPOSITION;
}

bool LiftRotateOut::IsFinished()
{
    return m_lift->getRotationEncoderValue() > (LiftConstants::ROTATE_FIRST_POSITION - 5);
}