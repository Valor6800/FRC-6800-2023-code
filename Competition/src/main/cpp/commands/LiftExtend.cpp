#include "commands/LiftExtend.h"

LiftExtend::LiftExtend(Lift *lift) : m_lift(lift) {}

void LiftExtend::Initialize()
{
    m_lift->state.liftstateMain = Lift::LiftMainState::LIFT_MAIN_FIRSTPOSITION;
}

bool LiftExtend::IsFinished()
{
    return m_lift->getExtensionEncoderValue() > (LiftConstants::MAIN_FIRST_POSITION - 2000);
}