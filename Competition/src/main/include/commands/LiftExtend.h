#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/Lift.h"

class LiftExtend : public frc2::CommandHelper<frc2::CommandBase, LiftExtend> {
public:
    explicit LiftExtend(Lift* lift);

    bool IsFinished() override;
    void Initialize() override;

private:
    Lift* m_lift;
};