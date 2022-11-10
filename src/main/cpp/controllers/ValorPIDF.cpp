#include "controllers/ValorPIDF.h"

ValorPIDF::ValorPIDF()//(int _motorId, int _slot)
{
    //wpi::SendableRegistry::AddLW(this, "ValorPIDF", "Motor ID " + std::to_string(motorId) + " Motor ID " + std::to_string(motorId));
}

void ValorPIDF::InitSendable(wpi::SendableBuilder& builder)
{
    builder.SetSmartDashboardType("Susbsystem");
    builder.AddDoubleProperty(
        "VALORPIDF works",
        [this] { return 2.0; },
        nullptr
    );
}

