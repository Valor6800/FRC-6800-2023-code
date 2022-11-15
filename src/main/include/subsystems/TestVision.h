#include "ValorSubsystem.h"
#include "sensors/ValorVision.h"

#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableInstance.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include "networktables/NetworkTable.h"

class TestVision : public ValorSubsystem {
    public:

    TestVision(frc::TimedRobot *_robot) : ValorSubsystem(_robot, "TestVision"), vision(_robot) {}
    void init();

    void assessInputs();
    void analyzeDashboard();
    void assignOutputs();

    void resetState();
    ValorVision vision;
    photonlib::PhotonCamera camera{"valorVision"};
    std::shared_ptr<nt::NetworkTable> photonTable;
};