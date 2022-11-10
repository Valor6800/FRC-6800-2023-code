#include "ValorSubsystem.h"
#include "sensors/ValorVision.h"

class TestVision : public ValorSubsystem {

    TestVision(frc::TimedRobot *_robot) : ValorSubsystem(_robot, "TestVision"){}
    void init();
    ValorVision vision;
    photonlib::PhotonCamera camera{"valorVision"};
    
};