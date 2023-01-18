#include "ValorSubsystem.h"
#include "sensors/ValorVisionSensor.h"

#include "frc/smartdashboard/Smartdashboard.h"

class TestVision : public ValorSubsystem {
    public:
    
    TestVision(frc::TimedRobot *_robot) : ValorSubsystem(_robot, "TestVision"), visionSensor(_robot){}
    
    void init();

    void assessInputs();
    void analyzeDashboard();
    void assignOutputs();

    void resetState();

    frc::Pose2d visionRobotPose;

    ValorVisionSensor visionSensor;
    std::shared_ptr<nt::NetworkTable> visionTable;
};