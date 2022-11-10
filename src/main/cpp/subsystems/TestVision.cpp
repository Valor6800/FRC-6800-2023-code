#include "subsystems/TestVision.h"

TestVision::TestVision(frc::TimedRobot *_robot) : ValorSubsystem(_robot, "TestVision") {
    // frc2::CommandScheduler::GetInstance().RegisterSubsystem(this);
}
    
void TestVision::init() {
            
        vision.setGetter([this] () {
            photonlib::PhotonPipelineResult result = camera.GetLatestResult();

            if (result.HasTargets()) {
                return result.GetTargets();
            }
            return wpi::span<const photonlib::PhotonTrackedTarget>();
        });
}

    // TestVision::init() {

    // }
