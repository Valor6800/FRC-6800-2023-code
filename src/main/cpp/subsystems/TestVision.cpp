#include "subsystems/TestVision.h"
#define LimelightHeight 1.0

TestVision::TestVision(frc::TimedRobot *_robot) : ValorSubsystem(_robot, "TestVision") {
    // frc2::CommandScheduler::GetInstance().RegisterSubsystem(this);
}
    
void TestVision::init() {
            
        vision.setGetter([this] () {
            return camera.GetLatestResult();
        });
}

void TestVision::analyzeDashboard() {

    
    photonTable->PutNumber("Fiducial ID", vision.getSensor().GetTargets()[0].GetFiducialId());
    photonTable->PutNumber("X value", vision.getSensor().GetTargets()[0].GetAlternateCameraToTarget().X().to<double>());
    photonTable->PutNumber("Y value", vision.getSensor().GetTargets()[0].GetAlternateCameraToTarget().Y().to<double>());
    photonTable->PutNumber("Z value", vision.getSensor().GetTargets()[0].GetAlternateCameraToTarget().Rotation().Z().to<double>());
    vision.calculate();
    photonTable->PutNumber("Return X Coordinate", vision.finalPose.X().to<double>());
    photonTable->PutNumber("Return Y Coordinate", vision.finalPose.Y().to<double>());
    photonTable->PutNumber("Return Z Coordinate", LimelightHeight);
}
