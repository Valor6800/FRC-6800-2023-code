#include "subsystems/TestVision.h"
#define LimelightHeight 1.0
    
void TestVision::init() {
            
        visionSensor.setGetter([this] () {
            return visionRobotPose;
        });
}


void TestVision::resetState() {
    
}

void TestVision::analyzeDashboard() {

    
    // photonTable->PutNumber("Fiducial ID", vision.getSensor().GetTargets()[0].GetFiducialId());
    // photonTable->PutNumber("X value", vision.getSensor().GetTargets()[0].GetAlternateCameraToTarget().X().to<double>());
    // photonTable->PutNumber("Y value", vision.getSensor().GetTargets()[0].GetAlternateCameraToTarget().Y().to<double>());
    // photonTable->PutNumber("Z value", vision.getSensor().GetTargets()[0].GetAlternateCameraToTarget().Rotation().Z().to<double>());
    // vision.calculate();
    // photonTable->PutNumber("Return X Coordinate", vision.finalPose.X().to<double>());
    // photonTable->PutNumber("Return Y Coordinate", vision.finalPose.Y().to<double>());
    // photonTable->PutNumber("Return Z Coordinate", LimelightHeight);
    
    visionRobotPose.X() = static_cast<units::meter_t>(visionTable->GetNumberArray("botpose",std::span<const double>())[0]);
    visionRobotPose.Y() = static_cast<units::meter_t>(visionTable->GetNumberArray("botpose",std::span<const double>())[1]);
    visionRobotPose.Rotation().Degrees() = static_cast<units::degree_t>(visionTable->GetNumberArray("botpose",std::span<const double>())[5]);
    visionSensor.robotPose = visionTable->GetNumberArray("botpose", std::span<const double>());
    visionSensor.tv = visionTable->GetNumber("tv", 0);
    visionSensor.tid = visionTable->GetNumber("tid",0);

    

}

void TestVision::assessInputs() {

}

void TestVision::assignOutputs() {
    //TODO: change motor values according to outputs
}
