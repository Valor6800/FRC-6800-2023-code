#include "subsystems/TestVision.h"
#define LimelightHeight 1.0
    
void TestVision::init() {
        visionTable = nt::NetworkTableInstance::GetDefault().GetTable("limelight");

        visionSensor.setGetter([this] () {
            return visionRobotPose;
        });
}


void TestVision::resetState() {
    
}

void TestVision::analyzeDashboard() {

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
