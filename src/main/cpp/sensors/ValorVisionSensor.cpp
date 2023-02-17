#include "sensors/ValorVisionSensor.h"

#include <math.h>


#define ROBOTHEIGHT 20.0

void ValorVisionSensor::reset()
{
    setGetter([this] () {
            return visionRobotPose;
        });
    visionTable->PutNumber("pipeline", 0);
}

void ValorVisionSensor::calculate() {
    if (visionTable->GetNumber("getpipe", 0) == 0){
        tx = 0;
        tv = visionTable->GetNumber("tv", 0);
        if (tv == 1){
            TranslatePose();

            tid = visionTable->GetNumber("tid",0);

            if (robotPoseList.size() >= 6){
                visionRobotPose = frc::Pose2d(
                    static_cast<units::meter_t>(robotPoseList[0]),//x
                    static_cast<units::meter_t>(robotPoseList[1]),//y
                    static_cast<units::degree_t>(robotPoseList[5])//angle
                );
            }
        } else {
            robotPoseList = std::vector<double>();
        }
    } else if (visionTable->GetNumber("getpipe", 0) == 1) {
        tv = visionTable->GetNumber("tv", 0);
        if (tv == 1){
            tx = visionTable -> GetNumber("tx", 0);
        } else {
            tx = 0;
        }
    }

}

void ValorVisionSensor::InitSendable(wpi::SendableBuilder& builder) {
    builder.SetSmartDashboardType("VisionSensor");
    builder.AddIntegerProperty(
            "tag", 
            [this] { return GetTag(); },
            nullptr
    );

}

int ValorVisionSensor::GetTag(){
    visionTable -> GetNumber("tid", 0);
}

void ValorVisionSensor::TranslatePose(){
    if (frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kBlue){
        robotPoseList = visionTable->GetNumberArray("botpose_wpiblue", std::span<const double>());
    } else{
        robotPoseList = visionTable->GetNumberArray("botpose_wpired", std::span<const double>());
    }
}

