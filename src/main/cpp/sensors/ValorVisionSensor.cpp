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
                    static_cast<units::meter_t>(robotPoseList[0]),
                    static_cast<units::meter_t>(robotPoseList[1]),
                    static_cast<units::degree_t>(robotPoseList[5])
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

    //-> Chin Math? <-
    // double x = 5;
    // double y = 5;
    // double z = 1;

    // auto returnPose = getSensor()->GetNumber("tv", 0);
    
    // double angle;
    // double returnx = (double) returnPose[0].GetAlternateCameraToTarget().X();
    // double returny = (double) returnPose[0].GetAlternateCameraToTarget().Y();
    // double anglez = (double) ((units::degree_t) returnPose[0].GetAlternateCameraToTarget().Rotation().Z());

    // if (anglez <= 0){
    //     angle = 180 + anglez;
    // }else{
    //     angle = 180 - anglez; 
    // }
    
    // angle = angle * M_PI / 180;
   
    // double finalx,finaly,finalz;
    // double magnitude;
    // double value;

    // magnitude = sqrt(pow(returnx,2)+pow(returny,2));
    // if (angle < atan(returnx/returny))
    // {
    //     if (anglez < 0){
    //         finalx = x - magnitude * cos(atan(returnx/returny) - angle);
    //     }else{
    //         finalx = x + magnitude * cos(atan(returnx/returny) - angle);
    //     }
    //     finaly = y - magnitude * sin(atan(returnx/returny) - angle);
    // }else{
    //     if (anglez < 0){
    //         finalx = x - magnitude * cos(angle - atan(returnx/returny));
    //     }else{
    //         finalx = x + magnitude * cos(angle - atan(returnx/returny));
    //     }
       
    //     finaly = y - magnitude * sin(angle - atan(returnx/returny));
    // }

    // finalPose = frc::Translation2d((units::meter_t) finalx, (units::meter_t) finaly);
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

