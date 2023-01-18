#include "sensors/ValorVisionSensor.h"

#include <math.h>


#define ROBOTHEIGHT 20.0

void ValorVisionSensor::reset()
{
    frc::Transform2d pose;
}

void ValorVisionSensor::calculate() {
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

void ValorVisionSensor::aim() {
    //Get in range of target
    auto result = getSensor();

}

