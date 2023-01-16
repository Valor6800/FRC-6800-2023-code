#pragma once

#include "sensors/ValorSensor.h"

#include <frc/controller/PIDController.h>
#include <controllers/ValorController.h>
#include <units/base.h>
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"
#include "networktables/NetworkTableEntry.h"
#include "networktables/NetworkTableValue.h"
#include <frc/geometry/Translation2d.h>
#include <frc/geometry/Pose2d.h>
#include <map>

#include <units/length.h>

class ValorVision : public ValorSensor<frc::Pose2d>
{
    public:
    ValorVision(frc::TimedRobot *_robot) : ValorSensor(_robot){

    }
    void reset();
    void calculate();
    void aim();

    const units::radian_t CAMERA_PITCH = 0_deg;
    const units::centimeter_t GOAL_RANGE_CENTIMETERS = 161_cm;

    const double P_GAIN = 0.1;
    const double D_GAIN = 0.0;

    int tv;
    int tid;
    double tx, ty;
    std::vector<double> robotPose;

    std::map<int, frc::Pose2d> tags = {
        {1, frc::Pose2d{15.513558_m, 1.071626_m, 0_deg}},
        {2, frc::Pose2d{15.513558_m, 2.0748026_m, 0_deg}},
        {3, frc::Pose2d{15.513558_m, 4.424426_m, 0_deg}},
        {4, frc::Pose2d{16.178784_m, 6.749796_m, 0_deg}},
        {5, frc::Pose2d{0.36195_m, 6.749796_m, 0_deg}},
        {6, frc::Pose2d{1.02743_m, 4.424426_m, 0_deg}},
        {7, frc::Pose2d{1.02743_m, 2.748026_m, 0_deg}},
        {8, frc::Pose2d{1.02743_m, 1.071626_m, 0_deg}},
    }; //When selecting an index subtract 1 from the tag id.

    
    

    private:
    // photonlib::PhotonCamera camera{"valorVision"};
    
};
