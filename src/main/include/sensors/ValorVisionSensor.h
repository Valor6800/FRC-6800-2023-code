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
#include <frc/DriverStation.h>

#include <units/length.h>

class ValorVisionSensor : public ValorSensor<frc::Pose2d>
{
    public:
    ValorVisionSensor(frc::TimedRobot *_robot) : ValorSensor(_robot, "visionSensor"){}
    
    std::shared_ptr<nt::NetworkTable> visionTable;
    int pipe{0};
    
    void reset();
    void calculate();
    void InitSendable(wpi::SendableBuilder& builder) override;
    int GetTag();

    int tv;
    int tid;
    double tx, ty;
    double xDist;
    frc::Pose2d botpose;
    frc::Pose2d visionRobotPose;
    std::vector<double> robotPoseList;

    void translatePoseToCornerRed(frc::Pose2d tagPose);
    void translatePoseToCornerBlue(frc::Pose2d tagPose);
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
