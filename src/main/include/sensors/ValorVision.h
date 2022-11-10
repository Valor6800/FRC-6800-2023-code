#pragma once

#include "sensors/ValorSensor.h"

#include <photonlib/PhotonCamera.h>
#include <photonlib/PhotonUtils.h>

#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableInstance.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include "networktables/NetworkTable.h"

class ValorVision : public ValorSensor<wpi::span<const photonlib::PhotonTrackedTarget>>
{
    public:
    ValorVision();
    void reset();
    void calculate();
    void aim();
    void takeImage();

    frc::Pose2d getPose{};
    std::shared_ptr<nt::NetworkTable> photonTable;

    private:
    photonlib::PhotonCamera camera{"valorVision"};
    frc::Translation3d finalPose{};
    
};
