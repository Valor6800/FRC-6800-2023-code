#pragma once

#include "sensors/ValorSensor.h"

#include <photonlib/PhotonCamera.h>
#include <photonlib/PhotonUtils.h>

class ValorVision : public ValorSensor<photonlib::PhotonPipelineResult>
{
    public:
    ValorVision();
    void reset();
    void calculate();
    void aim();
    void takeImage();

    frc::Pose2d getPose{};
    std::shared_ptr<nt::NetworkTable> photonTable;
    frc::Translation2d finalPose{};

    private:
    photonlib::PhotonCamera camera{"valorVision"};
    
};
