#include <map>
#include <frc/geometry/Translation2d.h>

#include <frc/kinematics/DifferentialDriveWheelSpeeds.h>
#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc/kinematics/DifferentialDriveKinematics.h>
#include <frc/trajectory/constraint/DifferentialDriveVoltageConstraint.h>
#include <frc/trajectory/Trajectory.h>
#include <frc/trajectory/TrajectoryGenerator.h>

#include <frc2/command/RamseteCommand.h>
#include <frc2/command/Command.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/WaitCommand.h>

#ifndef VALOR_AUTO_H
#define VALOR_AUTO_H

class ValorAuto {
    public:
        ValorAuto();

        frc2::Command* getCurrentAuto();

    private:

        std::map<std::string, frc2::SequentialCommandGroup*> autos;
};

#endif