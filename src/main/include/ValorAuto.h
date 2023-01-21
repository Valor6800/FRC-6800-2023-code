#include "subsystems/Drivetrain.h"
#include "subsystems/Elevarm.h"
#include "ValorAutoAction.h"

#include <frc/trajectory/Trajectory.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/WaitCommand.h>
#include <frc2/command/SequentialCommandGroup.h>


#include <string>
#include <vector>
#include <map>

#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/NetworkTableEntry.h>

#include <cstdint>
#include <iostream>

#ifndef VALOR_AUTO_H
#define VALOR_AUTO_H

struct UsableCommand{
    frc2::InstantCommand instantCommand;
    frc2::WaitCommand waitCommand;
};

class ValorAuto {
    public:
        ValorAuto(Drivetrain*, Elevarm*);
        void readPointsCSV(std::string);
        frc2::SequentialCommandGroup* makeAuto(std::string);
        void precompileActions(std::string);
        void fillAutoList();
        frc2::SequentialCommandGroup* getCurrentAuto();

    protected:

        frc::Trajectory createTrajectory(std::vector<frc::Pose2d>& poses, bool reversed = false);
        frc2::SwerveControllerCommand<SWERVE_COUNT> createTrajectoryCommand(frc::Trajectory);

        void readAuto(std::string);

    private:


        std::vector<ValorAutoAction> autoActions;

        std::map<std::string, frc::Translation2d> points;
        Drivetrain *drivetrain;
        Elevarm *elevarm;
        frc::SendableChooser<std::string> m_chooser;

        // std::map<std::string, std::vector<UsableCommand> > precompiledActions;
        std::map<std::string, frc2::SequentialCommandGroup *> precompiledActions;

        nt::NetworkTableEntry entry;
};
#endif