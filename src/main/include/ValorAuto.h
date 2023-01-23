#include "subsystems/Drivetrain.h"
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
        ValorAuto(Drivetrain*);
        void readPointsCSV(std::string);
        frc2::SequentialCommandGroup* makeAuto(std::string);
        void precompileActions(std::string);
        void fillAutoList();
        frc2::SequentialCommandGroup* getCurrentAuto();

    /**
      * @brief Destroy the ValorAuto object
      * 
      * ValorAuto objects have member objects on the heap - need a destructor to take care of memory on destruction
      */
     ~ValorAuto();

    protected:

        frc::Trajectory createTrajectory(std::vector<frc::Pose2d>& poses, bool reversed = false);
        frc2::SwerveControllerCommand<SWERVE_COUNT> createTrajectoryCommand(frc::Trajectory);

        void readAuto(std::string);

    private:

        units::velocity::meters_per_second_t * driveMaxSpeed;

        frc::TrajectoryConfig * config;

        std::vector<ValorAutoAction> autoActions;

        frc::ProfiledPIDController<units::radians> * thetaController;

        std::map<std::string, frc::Translation2d> points;
        Drivetrain *drivetrain;
        frc::SendableChooser<std::string> m_chooser;

        // std::map<std::string, std::vector<UsableCommand> > precompiledActions;
        std::map<std::string, frc2::SequentialCommandGroup *> precompiledActions;

        nt::NetworkTableEntry entry;
};
#endif