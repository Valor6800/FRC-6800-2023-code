#include "subsystems/Drivetrain.h"
#include "subsystems/Intake.h"
#include "subsystems/Elevarm.h"
#include "ValorAutoAction.h"

#include <frc/trajectory/Trajectory.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/WaitCommand.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/ParallelCommandGroup.h>

#include "subsystems/Direction.h"
#include "subsystems/Position.h"
#include "subsystems/Piece.h"

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
        ValorAuto(Drivetrain*, Intake*, Elevarm*);
        bool readPointsCSV(std::string);
        frc2::SequentialCommandGroup* makeAuto(std::string, bool);
        void precompileActions(std::string);
        void fillAutoList();
        frc2::SequentialCommandGroup* getCurrentAuto();

    protected:

        frc::Trajectory createTrajectory(std::vector<frc::Pose2d>& poses, bool reversed, double, double);
        frc2::SwerveControllerCommand<SWERVE_COUNT> createTrajectoryCommand(frc::Trajectory);

        void readAuto(std::string);

    private:
        std::unordered_map<ValorAutoAction::Type, std::string> commandToStringMap = {
            {ValorAutoAction::NONE, "none"},
            {ValorAutoAction::TIME, "time"},
            {ValorAutoAction::STATE, "state"},
            {ValorAutoAction::TRAJECTORY, "trajectory"},
            {ValorAutoAction::RESET_ODOM, "reset_odom"},
            {ValorAutoAction::ACTION, "action"},
            {ValorAutoAction::SPLIT, "split"},
            {ValorAutoAction::XMODE, "xmode"},
            {ValorAutoAction::ACCELERATION, "acceleration"}
        };

        std::unordered_map<ValorAutoAction::Error, std::string> errorToStringMap = {
            {ValorAutoAction::NONE_ERROR, "no error"},
            {ValorAutoAction::POINT_MISSING, "non-existent point used"},
            {ValorAutoAction::SIZE_MISMATCH, "insufficient number of parameters passed in"}
        };

        std::vector<ValorAutoAction> autoActions;

        std::map<std::string, frc::Translation2d> points;
        Drivetrain *drivetrain;
        Intake *intake;
        Elevarm *elevarm;
        frc::SendableChooser<std::string> m_chooser;

        // std::map<std::string, std::vector<UsableCommand> > precompiledActions;
        std::map<std::string, frc2::SequentialCommandGroup *> precompiledActions;

        nt::NetworkTableEntry entry;

        std::shared_ptr<nt::NetworkTable> table;
};
#endif