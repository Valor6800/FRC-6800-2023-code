#include "auto/ValorAuto.h"
#include "Constants.h"

#include <fstream>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <iostream>
#include <map>
#include <functional>
#include <vector>
#include <string>

#include <wpi/ghc/filesystem.hpp>

#include <frc2/command/CommandBase.h>
#include <frc/DriverStation.h>

#include <frc2/command/PrintCommand.h>

#include <pathplanner/lib/PathConstraints.h>
#include <pathplanner/lib/PathPlanner.h>
#include <pathplanner/lib/PathPlannerTrajectory.h>
#include <pathplanner/lib/PathPoint.h>
#include <pathplanner/lib/auto/SwerveAutoBuilder.h>
#include <pathplanner/lib/commands/PPSwerveControllerCommand.h>

#define PATHS_PATH (std::string)"/home/lvuser/deploy/pathplanner/"
#define EVENTS_PATH (std::string)"/home/lvuser/auto/events/"
#define AUTOS_PATH (std::string)"/home/lvuser/auto/autos/"


ValorAuto::ValorAuto(Drivetrain *_drivetrain, Intake *_intake, Elevarm *_elevarm) :
    drivetrain(_drivetrain), intake(_intake), elevarm(_elevarm)
{
    drivetrain->getTrajectoryConfig().SetKinematics(*drivetrain->getKinematics());

    table = nt::NetworkTableInstance::GetDefault().GetTable("Auto");
    elevarmTable = nt::NetworkTableInstance::GetDefault().GetTable("Elevarm");

    precompileEvents(EVENTS_PATH);
}

ValorAuto::~ValorAuto()
{

}

// directory_iterator doesn't exist in vanilla c++11, luckily wpilib has it in their library
// https://first.wpi.edu/wpilib/allwpilib/docs/release/cpp/classghc_1_1filesystem_1_1directory__iterator.html
// implementation
// https://stackoverflow.com/questions/62409409/how-to-make-stdfilesystemdirectory-iterator-to-list-filenames-in-order
std::vector<std::string> listDirectory(std::string path_name){
    std::vector<std::string> files;

    for (auto &entry : ghc::filesystem::directory_iterator(path_name)) {
        std::string path = entry.path();
        if (path.find(".path") != std::string::npos || path.find(".csv") != std::string::npos) {
            files.push_back(entry.path());
        }
    }
    return files;
}

frc2::SequentialCommandGroup* ValorAuto::compileCommands(std::vector<ValorAutoAction> actions) {
    frc2::SequentialCommandGroup * commandGroup = new frc2::SequentialCommandGroup();

    for (int i = 0; i < actions.size(); i ++) {
        ValorAutoAction & action = actions[i];

        // Error encountered: stop trying to compile and output error
        if (action.type == ValorAutoAction::Type::STATE){
            std::function<void(void)> func;
            /* Example state setting command
            if (action.state == "flywheel"){
                func = [&, action] {
                    shooter->state.flywheelState = shooter->stringToFlywheelState(action.value);
                };
            }
            */ 
            commandGroup->AddCommands(frc2::InstantCommand(func));
        }
        else if (action.type == ValorAutoAction::Type::TIME){
            commandGroup->AddCommands(frc2::WaitCommand((units::millisecond_t)action.duration_ms));
        }
        else if (action.type == ValorAutoAction::Type::RESET_ODOM){
            if (action.vision){
                commandGroup->AddCommands(
                    std::move(*(drivetrain->getResetOdom()))
                );
            }
            else {
                commandGroup->AddCommands(
                    frc2::InstantCommand(
                        [&, action] {

                            frc::Pose2d p = action.pose;
                            drivetrain->resetOdometry(p);

                            // frc::Pose2d p = action.start;
                            // table->PutBoolean("Using x", action.used_vals.at("x"));
                            // table->PutBoolean("Using y", action.used_vals.at("y"));
                            // table->PutBoolean("Using angle", action.used_vals.at("angle"));
                            // p = frc::Pose2d{
                            //     (action.used_vals.at("x") ? p.X() : drivetrain->getPose_m().X()),
                            //     (action.used_vals.at("y") ? p.Y() : drivetrain->getPose_m().Y()),
                            //     (action.used_vals.at("angle") ? p.Rotation() : drivetrain->getPose_m().Rotation())
                            // };
                            // table->PutNumber("target pose x", p.X().to<double>());
                            // table->PutNumber("target pose y", p.Y().to<double>());
                            // table->PutNumber("target pose angle", p.Rotation().Degrees().to<double>());

                            //drivetrain->resetOdometry(p);
                        }
                    )
                );
            }
        } else if (action.type == ValorAutoAction::Type::XMODE){
            commandGroup->AddCommands(
                std::move(*(drivetrain->getSetXMode()))
            );
        } else if (action.type == ValorAutoAction::ACCELERATION){
            drivetrain->setAutoMaxAcceleration(action.maxAccel, action.accelMultiplier);
        }
        else if (action.type == ValorAutoAction::ELEVARM){
            Piece pieceState = elevarm->stringToPieceState(action.values[0]);
            Direction directionState = elevarm->stringToDirectionState(action.values[1]);
            Position positionState = elevarm->stringToPositionState(action.values[2]);

            if (!elevarmTable->GetBoolean("Pit Mode", false)) {
                commandGroup->AddCommands(
                    std::move(*elevarm->getAutoCommand(
                        action.values[0],
                        action.values[1],
                        action.values[2]
                    ))
                );
            }
            table->PutBoolean("Pit mode enabled for elevarm", elevarmTable->GetBoolean("Pit Mode", false));                
        } else if (action.type == ValorAutoAction::BALANCE){
            if (!action.oldBalance) {
                if (!action.reversed){
                    if (!action.vision)
                        commandGroup->AddCommands(
                            std::move(*drivetrain->getAutoLevel())
                        );
                    else
                        commandGroup->AddCommands(
                            std::move(*drivetrain->getVisionAutoLevel())
                        );
                }
                else
                    commandGroup->AddCommands(
                        std::move(*drivetrain->getAutoLevelReversed())
                    );
            } else {
                if (!action.reversed)
                    commandGroup->AddCommands(
                        std::move(*drivetrain->getOLDAutoLevel())
                    );
                else
                    commandGroup->AddCommands(
                        std::move(*drivetrain->getOLDAutoLevelReversed())
                    );
            }
        } else if (action.type == ValorAutoAction::INTAKE) {
            commandGroup->AddCommands(
                std::move(*intake->getAutoCommand(action.values[0], action.values[1]))
            );
        } else if (action.type == ValorAutoAction::CLIMB_OVER) {
            commandGroup->AddCommands(
                std::move(*drivetrain->getAutoClimbOver())
            );
        } //else if (action.type == ValorAutoAction::GO_TO){
        //     table->PutNumber("driving to", action.end.X().to<double>());
        //     commandGroup->AddCommands(
        //         frc2::FunctionalCommand(
        //             [&, action]() {
        //                 /*
        //                 frc::Pose2d & curPos = drivetrain->getPose_m();
        //                 double ang = curPos.Rotation().Radians().to<double>(), ang_d = curPos.Rotation().Degrees().to<double>();
        //                 ang = std::fmod(ang + 2 * PI, 2 * PI); // Normalize angle [0, 2 * pi]
        //                 ang_d = std::fmod(ang_d + 360, 360);
        //                 double x1 = std::cos(ang) * -20, y1 = std::sin(ang) * -20, x2 = std::cos(ang) * 20, y2 = std::sin(ang) * 20;
        //                 double x = action.end.X().to<double>(), y = action.end.Y().to<double>(); 
        //                 // Gives if the point is to the left of our line (or above it if the line is flat)
        //                 bool left = ((x1 - x2)*(y - y1) - (y2 - y1)*(x - x1)) > 0; // https://stackoverflow.com/questions/1560492/how-to-tell-whether-a-point-is-to-the-right-or-left-side-of-a-line
        //                 bool reversed = (((0 <= ang_d && ang_d < 90) || (270 <= ang_d && ang_d <= 360)) && left) || ((90 <= ang_d && ang_d < 270) && !left);
        //                 frc::Trajectory trajectory = createTrajectory({drivetrain->getPose_m(), action.end}, reversed);
        //                 */
        //                 std::vector<frc::Pose2d> poses = {drivetrain->getPose_m(), action.end};
        //                 frc::Trajectory trajectory = createTrajectory(poses, action.reversed);
        //                 frc2::SwerveControllerCommand<SWERVE_COUNT> _trajectoryCommand = createTrajectoryCommand(trajectory);
        //                 trajectoryCommand = &_trajectoryCommand;
        //                 trajectoryCommand->Initialize();
        //                 table->PutBoolean("initialized", true);
        //             },
        //             [&](){
        //                 // trajectoryCommand->Execute();
        //             },
        //             [&](bool interrupted){
        //                 // trajectoryCommand->End(interrupted);
        //             },
        //             [&](){
        //                 return trajectoryCommand->IsFinished();
        //             },
        //             {drivetrain}
        //         )
        //     );
        // }
    }
    return commandGroup;
}

void ValorAuto::precompileEvents(std::string directory_name) {
    eventMap.clear();
    sequentialEventMap.clear();

    std::vector<std::string> event_paths = listDirectory(directory_name);
    for (std::string event_path : event_paths){

        std::string name = event_path.substr(event_path.find_last_of('/') + 1, event_path.find_last_of('.') - event_path.find_last_of('/') - 1);

        std::ifstream infile(event_path);
        if (!infile.good()){
            return;
        }

        std::string line;

        int lineNum = 1;
        std::vector<ValorAutoAction> actions;
        while (std::getline(infile, line)){
            ValorAutoAction action(line);
            if (action.type != ValorAutoAction::NONE) {
                if (action.error != ValorAutoAction::NONE_ERROR) {
                    table->PutString("Loaded Event: " + name, "Failed");
                    table->PutString("Error in Event: " + name, "On line " + std::to_string(lineNum) + ": " + errorToStringMap[action.error] + " - " + action.error_message);
                    return;
                }
                actions.push_back(action);
            }
            lineNum++;
        }

        eventMap.emplace(name, compileCommands(actions));
        sequentialEventMap.emplace(name, std::move(*compileCommands(actions)));
        table->PutString("Loaded Event: " + name, "Success");
    }
}

frc2::Command * ValorAuto::createPPTrajectoryCommand(pathplanner::PathPlannerTrajectory trajectory){

    frc2::PIDController thetaController = frc2::PIDController(drivetrain->getThetaPIDF().P, drivetrain->getThetaPIDF().I, drivetrain->getThetaPIDF().D);
    thetaController.EnableContinuousInput(units::radian_t(-M_PI).to<double>(),
                                          units::radian_t(M_PI).to<double>());

    return new pathplanner::PPSwerveControllerCommand(trajectory,
			[&] () { return drivetrain->getPose_m(); },
			*drivetrain->getKinematics(),
			frc2::PIDController(drivetrain->getXPIDF().P, drivetrain->getXPIDF().I, drivetrain->getXPIDF().D),
            frc2::PIDController(drivetrain->getYPIDF().P, drivetrain->getYPIDF().I, drivetrain->getYPIDF().D),
			thetaController,
			[this] (auto states) { drivetrain->setModuleStates(states); },
			{drivetrain},
			true);
}

frc2::SequentialCommandGroup * ValorAuto::makePathAuto(std::string pathname){
    pathplanner::PathPlannerTrajectory trajectory = pathplanner::PathPlanner::loadPath(pathname, pathplanner::PathConstraints(units::meters_per_second_t(drivetrain->getAutoMaxSpeed()), units::meters_per_second_squared_t(drivetrain->getAutoMaxAcceleration())));
    if (trajectory.getStates().size() < 1) {
        table->PutString("Error in Path:", pathname + " does not exist");
        return nullptr;
    }

    table->PutString("Running Path:", pathname);

    pathplanner::PathPlannerTrajectory::PathPlannerState intitalState = pathplanner::PathPlannerTrajectory::transformStateForAlliance(trajectory.getInitialState(), frc::DriverStation::GetAlliance());
    drivetrain->resetOdometry(frc::Pose2d(intitalState.pose.X(), intitalState.pose.Y(), intitalState.holonomicRotation));

    frc2::SequentialCommandGroup * commandGroup = new frc2::SequentialCommandGroup();

    for (std::string name : trajectory.getStartStopEvent().names) {
        commandGroup->AddCommands(std::move(sequentialEventMap.at(name)));
    }
    
    commandGroup->AddCommands(
        pathplanner::FollowPathWithEvents (
            std::unique_ptr<frc2::Command>(createPPTrajectoryCommand(trajectory)),
            trajectory.getMarkers(),
            eventMap
        )
    );

    for (std::string name : trajectory.getEndStopEvent().names) {
        commandGroup->AddCommands(std::move(sequentialEventMap.at(name)));
    }

    return commandGroup;
}

frc2::SequentialCommandGroup * ValorAuto::makeAuto(std::string path) {
    std::string name = path.substr(path.find_last_of('/') + 1, path.find_last_of('.') - path.find_last_of('/') - 1);

    std::ifstream infile(path);
    if (!infile.good()){
        table->PutString("Error in Auto:", name + " does not exist");
        return nullptr;
    }

    std::string line;

    std::vector<ValorAutoAction> actions;
    while (std::getline(infile, line)){
        ValorAutoAction action(line);
        int lineNum = 1;
        if (action.type != ValorAutoAction::NONE) {
            if (action.error != ValorAutoAction::NONE_ERROR) {
                table->PutString("Error in Auto: " + name, "On line " + std::to_string(lineNum) + ": " + errorToStringMap[action.error] + " - " + action.error_message);
                return nullptr;
            }
            actions.push_back(action);
        }
        lineNum++;
    }

    return compileCommands(actions);
}

bool is_alpha(char c){
    return (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z');
}
bool is_caps(char c){
    return (c >= 'A' && c <= 'Z');
}

std::string makeFriendlyName(std::string filename){
    // take last part of the path string when divided with /'s - this should be the filename
    filename = filename.substr(filename.find_last_of('/') + 1);
    std::string n_name = "";
    for (uint i = 0; i < filename.length(); i ++){
        // .'s signify the end of the filename and the start of the file extension
        if (filename[i] == '.'){
            break;
        } else if (filename[i] == '_'){ // replace _'s with spaces for a snake case filename
            // make sure we dont have double spaces
            if (*(n_name.end() - 1) != ' ')
                n_name += ' ';
        } else if (i >= 1 && is_alpha(filename[i]) && is_caps(filename[i]) && !is_caps(filename[i - 1]) && *(n_name.end() - 1) != ' '){ // check for camel case, add space if present
            n_name += ' ';
            n_name += tolower(filename[i]);
        } else if (i == 0){ // first letter should be capitaized
            n_name += toupper(filename[i]);
        } else{
            n_name += tolower(filename[i]);
        }
    }
    return n_name;
}

bool directoryContainsPath(std::string directory, std::string path) {
    for (std::string p : listDirectory(directory)) {
        if (path == p) {
            return true;
        }
    }
    return false;
}

std::string removeFileType(std::string fileName) {
    return fileName.substr(fileName.find_last_of('/') + 1, fileName.find_last_of('.') - fileName.find_last_of('/') - 1);
}

frc2::SequentialCommandGroup * ValorAuto::getCurrentAuto(){
    precompileEvents(EVENTS_PATH);
    std::string selection = m_chooser.GetSelected();

    if (directoryContainsPath(AUTOS_PATH, selection)) {
        return makeAuto(selection);
    }
    return makePathAuto(selection);
}

void ValorAuto::fillAutoList(){
    for (std::string path : listDirectory(PATHS_PATH)){
        m_chooser.AddOption(makeFriendlyName(removeFileType(path)), removeFileType(path));
    }

    for (std::string path : listDirectory(AUTOS_PATH)){
        m_chooser.AddOption(makeFriendlyName(removeFileType(path)), path);
    }

    frc::SmartDashboard::PutData(&m_chooser);
}
