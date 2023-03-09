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

#define ROOT_AUTO_PATH (std::string)"/home/lvuser/auto/"

ValorAuto::ValorAuto(Drivetrain *_drivetrain, Intake *_intake, Elevarm *_elevarm) :
    drivetrain(_drivetrain), intake(_intake), elevarm(_elevarm)
{
    drivetrain->getTrajectoryConfig().SetKinematics(*drivetrain->getKinematics());

    // @TODO look at angle wrapping and modding
    drivetrain->getThetaController().EnableContinuousInput(units::radian_t(-M_PI),
                                          units::radian_t(M_PI));

    table = nt::NetworkTableInstance::GetDefault().GetTable("Auto");
}

// directory_iterator doesn't exist in vanilla c++11, luckily wpilib has it in their library
// https://first.wpi.edu/wpilib/allwpilib/docs/release/cpp/classghc_1_1filesystem_1_1directory__iterator.html
// implementation
// https://stackoverflow.com/questions/62409409/how-to-make-stdfilesystemdirectory-iterator-to-list-filenames-in-order
std::vector<std::string> listDirectory(std::string path_name){
    std::vector<std::string> files;

    for (auto &entry : ghc::filesystem::directory_iterator(path_name))
        files.push_back(entry.path());
    return files;
}

frc2::SwerveControllerCommand<SWERVE_COUNT> ValorAuto::createTrajectoryCommand(frc::Trajectory trajectory)
{
    return frc2::SwerveControllerCommand<SWERVE_COUNT>(
        trajectory,
        [&] () { return drivetrain->getPose_m(); },
        *drivetrain->getKinematics(),
        frc2::PIDController(drivetrain->getXPIDF().P, drivetrain->getXPIDF().I, drivetrain->getXPIDF().D),
        frc2::PIDController(drivetrain->getYPIDF().P, drivetrain->getYPIDF().I, drivetrain->getYPIDF().D),
        drivetrain->getThetaController(),
        [this] (auto states) { drivetrain->setModuleStates(states); },
        {drivetrain}
    );
}

frc::Trajectory ValorAuto::createTrajectory(std::vector<frc::Pose2d>& poses, bool reversed, double start_vel=0, double end_vel=0)
{
    frc::TrajectoryConfig & config = drivetrain->getTrajectoryConfig();
    config.SetReversed(reversed);
    config.SetStartVelocity(units::meters_per_second_t{start_vel});
    config.SetEndVelocity(units::meters_per_second_t{end_vel});
    
    return frc::TrajectoryGenerator::GenerateTrajectory(poses, drivetrain->getTrajectoryConfig());
}

/* Read in the points from a CSV file.
 * Each line in the CSV file must be written in the following format:
`point_name,point_x,point_y`
 * Example points CSV file:
 ```Bugs,5,2.5
    F1,8,9
    start,0,0```
 * Note that there are no spaces.
 * There are no restricitions on what points must be included in the file - just
   make sure that the points used in whatever auto you are using also exist in this CSV file.
 */
bool ValorAuto::readPointsCSV(std::string filename){
    std::ifstream infile(filename);
    if (!infile.good()){
        table->PutString("Error", "points file not found");
        return false;
    }

    points.clear();

    std::string line; 
    while (std::getline(infile, line)){
        std::vector<std::string> items = ValorAutoAction::parseCSVLine(line);

        // empty or invalid line
        if (items.size() != 3 || (!items.empty() && items[0].starts_with("//")))
            continue;

        std::string name = items[0];
        double x = std::stod(items[1]), y = std::stod(items[2]);
        points[name] = frc::Translation2d((units::length::meter_t)x, (units::length::meter_t)y);
    }
    return true;
}

void createTrajectoryDebugFile(frc::Trajectory& trajectory, int i){
    std::ofstream outfile("/home/lvuser/trajectory" + std::to_string(i) + ".csv");
    if (!outfile.good()){
        return;
    }

    std::vector<frc::Trajectory::State> states = trajectory.States();
    for (frc::Trajectory::State state: states){
        outfile << std::to_string(state.t.to<double>()) + "," + std::to_string(state.pose.X().to<double>()) + "," + std::to_string(state.pose.Y().to<double>()) + "," + std::to_string(state.pose.Rotation().Degrees().to<double>()) + "," + std::to_string(state.velocity.to<double>()) + "\n";
    }
}


frc2::SequentialCommandGroup* ValorAuto::makeAuto(std::string filename, bool blueSide=false){
    auto elevarmTable = nt::NetworkTableInstance::GetDefault().GetTable("Elevarm");

    frc2::SequentialCommandGroup *currentGroup = new frc2::SequentialCommandGroup();
    std::vector<frc2::SequentialCommandGroup> cmdGroups = {};

    std::ifstream infile(filename); 
    if (!infile.good()){
        return nullptr;
    }

    std::string line;

    std::vector<ValorAutoAction> actions;
    while (std::getline(infile, line)){
        ValorAutoAction action(line, &points, blueSide);
        if (action.type != ValorAutoAction::NONE)
            actions.push_back(action);
    }

    // Stores trajectory poses until a non-trajectory command is run,
    // in which case poses stored here are compiled into a single trajectory
    // and placed into the command group, and then this is reset
    std::vector<frc::Pose2d> trajPoses = {};
    bool trajReversed = false;
    frc::Rotation2d last_angle;

    int trajCount = 0;

    drivetrain->setAutoMaxAcceleration(NULL, 1.0);
    
    currentGroup->AddCommands(std::move(*elevarm->getRotatePIDSetterCommand(true)));

    for (int i = 0; i < actions.size(); i ++){
        table->PutString("Action " + std::to_string(i), commandToStringMap[actions[i].type]);
        ValorAutoAction & action = actions[i];

        // Error encountered: stop trying to compile and output error
        if (action.error != ValorAutoAction::NONE_ERROR){
            table->PutString("Error", "On line " + std::to_string(i + 1) + ": " + errorToStringMap[action.error] + " - " + action.error_message);
            return nullptr;
        }

        if (action.type == ValorAutoAction::Type::TRAJECTORY){
            if (trajPoses.size() == 0){ // starting a new trajectory, set poses inside to stored action.start and action.end
                action.start = frc::Pose2d{action.start.X(), action.start.Y(), last_angle};
                trajPoses = {action.start, action.end};
                trajReversed = action.reversed;
                last_angle = trajPoses.back().Rotation();
            }
            else {
                // Create a new trajectory with each switch of normal <-> reversed, as they use different configs
                if (action.reversed != trajReversed){ 
                    if (trajPoses.size() > 0)
                        last_angle = trajPoses.back().Rotation();

                    double s_vel = 0, e_vel = 0;
                    int ei = i - 1, si = i - 1;
                    while (actions[si].type == ValorAutoAction::TRAJECTORY && actions[si].reversed == trajReversed)
                        si--;
                    si++;
                    if (si >= 2 && actions[si - 1].type == ValorAutoAction::SPLIT) {
                        s_vel = actions[si - 1].vel;
                        if (si >=3 && actions[si - 2].type == ValorAutoAction::ACCELERATION) {
                        }
                    }
                    if (ei + 2 < actions.size() && actions[ei + 1].type == ValorAutoAction::SPLIT)
                        e_vel = actions[ei + 1].vel;

                    if (si >= 2 && actions[si - 1].type == ValorAutoAction::ACCELERATION) {
                        if (si >= 3 && actions[si - 2].type == ValorAutoAction::SPLIT) {
                            s_vel = actions[si - 1].vel;
                        }
                    }
                    table->PutNumber("trajectory " + std::to_string(trajCount) + " start velocity", s_vel);
                    table->PutNumber("trajectory " + std::to_string(trajCount) + " end velocity", e_vel);
                    
                    frc::Trajectory trajectory = createTrajectory(trajPoses, trajReversed, s_vel, e_vel);
                    table->PutNumber("trajectory " + std::to_string(trajCount) + " time", std::fabs(trajectory.TotalTime().to<double>()));
                    if (std::fabs(trajectory.TotalTime().to<double>()) < 0.001){
                        table->PutString("Error", "On line " + std::to_string(i + 1) + ": malformed trajectory");
                        return nullptr;
                    }
                    createTrajectoryDebugFile(trajectory, trajCount);
                    trajCount++;
                    currentGroup->AddCommands(createTrajectoryCommand(trajectory));
                    trajPoses.clear();

                    action.start = frc::Pose2d{action.start.X(), action.start.Y(), last_angle};
                    trajPoses = {action.start, action.end};
                    trajReversed = action.reversed;
                }
                else {
                    trajPoses.push_back(action.end);
                }
            }
        } else{
            if (trajPoses.size() != 0){
                if (trajPoses.size() > 0)
                    last_angle = trajPoses.back().Rotation();
                
                double s_vel = 0, e_vel = 0;
                int ei = i - 1, si = i - 1;
                while (actions[si].type == ValorAutoAction::TRAJECTORY && actions[si].reversed == trajReversed)
                    si--;
                si++;
                if (si >= 2 && actions[si - 1].type == ValorAutoAction::SPLIT) {
                    s_vel = actions[si - 1].vel;
                    if (si >=3 && actions[si - 2].type == ValorAutoAction::ACCELERATION) {
                    }
                }

                if (ei + 2 < actions.size() && actions[ei + 1].type == ValorAutoAction::SPLIT)
                    e_vel = actions[ei + 1].vel;

                if (si >= 2 && actions[si - 1].type == ValorAutoAction::ACCELERATION) {
                    if (si >= 3 && actions[si - 2].type == ValorAutoAction::SPLIT) {
                        s_vel = actions[si - 1].vel;
                    }
                }

                table->PutNumber("trajectory " + std::to_string(trajCount) + " start velocity", s_vel);
                table->PutNumber("trajectory " + std::to_string(trajCount) + " end velocity", e_vel);

                frc::Trajectory trajectory = createTrajectory(trajPoses, trajReversed, s_vel, e_vel);
                table->PutNumber("trajectory " + std::to_string(trajCount) + " time", std::fabs(trajectory.TotalTime().to<double>()));
                if (std::fabs(trajectory.TotalTime().to<double>()) < 0.001){
                    table->PutString("Error", "On line " + std::to_string(i + 1) + ": malformed trajectory");
                    return nullptr;
                }
                createTrajectoryDebugFile(trajectory, trajCount);
                trajCount++;
                currentGroup->AddCommands(createTrajectoryCommand(trajectory));
                trajPoses.clear();
            }

            if (action.type == ValorAutoAction::Type::STATE){
                std::function<void(void)> func;
                /* Example state setting command

                if (action.state == "flywheel"){
                    func = [&, action] {
                        shooter->state.flywheelState = shooter->stringToFlywheelState(action.value);
                    };
                }
                */ 
                currentGroup->AddCommands(frc2::InstantCommand(func));
            }
            else if (action.type == ValorAutoAction::Type::TIME){
                currentGroup->AddCommands(frc2::WaitCommand((units::millisecond_t)action.duration_ms));
            }
            else if (action.type == ValorAutoAction::Type::RESET_ODOM){
                currentGroup->AddCommands(
                    frc2::InstantCommand(
                        [&, action] {
                            drivetrain->resetOdometry(action.start);
                        }
                    )
                );
                last_angle = action.start.Rotation();
            }
            else if (action.type == ValorAutoAction::Type::ACTION){
                /*
                This doesn't work for a specific reason.
                If you look into SequentialCommandGroup.h, you'll find the following:

                SequentialCommandGroup(SequentialCommandGroup&& other) = default;
                SequentialCommandGroup(const SequentialCommandGroup&) = delete;
                SequentialCommandGroup(SequentialCommandGroup&) = delete;

                You'll notice that functions taking in SequentialCommandGroup& are unavailable, which is exactly what we're passing in
                However, SequeuentialCommandGroup&& is available.
                This means that rvalue SequentialCommandGroups can be passed in.
                rvalue are values that have no memory address.
                Your typical values, defined by something like
                int a = 5;
                are lvalues. They are an object reference, as compared to an rvalue, which is just a value, not related to any object.
                So in this scenario, if you were to access a, that would be an lvalue,
                but if you acccessed 5, that would be an rvalue
                rvalues are defined with
                int&& a = 5;

                So either you create the command group right here, or you need to find a way to convert
                lvalues (the typical ones) to rvalues
                */

                // ...which is why this code works
                
                /*
                cmdGroup->AddCommands(frc2::SequentialCommandGroup{
                    frc2::InstantCommand(
                        [&, action] {
                            drivetrain->resetOdometry(frc::Pose2d(0_m, 0_m, 0_deg));
                        }
                    )
                });*/

                /*
                frc2::SequentialCommandGroup grp{};
                cmdGroup->AddCommands(grp); // This doesn't compile!
                cmdGroup->AddCommands(std::move(grp)); This does!!!!
                */
                

                // The issue is not with the pointers!!
                // cmdGroup->AddCommands((*precompiledActions[action.name]));
                // std::move - Convert a value to an rvalue
                if (action.parallel){
                    cmdGroups.push_back(std::move(*currentGroup));
                    cmdGroups.push_back(std::move(*precompiledActions[action.name]));
                    currentGroup = new frc2::SequentialCommandGroup();
                }
                else
                    currentGroup->AddCommands(std::move(*precompiledActions[action.name]));

                /*
                C++ doesn't allow you to pass an lvalue into an rvalue reference, as  
                you may not know that that destroys it. As such, you have to knowingly destroy it
                with std::move.
                */

                /*
                What confuses me about this is that the header code I showed above is for the constructor.
                Why would passing in a command (in this case a SequentialCommandGroup) call the constructor for the cmdGroup?
                */
            } else if (action.type == ValorAutoAction::Type::XMODE){
                currentGroup->AddCommands(
                    std::move(*(drivetrain->getSetXMode()))
                );
            } else if (action.type == ValorAutoAction::ACCELERATION){
               drivetrain->setAutoMaxAcceleration(action.maxAccel, action.accelMultiplier);
            }
            else if (action.type == ValorAutoAction::ELEVARM){
                Piece pieceState = elevarm->stringToPieceState(action.values[0]);
                Direction directionState = elevarm->stringToDirectionState(action.values[1]);
                Position positionState = elevarm->stringToPositionState(action.values[2]);

                if (!elevarmTable->GetBoolean("Pit Mode", false))
                    currentGroup->AddCommands(
                        std::move(*elevarm->getAutoCommand(
                            action.values[0],
                            action.values[1],
                            action.values[2],
                            action.parallel
                        ))
                    );
                table->PutBoolean("Pit mode enabled for elevarm", elevarmTable->GetBoolean("Pit Mode", false));
                
                table->PutBoolean("Action " + std::to_string(i) + " parallel", action.parallel);
            } else if (action.type == ValorAutoAction::BALANCE){
                if (!action.reversed)
                    currentGroup->AddCommands(
                        std::move(*drivetrain->getAutoLevel())
                    );
                else
                    currentGroup->AddCommands(
                        std::move(*drivetrain->getAutoLevelReversed())
                    );
            } else if (action.type == ValorAutoAction::INTAKE) {
                currentGroup->AddCommands(
                    std::move(*intake->getAutoCommand(action.value))
                );
            }
        }
    }

    if (trajPoses.size() != 0){
        int i = actions.size() - 1;
        double s_vel = 0, e_vel = 0;

        int ei = i - 1, si = i - 1;
        while (actions[si].type == ValorAutoAction::TRAJECTORY && actions[si].reversed == trajReversed)
            si--;
        si++;
        if (si >= 2 && actions[si - 1].type == ValorAutoAction::SPLIT) {
            s_vel = actions[si - 1].vel;
            if (si >=3 && actions[si - 2].type == ValorAutoAction::ACCELERATION) {
            }
        }
        if (ei + 2 < actions.size() && actions[ei + 1].type == ValorAutoAction::SPLIT)
            e_vel = actions[ei + 1].vel;

        if (si >= 2 && actions[si - 1].type == ValorAutoAction::ACCELERATION) {
            if (si >= 3 && actions[si - 2].type == ValorAutoAction::SPLIT) {
                s_vel = actions[si - 1].vel;
            }
        }
        table->PutNumber("trajectory " + std::to_string(trajCount) + " start velocity", s_vel);
        table->PutNumber("trajectory " + std::to_string(trajCount) + " end velocity", e_vel);
                        
        
        frc::Trajectory trajectory = createTrajectory(trajPoses, trajReversed, s_vel, e_vel);
        table->PutNumber("trajectory " + std::to_string(trajCount) + " time", std::fabs(trajectory.TotalTime().to<double>()));
        if (std::fabs(trajectory.TotalTime().to<double>()) < 0.001){
            table->PutString("Error", "On line " + std::to_string(i + 1) + ": malformed trajectory");
            return nullptr;
        }
        createTrajectoryDebugFile(trajectory, trajCount);
        trajCount++;
        currentGroup->AddCommands(createTrajectoryCommand(trajectory));
        trajPoses.empty();
    }
    cmdGroups.push_back(std::move(*currentGroup));

    // bad way of doing this currently this relies on 
    // adding the action command group and the other command group in a specific order, which i know i'll mess up some day
    // the action command group and other cocmmadn group should be named
    frc2::SequentialCommandGroup * combinedGroup = new frc2::SequentialCommandGroup(std::move(cmdGroups[0]));
    for (int i = cmdGroups.size() - 1; i >= 1; i -= 2){
        combinedGroup->AddCommands(
            frc2::ParallelCommandGroup(
                std::move(cmdGroups[i]), std::move(cmdGroups[i - 1])
            )
        );
    }

    combinedGroup->AddCommands(std::move(*elevarm->getRotatePIDSetterCommand(false)));

    return combinedGroup;
}

void ValorAuto::precompileActions(std::string path_name){
    precompiledActions.clear();
    std::vector<std::string> action_paths = listDirectory(path_name);
    for (std::string action_path: action_paths){
        frc2::SequentialCommandGroup * cmdGroup = makeAuto(action_path);
        std::string name = action_path.substr(action_path.find_last_of('/') + 1, action_path.find_last_of('.') - action_path.find_last_of('/') - 1);
        
        precompiledActions[name] = (cmdGroup);
    }
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

frc2::SequentialCommandGroup * ValorAuto::getCurrentAuto(){
    table->PutString("Error", "none");
    bool read_points = false;
    if (frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kBlue){
        read_points = readPointsCSV(ROOT_AUTO_PATH + "points/blue_points.csv");
    }
    else {
        read_points = readPointsCSV(ROOT_AUTO_PATH + "points/red_points.csv");
    }
    if (!read_points)
        return nullptr;
    
    precompileActions(ROOT_AUTO_PATH + "actions/");

    return makeAuto(m_chooser.GetSelected(), frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kBlue);
}

void ValorAuto::fillAutoList(){
    std::string autos_path = ROOT_AUTO_PATH + "autos/";
    std::vector<std::string> avAutos = listDirectory(autos_path);
    for (std::string a: avAutos){
        m_chooser.AddOption(makeFriendlyName(a), a);
    }
    frc::SmartDashboard::PutData(&m_chooser);
}
