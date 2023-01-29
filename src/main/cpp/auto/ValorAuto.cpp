#include "ValorAuto.h"
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

#define TRANS_VELOCITY 4.952f

ValorAuto::ValorAuto(Drivetrain *_drivetrain) :
    drivetrain(_drivetrain)
{
    drivetrain->getTrajectoryConfig().SetKinematics(*drivetrain->getKinematics());

    // @TODO look at angle wrapping and modding
    drivetrain->getThetaController().EnableContinuousInput(units::radian_t(-M_PI),
                                          units::radian_t(M_PI));
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
void ValorAuto::readPointsCSV(std::string filename){
    std::ifstream infile(filename);
    if (!infile.good()){
        return;
    }

    points.clear();

    std::string line; 
    while (std::getline(infile, line)){
        std::vector<std::string> items = ValorAutoAction::parseCSVLine(line);

        // empty or invalid line
        if (items.size() != 3)
            continue;

        std::string name = items[0];
        double x = std::stod(items[1]), y = std::stod(items[2]);
        points[name] = frc::Translation2d((units::length::meter_t)x, (units::length::meter_t)y);
    }
}

void createTrajectoryDebugFile(frc::Trajectory& trajectory, int i){
    std::ofstream outfile("/home/lvuser/trajectory" + std::to_string(i) + ".csv");
    if (!outfile.good()){
        return;
    }

    std::vector<frc::Trajectory::State> states = trajectory.States();
    for (frc::Trajectory::State state: states){
        outfile << std::to_string(state.t.to<double>()) + "," + std::to_string(state.pose.X().to<double>()) + "," + std::to_string(state.pose.Y().to<double>()) + "," + std::to_string(state.pose.Rotation().Degrees().to<double>()) + "," + std::to_string(state.acceleration.to<double>()) + "\n";
    }
}

frc2::SequentialCommandGroup* ValorAuto::makeAuto(std::string filename){
    std::shared_ptr<nt::NetworkTable> table = nt::NetworkTableInstance::GetDefault().GetTable("Auto");
    frc2::SequentialCommandGroup *cmdGroup = new frc2::SequentialCommandGroup();

    std::ifstream infile(filename); 
    if (!infile.good()){
        return cmdGroup;
    }

    std::string line;

    std::vector<ValorAutoAction> actions;
    // @todo PLEASE find a better way to do this. 
    std::vector<bool> traj_directions;
    while (std::getline(infile, line)){
        actions.push_back(ValorAutoAction(line, &points));
        if (actions.back().type == ValorAutoAction::Type::TRAJECTORY){
            traj_directions.push_back(actions.back().reversed);
        }
    }

    // Stores trajectory poses until a non-trajectory command is run,
    // in which case poses stored here are compiled into a single trajectory
    // and placed into the command group, and then this is reset
    std::vector<frc::Pose2d> trajPoses = {};
    bool trajReversed = false;
    frc::Rotation2d last_angle;

    int traj_count = 0;

    for (ValorAutoAction action: actions){
        if (action.type == ValorAutoAction::Type::TRAJECTORY){
            if (trajPoses.size() == 0){ // starting a new trajectory, set poses inside to stored action.start and action.end
                action.start = frc::Pose2d{action.start.X(), action.start.Y(), last_angle};
                trajPoses = {action.start, action.end};
                trajReversed = action.reversed;
                // table->PutNumber("Trajectory " + std::to_string(traj_count) + " start angle", last_angle.Degrees().to<double>());
                last_angle = trajPoses.back().Rotation();
            }
            else {
                // Create a new trajectory with each switch of normal <-> reversed, as they use different configs
                if (action.reversed != trajReversed){ 
                    // std::vector<double> angles;
                    // for (frc::Pose2d pose: trajPoses){
                    //     angles.push_back(pose.Rotation().Degrees().to<double>());
                    // }
                    // table->PutNumberArray("Trajectory " + std::to_string(traj_count), angles);

                    if (trajPoses.size() > 0)
                        last_angle = trajPoses.back().Rotation();

                    double s_vel = 0, e_vel = 0;
                    if (traj_count > 0 && traj_directions[traj_count] == traj_directions[traj_count - 1])
                        s_vel = TRANS_VELOCITY;
                    if (traj_count < traj_directions.size() - 1 && traj_directions[traj_count] == traj_directions[traj_count + 1])
                        e_vel = TRANS_VELOCITY;
                    
                    frc::Trajectory trajectory = createTrajectory(trajPoses, trajReversed, s_vel, e_vel);
                    // createTrajectoryDebugFile(trajectory, traj_count);
                    traj_count ++;
                    cmdGroup->AddCommands(createTrajectoryCommand(trajectory));
                    trajPoses.clear();

                    action.start = frc::Pose2d{action.start.X(), action.start.Y(), last_angle};
                    trajPoses = {action.start, action.end};
                    // table->PutNumber("Trajectory " + std::to_string(traj_count) + " start angle", last_angle.Degrees().to<double>());
                    trajReversed = action.reversed;
                }
                else {
                    trajPoses.push_back(action.end);
                }
            }
        }
        else {
            if (trajPoses.size() != 0){
                // std::vector<double> angles;
                // for (frc::Pose2d pose: trajPoses){
                //     angles.push_back(pose.Rotation().Degrees().to<double>());
                // }
                // table->PutNumberArray("Trajectory " + std::to_string(traj_count), angles);

                if (trajPoses.size() > 0)
                    last_angle = trajPoses.back().Rotation();

                double s_vel = 0, e_vel = 0;
                if (traj_count > 0 && traj_directions[traj_count] == traj_directions[traj_count - 1])
                    s_vel = TRANS_VELOCITY;
                if (traj_count < traj_directions.size() - 1 && traj_directions[traj_count] == traj_directions[traj_count + 1])
                    e_vel = TRANS_VELOCITY;
                
                frc::Trajectory trajectory = createTrajectory(trajPoses, trajReversed, s_vel, e_vel);
                // createTrajectoryDebugFile(trajectory, traj_count);
                traj_count ++;
                cmdGroup->AddCommands(createTrajectoryCommand(trajectory));
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
                cmdGroup->AddCommands(frc2::InstantCommand(func));
            }
            else if (action.type == ValorAutoAction::Type::TIME){
                cmdGroup->AddCommands(frc2::WaitCommand((units::millisecond_t)action.duration_ms));
            }
            else if (action.type == ValorAutoAction::Type::RESET_ODOM){
                cmdGroup->AddCommands(
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
                cmdGroup->AddCommands(std::move(*precompiledActions[action.name]));

                /*
                C++ doesn't allow you to pass an lvalue into an rvalue reference, as  
                you may not know that that destroys it. As such, you have to knowingly destroy it
                with std::move.
                */
                
                /*
                What confuses me about this is that the header code I showed above is for the constructor.
                Why would passing in a command (in this case a SequentialCommandGroup) call the constructor for the cmdGroup?
                */
            }
        }
    }

    if (trajPoses.size() != 0){
        // std::vector<double> angles;
        // for (frc::Pose2d pose: trajPoses){
        //     angles.push_back(pose.Rotation().Degrees().to<double>());
        // }
        // table->PutNumberArray("Trajectory " + std::to_string(traj_count), angles);
        
        double s_vel = 0, e_vel = 0;
        if (traj_count > 0 && traj_directions[traj_count] == traj_directions[traj_count - 1])
            s_vel = TRANS_VELOCITY;
        if (traj_count < traj_directions.size() - 1 && traj_directions[traj_count] == traj_directions[traj_count + 1])
            e_vel = TRANS_VELOCITY;
        
        frc::Trajectory trajectory = createTrajectory(trajPoses, trajReversed, s_vel, e_vel);
        // createTrajectoryDebugFile(trajectory, traj_count);
        traj_count ++;
        cmdGroup->AddCommands(createTrajectoryCommand(trajectory));
        trajPoses.empty();
    }

    table->PutNumber("trajectory count", traj_count);

    return cmdGroup;
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

frc2::SequentialCommandGroup * ValorAuto::getCurrentAuto()
{
    readPointsCSV("/home/lvuser/test_points.csv");
    precompileActions("/home/lvuser/actions/");

    return makeAuto(m_chooser.GetSelected());
}

void ValorAuto::fillAutoList(){
    std::string autos_path = "/home/lvuser/auto_csvs/";
    std::vector<std::string> avAutos = listDirectory(autos_path);
    for (std::string a: avAutos){
        m_chooser.AddOption(makeFriendlyName(a), a);
    }
    frc::SmartDashboard::PutData(&m_chooser);
}