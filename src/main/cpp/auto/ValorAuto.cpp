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

#include <pathplanner/lib/PathConstraints.h>
#include <pathplanner/lib/PathPlanner.h>
#include <pathplanner/lib/PathPlannerTrajectory.h>
#include <pathplanner/lib/PathPoint.h>
#include <pathplanner/lib/auto/SwerveAutoBuilder.h>
#include <pathplanner/lib/commands/PPSwerveControllerCommand.h>

using namespace pathplanner;

#define PATHS_PATH (std::string)"/home/lvuser/deploy/"
#define EVENTS_PATH (std::string)"home/lvuser/events/"

#define PI 3.14159265359

ValorAuto::ValorAuto(Drivetrain *_drivetrain, Intake *_intake, Elevarm *_elevarm) :
    drivetrain(_drivetrain), intake(_intake), elevarm(_elevarm)
{
    drivetrain->getTrajectoryConfig().SetKinematics(*drivetrain->getKinematics());

    // @TODO look at angle wrapping and modding
    drivetrain->getThetaController().EnableContinuousInput(units::radian_t(-M_PI),
                                          units::radian_t(M_PI));

    table = nt::NetworkTableInstance::GetDefault().GetTable("Auto");
}

ValorAuto::~ValorAuto(){
    delete trajectoryCommand;
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

void ValorAuto::generateEventMap(){
    eventMap.clear();
    /*eventMap.emplace("score_high_cone", frc2::SequentialCommandGroup(
        std::move(*elevarm->getAutoCommand("cone", "front", "high", false)),
        frc2::WaitCommand(0.150_s),
        std::move(*intake->getAutoCommand("outtake", "cone"))
    ));*/
    eventMap.emplace("event", std::make_shared<frc2::InstantCommand>([&](){
        table->PutString("haii", "hewwwo");
    }));
}

PPSwerveControllerCommand * ValorAuto::createTrajectoryCommand(PathPlannerTrajectory trajectory){
    frc::ProfiledPIDController<units::radians>& tPID = drivetrain->getThetaController();
    return new PPSwerveControllerCommand(
        trajectory,
        [&] () { return drivetrain->getPose_m(); },
        *drivetrain->getKinematics(),
        frc2::PIDController(drivetrain->getXPIDF().P, drivetrain->getXPIDF().I, drivetrain->getXPIDF().D),
        frc2::PIDController(drivetrain->getYPIDF().P, drivetrain->getYPIDF().I, drivetrain->getYPIDF().D),
        frc2::PIDController(tPID.GetP(), tPID.GetI(), tPID.GetD()),
        [this] (std::array<frc::SwerveModuleState, SWERVE_COUNT> states) { drivetrain->setModuleStates(states); },
        {drivetrain},
        true
    );
}

std::unique_ptr<frc2::Command> ValorAuto::makeAuto(std::string pathname){
    generateEventMap();
    PathPlannerTrajectory path = PathPlanner::loadPath(pathname, PathConstraints(units::meters_per_second_t(drivetrain->getAutoMaxSpeed()), units::meters_per_second_squared_t(drivetrain->getAutoMaxAcceleration())));
    // path = PathPlannerTrajectory::transformTrajectoryForAlliance(path, frc::DriverStation::GetAlliance());
    drivetrain->resetOdometry(path.getInitialPose());
    // FollowPathWithEvents command(
    //     std::unique_ptr<frc2::SwerveControllerCommand<SWERVE_COUNT> >{createTrajectoryCommand(path)},
    //     path.getMarkers(),
    //     eventMap
    // );
    // SwerveAutoBuilder autoBuilder(
    //     [this]() { return drivetrain->getPose_m(); }, // Function to supply current robot pose
    //     [this](auto initPose) { drivetrain->resetOdometry(initPose); }, // Function used to reset odometry at the beginning of auto
    //     PIDConstants(60.0, 0.0, 0.0), // PID constants to correct for translation error (used to create the X and Y PID controllers)
    //     PIDConstants(15.0, 0.0, 0.0), // PID constants to correct for rotation error (used to create the rotation controller)
    //     [this](auto speeds) { drivetrain->drive(speeds.vx, speeds.vy, speeds.omega, true); }, // Output function that accepts field relative ChassisSpeeds
    //     eventMap, // Our event map
    //     { drivetrain }, // Drive requirements, usually just a single drive subsystem
    //     true // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
    // );
    return std::unique_ptr<PPSwerveControllerCommand>{createTrajectoryCommand(path)};
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

std::unique_ptr<frc2::Command> ValorAuto::getCurrentAuto(){
    // return makeAuto(m_chooser.GetSelected());
    return makeAuto("New Path");
}

void ValorAuto::fillAutoList(){
    std::vector<std::string> avPaths = listDirectory(PATHS_PATH);
    for (std::string a: avPaths){
        m_chooser.AddOption(makeFriendlyName(a), a);
    }
    frc::SmartDashboard::PutData(&m_chooser);
}
