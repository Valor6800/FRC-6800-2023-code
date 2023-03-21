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

using namespace pathplanner;

#define PATHS_PATH (std::string)"/home/lvuser/"

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

FollowPathWithEvents* ValorAuto::makeAuto(std::string pathname){
    PathPlannerTrajectory path = PathPlanner::loadPath(pathname, PathConstraints(units::meters_per_second_t(drivetrain->getAutoMaxSpeed()), units::meters_per_second_squared_t(drivetrain->getAutoMaxAcceleration())));
    std::unordered_map<std::string, std::shared_ptr<frc2::Command> > eventMap;
    eventMap.emplace("prep cube intake", std::move(*elevarm->getAutoCommand("cube", "back", "ground", false)));

    return new FollowPathWithEvents(
        getPathFollowingCommand(path),
        path.getMarkers(),
        eventMap
    );
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

FollowPathWithEvents * ValorAuto::getCurrentAuto(){
    return makeAuto(m_chooser.GetSelected());
}

void ValorAuto::fillAutoList(){
    std::vector<std::string> avPaths = listDirectory(PATHS_PATH);
    for (std::string a: avPaths){
        m_chooser.AddOption(makeFriendlyName(a), a);
    }
    frc::SmartDashboard::PutData(&m_chooser);
}
