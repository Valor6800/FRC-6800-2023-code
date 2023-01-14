#include <frc/geometry/Translation2d.h>
#include <frc/geometry/Pose2d.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/WaitCommand.h>
#include <frc2/command/SequentialCommandGroup.h>

#include <map>
#include <vector>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <iostream>
#include <string>

#ifndef VALOR_AUTO_ACTION_H
#define VALOR_AUTO_ACTION_H

struct ValorAutoAction {
    enum Type {
        NONE,
        TIME,
        STATE,
        TRAJECTORY,
        RESET_ODOM,
        ACTION
    } type;

    enum Error {
        NONE_ERROR, // can't have duplicate enum names
        SIZE_MISMATCH,
        POINT_MISSING
    } error;

    frc::Pose2d start;
    frc::Pose2d end;
    bool reversed;

    std::string state;
    std::string value;
    
    int duration_ms;

    ValorAutoAction(std::string line, std::map<std::string, frc::Translation2d> *);

    std::string name;
    

public:
    static std::vector<std::string> parseCSVLine(std::string);

private:
    frc::Pose2d getPose(frc::Translation2d, double);
};

#endif