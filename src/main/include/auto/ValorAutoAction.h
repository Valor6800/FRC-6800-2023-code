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
#include <unordered_map>
#include <networktables/NetworkTable.h>

#ifndef VALOR_AUTO_ACTION_H
#define VALOR_AUTO_ACTION_H

struct ValorAutoAction {
    enum Type {
        NONE,
        TIME,
        STATE,
        RESET_ODOM,
        XMODE,
        ELEVARM,
        ACCELERATION,
        BALANCE,
        INTAKE,
        CLIMB_OVER,
        GO_TO
    } type;

    enum Error {
        NONE_ERROR, // can't have duplicate enum names
        SIZE_MISMATCH,
        COMMAND_MISSING
    } error;
    std::string error_message;
    
    //frc::Pose2d pose;

    double heading; // This is NOT the robots rotation, this is its direction of travel. Robot's rotation is stored in the pose
    bool reversed;

    std::string state;
    std::string value;
    std::vector<std::string> values;

    int duration_ms;

    ValorAutoAction(std::string line);

    std::string name;
    
    double vel;
    double maxAccel;
    double accelMultiplier;

    bool vision;
    
public:
    static std::vector<std::string> parseCSVLine(std::string);
};

#endif