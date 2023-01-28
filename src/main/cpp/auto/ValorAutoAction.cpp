#include "ValorAutoAction.h"

// Split a string by commas, spaces not accounted for
std::vector<std::string> ValorAutoAction::parseCSVLine(std::string line)
{
    int pointerPos = 0;
    std::vector<std::string> items;

    while (pointerPos >= 0 && pointerPos < line.length()) {
        int returnPos = line.find_first_of(",", pointerPos);
        if (returnPos != std::string::npos) {
            items.push_back(line.substr(pointerPos,returnPos-pointerPos));
            pointerPos = returnPos + 1;
        } else {
            items.push_back(line.substr(pointerPos));
            pointerPos = -1;
        }
    }
    return items;
}

frc::Pose2d ValorAutoAction::getPose(frc::Translation2d position, double angle)
{
    return frc::Pose2d(position, frc::Rotation2d(units::degree_t{angle}));
}

ValorAutoAction::ValorAutoAction(std::string line, std::map<std::string, frc::Translation2d> * points)
{
    std::vector<std::string> items = parseCSVLine(line);
    error = ValorAutoAction::Error::NONE_ERROR;
    
    if (items.empty()) {
        type = ValorAutoAction::Type::NONE;
        return;
    } else if (items[0] == "time") {
        type = ValorAutoAction::Type::TIME;
    } else if (items[0] == "state") {
        type = ValorAutoAction::Type::STATE;
    } else if (items[0] == "trajectory") {
        type = ValorAutoAction::Type::TRAJECTORY;
    } else if (items[0] == "reset_odom") {
        type = ValorAutoAction::Type::RESET_ODOM;
    } else if (items[0] == "action"){
        type = ValorAutoAction::Type::ACTION;
    }

    if (type == ValorAutoAction::Type::TIME) {
        if (items.size() < 2) {
            error = ValorAutoAction::Error::SIZE_MISMATCH;
            return;
        }
        duration_ms = atoi(items[1].c_str());
        return;
    }
    else if (type == ValorAutoAction::Type::STATE) {
        if (items.size() < 3) {
            error = ValorAutoAction::Error::SIZE_MISMATCH;
            return;
        }
        state = items[1];
        value = items[2];
    }
    else if (type == ValorAutoAction::Type::TRAJECTORY) {
        if (items.size() < 4) {
            error = ValorAutoAction::Error::SIZE_MISMATCH;
            return;
        }

        if (points->count(items[1]) == 0 || points->count(items[2]) == 0) {
            error = ValorAutoAction::Error::POINT_MISSING;
            return;
        }

        auto _start = points->at(items[1]);
        auto _end = points->at(items[2]);

        start = getPose(_start, atoi(items[3].c_str()));
        end = getPose(_end, atoi(items[3].c_str()));

        reversed = false;

        if (items.size() == 5){
            if (items[4] == "reversed" || items[4] == "reverse")
                reversed = true;
        }
    }
    else if (type == ValorAutoAction::Type::RESET_ODOM){
        if (items.size() < 3){
            error = ValorAutoAction::Error::SIZE_MISMATCH;
            return;
        }
        auto _start = points->at(items[1]);
        start = getPose(_start, atoi(items[2].c_str()));
    }
    else if (type == ValorAutoAction::Type::ACTION){
        if (items.size() < 2){
            error = ValorAutoAction::Error::SIZE_MISMATCH;
        }
        name = items[1];
        // Code to load commands into the action is handled in ValorAuto
    }
}