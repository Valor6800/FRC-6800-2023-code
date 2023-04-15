#include "auto/ValorAutoAction.h"
#include <units/length.h>
#include <units/angle.h>

#define TRANS_VELOCITY 4.952f

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

ValorAutoAction::ValorAutoAction(std::string line)
{
    std::vector<std::string> items = parseCSVLine(line);
    error = ValorAutoAction::Error::NONE_ERROR;
    error_message = "";
    
    if (items.empty() || (!items.empty() && items[0].starts_with("//"))) {
        type = ValorAutoAction::Type::NONE;
        return;
    } else if (items[0] == "time") {
        type = ValorAutoAction::Type::TIME;
    } else if (items[0] == "state") {
        type = ValorAutoAction::Type::STATE;
    } else if (items[0] == "reset_odom") {
        type = ValorAutoAction::Type::RESET_ODOM;
    } else if (items[0] == "xmode" || items[0] == "XMode"){
        type = ValorAutoAction::Type::XMODE;
    } else if (items[0] == "elevarm"){
        type = ValorAutoAction::ELEVARM;
    } else if (items[0] == "acceleration" || items[0] == "accel") {
        type = ValorAutoAction::Type::ACCELERATION;
    } else if (items[0] == "balance") {
        type = ValorAutoAction::BALANCE;
    } else if (items[0] == "intake") {
        type = ValorAutoAction::INTAKE;
    } else if (items[0] == "climb_over"){
        type = ValorAutoAction::CLIMB_OVER;
    } else if (items[0] == "goto"){
        type = ValorAutoAction::GO_TO;
    } else {
        error = ValorAutoAction::COMMAND_MISSING;
        error_message = items[0];
        return;
    }

    if (type == ValorAutoAction::Type::TIME) {
        if (items.size() < 2) {
            error = ValorAutoAction::Error::SIZE_MISMATCH;
            error_message = "received " + std::to_string(items.size());
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
    else if (type == ValorAutoAction::Type::RESET_ODOM){
        if (items.size() < 2){
            error = ValorAutoAction::Error::SIZE_MISMATCH;
            error_message = "received " + std::to_string(items.size());
            return;
        }

        vision = false;
        if (items[1] == "\"vision\""){
            vision = true;
            return;
        }

        if (items.size() < 3){
            error = ValorAutoAction::Error::SIZE_MISMATCH;
            error_message = "received " + std::to_string(items.size());
            return;
        }

        //@TODO FIX THIS
        // change to output just a pose then reset to that pose in valorAuto.cpp
        //now only needed for climbing the charging station
        // now only for mid auto changes

        pose = frc::Pose2d(units::length::meter_t{std::stod(items[1])}, units::length::meter_t{std::stod(items[2])}, units::degree_t{std::stod(items[3])});
        
        // if (points->contains(items[1])){
        //     auto _start = points->at(items[1]);
        //     start = getPose(_start, atoi(items[2].c_str()));
        //     used_vals = {{"x", true}, {"y", true}, {"angle", true}};
        // } else if (items[1] == "\"x\""){
        //     start = frc::Pose2d((units::length::meter_t)stod(items[2]), 0_m, 0_deg);
        //     used_vals = {{"x", true}, {"y", false}, {"angle", false}};
        // } else if (items[1] == "\"y\""){
        //     start = frc::Pose2d(0_m, (units::length::meter_t)stod(items[2]), 0_deg);
        //     used_vals = {{"x", false}, {"y", true}, {"angle", false}};
        // } else if (items[1] == "\"angle\""){
        //     start = frc::Pose2d(0_m, 0_m, (units::degree_t)stod(items[2]));
        //     used_vals = {{"x", false}, {"y", false}, {"angle", true}};
        // } else {
        //     error = ValorAutoAction::Error::POINT_MISSING;
        //     error_message = items[1];
        //     return;
        // }
        
    }
    else if (type == ValorAutoAction::ELEVARM){
        if (items.size() < 4){
            error = ValorAutoAction::SIZE_MISMATCH;
            error_message = "received " + std::to_string(items.size());
            return;
        }
            
        values = {items[1], items[2], items[3]};
    } else if (type == ValorAutoAction::Type::ACCELERATION) {
        if (items.size() == 2){
            maxAccel = stod(items[1]);
        }
        else
            maxAccel = NULL;
        
        if (items.size() == 3 && items[2] == "multiplier")
            accelMultiplier = stod(items[1]);
        else
            accelMultiplier = 1.0;
    } else if (type == ValorAutoAction::BALANCE){
        reversed = false;
        oldBalance = false;
        if (items.size() > 1) {
            reversed = items[1] == "reversed";
            if (items.size() > 2){
                oldBalance = items[2] == "old";
                vision = items[2] == "vision";
            }
        }
    } else if (type == ValorAutoAction::Type::INTAKE) {
        if (items.size() < 2){
            error + ValorAutoAction::SIZE_MISMATCH;
            error_message = "received " + std::to_string(items.size());
            return;
        }

        if (items[1] == "disabled") {
            values = {items[1], "none"};
        } else {
           values = {items[1], items[2]}; 
        }
        
    }
    else if (type == ValorAutoAction::Type::CLIMB_OVER) {
        
    } else if (type == ValorAutoAction::Type::GO_TO) {
        if (items.size() < 3){
            error = ValorAutoAction::SIZE_MISMATCH;
            error_message = "received " + std::to_string(items.size());
            return;
        }

        //@TODO do something else cause this will be totally differnt lol

        // if (!points->contains(items[1])){
        //     error = ValorAutoAction::Error::POINT_MISSING;
        //     error_message = items[1];
        //     return;
        // }
        // end = getPose(points->at(items[1]), std::stod(items[2]));
        // if (items.size() >= 4 && items[3] == "reverse" || items[3] == "reversed")
        //     reversed = true;
    }
}

frc::Pose2d ValorAutoAction::getPose() {
    return pose;
}