/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "ValorSubsystem.h"

ValorSubsystem::ValorSubsystem() {
    robotMode = RobotMode::DISABLED;
    init();
    resetState();
}

void ValorSubsystem::Periodic() {
    if (robotMode == RobotMode::TELEOP)
        assessInputs();

    analyzeDashboard();
    
    if (robotMode != RobotMode::DISABLED)
        assignOutputs();
}

ValorSubsystem& ValorSubsystem::GetInstance() {
    static ValorSubsystem instance;
    return instance;
}

void ValorSubsystem::init() {
    // init subsystem
}

void ValorSubsystem::analyzeDashboard() {
    // Analyze dashboard
}

void ValorSubsystem::assessInputs() {
    // Assess inputs and assign states
}

void ValorSubsystem::assignOutputs() {
    // Assess states and assign outputs
}

void ValorSubsystem::resetState() {
    // reset state
}

void ValorSubsystem::initTable(char* name) {
    table = nt::NetworkTableInstance::GetDefault().GetTable(name);
}
