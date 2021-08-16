/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "subsystems/Drivetrain.h"

Drivetrain::Drivetrain() : ValorSubsystem(),
                           driverController(NULL) {
    frc2::CommandScheduler::GetInstance().RegisterSubsystem(this);
    init();
}

void Drivetrain::init() {
}

void Drivetrain::setController(frc::XboxController* controller) {
    driverController = controller;
}

void Drivetrain::setDefaultState() {
    resetState();
}

void Drivetrain::resetState() {

}

void Drivetrain::assessInputs() {
    if (!driverController) {
        return;
    }

}

void Drivetrain::analyzeDashboard() {
}

void Drivetrain::assignOutputs() {
}