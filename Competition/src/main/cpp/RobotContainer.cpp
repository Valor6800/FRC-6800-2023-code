/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "RobotContainer.h"

RobotContainer::RobotContainer() : m_auto(&m_drivetrain, &m_shooter) {
    ConfigureButtonBindings();
    m_shooter.setDrivetrain(&m_drivetrain);
}

void RobotContainer::ConfigureButtonBindings() {
    m_drivetrain.setController(&m_GamepadDriver);
}

frc2::Command* RobotContainer::GetAutonomousCommand() {
    return m_auto.getCurrentAuto();
}