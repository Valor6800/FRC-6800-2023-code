/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc/TimedRobot.h>
#include <frc2/command/Command.h>

#include "Constants.h"
#include "ValorGamepad.h"

#include "subsystems/Drivetrain.h"
#include "subsystems/Elevarm.h"
#include "subsystems/Intake.h"

#include <frc/DriverStation.h>
#include <frc/DataLogManager.h>

#include <frc/livewindow/LiveWindow.h>

#include "auto/ValorAuto.h"

#include <fstream>

class Robot : public frc::TimedRobot {
    public:
        Robot();

        void RobotInit() override;
        void RobotPeriodic() override;
        void DisabledInit() override;
        void DisabledPeriodic() override;
        void AutonomousInit() override;
        void AutonomousPeriodic() override;
        void TeleopInit() override;
        void TeleopPeriodic() override;
        void TestPeriodic() override;
        void AutonomousExit() override;
        
    private:
        ValorGamepad gamepadOperator{OIConstants::GAMEPAD_OPERATOR_LOCATION};
        ValorGamepad gamepadDriver{OIConstants::GAMEPAD_BASE_LOCATION};

        frc2::Command* autoCommand = nullptr;

        Drivetrain drivetrain;
        Intake intake;
        Elevarm elevarm;
        ValorAuto autonomous;

        std::ofstream outfile;
};
