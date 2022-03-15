#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/DifferentialDriveWheelSpeeds.h>
#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc/kinematics/DifferentialDriveKinematics.h>
#include <frc/trajectory/constraint/DifferentialDriveVoltageConstraint.h>
#include <frc/trajectory/Trajectory.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include <frc2/command/SwerveControllerCommand.h>
#include <frc2/command/Command.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/WaitCommand.h>

#include "subsystems/Drivetrain.h"
#include "subsystems/Shooter.h"
#include "subsystems/Feeder.h"

#ifndef VALOR_AUTO_H
#define VALOR_AUTO_H

class ValorAuto {
    public:
        ValorAuto(Drivetrain *_drivetrain, Shooter *_shooter, Feeder *_feeder);

        frc2::Command* getCurrentAuto();

    private:
        Drivetrain *drivetrain;
        Shooter *shooter;
        Feeder *feeder;
        frc::SendableChooser<frc2::Command*> m_chooser;
        
};

#endif