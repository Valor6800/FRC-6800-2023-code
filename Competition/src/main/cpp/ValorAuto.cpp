#include "ValorAuto.h"

ValorAuto::ValorAuto(Drivetrain *_drivetrain) : drivetrain(_drivetrain)
{

    // See: https://github.com/wpilibsuite/allwpilib/blob/v2022.1.1/wpilibcExamples/src/main/cpp/examples/SwerveControllerCommand/cpp/RobotContainer.cpp

    frc::TrajectoryConfig config(units::velocity::meters_per_second_t{SwerveConstants::AUTO_MAX_SPEED_MPS},
                                 units::acceleration::meters_per_second_squared_t{SwerveConstants::AUTO_MAX_ACCEL_MPSS});

    config.SetKinematics(drivetrain->getKinematics());

    auto exampleTrajectory = frc::TrajectoryGenerator::GenerateTrajectory(
        frc::Pose2d(0_m, 0_m, frc::Rotation2d(0_deg)),
        {},
        frc::Pose2d(3_m, 0_m, frc::Rotation2d(0_deg)),
        config);

    frc::ProfiledPIDController<units::radians> thetaController{
            0,
            0,
            0,
            frc::ProfiledPIDController<units::radians>::Constraints(
                units::angular_velocity::radians_per_second_t{SwerveConstants::AUTO_MAX_ROTATION_RPS},
                units::angular_acceleration::radians_per_second_squared_t{SwerveConstants::AUTO_MAX_ROTATION_ACCEL_RPSS})
    };

    // @TODO look at angle wrapping and modding
    thetaController.EnableContinuousInput(units::radian_t(-wpi::numbers::pi),
                                        units::radian_t(wpi::numbers::pi));

    frc2::SwerveControllerCommand<4> cmd_move(
        exampleTrajectory,
        [&] () { return drivetrain->getPose_m(); },
        drivetrain->getKinematics(),
        frc2::PIDController(0,0,0),
        frc2::PIDController(0,0,0),
        thetaController,
        [this] (auto states) { drivetrain->setModuleStates(states); },
        {drivetrain}
    );

    frc2::SequentialCommandGroup *move = new frc2::SequentialCommandGroup();
    move->AddCommands(cmd_move);
    autos["Move"] = move;
}

frc2::Command* ValorAuto::getCurrentAuto() {
    std::string auto_name = "Move";
    // @TODO read auto name from driver station
    auto selected_trajectories = autos[auto_name];
    return selected_trajectories;
}