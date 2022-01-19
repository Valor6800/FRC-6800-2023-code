#include "ValorAuto.h"

ValorAuto::ValorAuto(Drivetrain *_drivetrain) : drivetrain(_drivetrain)
{    
    // See: https://github.com/wpilibsuite/allwpilib/blob/v2022.1.1/wpilibcExamples/src/main/cpp/examples/SwerveControllerCommand/cpp/RobotContainer.cpp

    frc::TrajectoryConfig config(units::velocity::meters_per_second_t{SwerveConstants::AUTO_MAX_SPEED_MPS},
                                 units::acceleration::meters_per_second_squared_t{SwerveConstants::AUTO_MAX_ACCEL_MPSS});

    config.SetKinematics(drivetrain->getKinematics());

    frc::TrajectoryConfig reverseConfig(units::velocity::meters_per_second_t{SwerveConstants::AUTO_MAX_SPEED_MPS},
                                    units::acceleration::meters_per_second_squared_t{SwerveConstants::AUTO_MAX_ACCEL_MPSS});

    reverseConfig.SetKinematics(drivetrain->getKinematics());
    reverseConfig.SetReversed(true);

    frc::ProfiledPIDController<units::radians> thetaController{
            DriveConstants::KPT,
            DriveConstants::KIT,
            DriveConstants::KDT,
            frc::ProfiledPIDController<units::radians>::Constraints(
                units::angular_velocity::radians_per_second_t{SwerveConstants::AUTO_MAX_ROTATION_RPS},
                units::angular_acceleration::radians_per_second_squared_t{SwerveConstants::AUTO_MAX_ROTATION_ACCEL_RPSS})
    };

    // @TODO look at angle wrapping and modding
    thetaController.EnableContinuousInput(units::radian_t(-wpi::numbers::pi),
                                        units::radian_t(wpi::numbers::pi));

    auto move1 = frc::TrajectoryGenerator::GenerateTrajectory(
        frc::Pose2d(0_m, 0_m, frc::Rotation2d(0_deg)),
        {},
        frc::Pose2d(1.234_m, 0_m, frc::Rotation2d(0_deg)),
        config);

    auto move2 = frc::TrajectoryGenerator::GenerateTrajectory(
        frc::Pose2d(1.234_m, 0_m, frc::Rotation2d(0_deg)),
        {frc::Translation2d{.867_m, -1.25_m}},
        frc::Pose2d(0_m, -2.516_m, frc::Rotation2d(-122.24_deg)),
        reverseConfig);

    auto move3 = frc::TrajectoryGenerator::GenerateTrajectory(
        frc::Pose2d(0_m, -2.516_m, frc::Rotation2d(-122.24_deg)),
        {frc::Translation2d{-.2_m, -4_m}},
        frc::Pose2d(.2_m, -6_m, frc::Rotation2d(-65_deg)),
        reverseConfig);
    
    frc2::SwerveControllerCommand<4> cmd_move_move1(
        move1,
        [&] () { return drivetrain->getPose_m(); },
        drivetrain->getKinematics(),
        frc2::PIDController(DriveConstants::KPX, DriveConstants::KIX, DriveConstants::KDX),
        frc2::PIDController(DriveConstants::KPY, DriveConstants::KIY, DriveConstants::KDY),
        thetaController,
        [this] (auto states) { drivetrain->setModuleStates(states); },
        {drivetrain}
    );

    frc2::SwerveControllerCommand<4> cmd_move_move2(
        move2,
        [&] () { return drivetrain->getPose_m(); },
        drivetrain->getKinematics(),
        frc2::PIDController(DriveConstants::KPX, DriveConstants::KIX, DriveConstants::KDX),
        frc2::PIDController(DriveConstants::KPY, DriveConstants::KIY, DriveConstants::KDY),
        thetaController,
        [this] (auto states) { drivetrain->setModuleStates(states); },
        {drivetrain}
    );

    frc2::SwerveControllerCommand<4> cmd_move_move3(
        move3,
        [&] () { return drivetrain->getPose_m(); },
        drivetrain->getKinematics(),
        frc2::PIDController(DriveConstants::KPX, DriveConstants::KIX, DriveConstants::KDX),
        frc2::PIDController(DriveConstants::KPY, DriveConstants::KIY, DriveConstants::KDY),
        thetaController,
        [this] (auto states) { drivetrain->setModuleStates(states); },
        {drivetrain}
    );

auto moveBugs = frc::TrajectoryGenerator::GenerateTrajectory(
        frc::Pose2d(0_m, 0_m, frc::Rotation2d(0_deg)),
        {frc::Translation2d{0_m,1.234_m}},
        frc::Pose2d(0.7_m, 1.234_m, frc::Rotation2d(0_deg)),
        config);

    auto movePorky = frc::TrajectoryGenerator::GenerateTrajectory(
        frc::Pose2d(0.7_m, 1.234_m, frc::Rotation2d(0_deg)),
        {frc::Translation2d{2.0_m, 0.057_m}},
        frc::Pose2d(6_m, 0.19_m, frc::Rotation2d(20_deg)),
        config);
/*
    auto moveDaffy = frc::TrajectoryGenerator::GenerateTrajectory(
        frc::Pose2d(0_m, -2.516_m, frc::Rotation2d(-122.24_deg)),
        {frc::Translation2d{-.2_m, -4_m}},
        frc::Pose2d(.2_m, -6_m, frc::Rotation2d(-65_deg)),
        reverseConfig);
*/
    auto moveShoot = frc::TrajectoryGenerator::GenerateTrajectory(
        frc::Pose2d(6_m, 0.19_m, frc::Rotation2d(20_deg)),
        {},
        frc::Pose2d(2.626_m, 0.0_m, frc::Rotation2d(-90_deg)),
        reverseConfig);
    
    frc2::SwerveControllerCommand<4> cmd_move_moveBugs(
        moveBugs,
        [&] () { return drivetrain->getPose_m(); },
        drivetrain->getKinematics(),
        frc2::PIDController(DriveConstants::KPX, DriveConstants::KIX, DriveConstants::KDX),
        frc2::PIDController(DriveConstants::KPY, DriveConstants::KIY, DriveConstants::KDY),
        thetaController,
        [this] (auto states) { drivetrain->setModuleStates(states); },
        {drivetrain}
    );
/*
    frc2::SwerveControllerCommand<4> cmd_move_moveDaffy(
        moveDaffy,
        [&] () { return drivetrain->getPose_m(); },
        drivetrain->getKinematics(),
        frc2::PIDController(DriveConstants::KPX, DriveConstants::KIX, DriveConstants::KDX),
        frc2::PIDController(DriveConstants::KPY, DriveConstants::KIY, DriveConstants::KDY),
        thetaController,
        [this] (auto states) { drivetrain->setModuleStates(states); },
        {drivetrain}
    );
    */

    frc2::SwerveControllerCommand<4> cmd_move_movePorky(
        movePorky,
        [&] () { return drivetrain->getPose_m(); },
        drivetrain->getKinematics(),
        frc2::PIDController(DriveConstants::KPX, DriveConstants::KIX, DriveConstants::KDX),
        frc2::PIDController(DriveConstants::KPY, DriveConstants::KIY, DriveConstants::KDY),
        thetaController,
        [this] (auto states) { drivetrain->setModuleStates(states); },
        {drivetrain}
    );

    frc2::SwerveControllerCommand<4> cmd_move_moveShoot(
        moveShoot,
        [&] () { return drivetrain->getPose_m(); },
        drivetrain->getKinematics(),
        frc2::PIDController(DriveConstants::KPX, DriveConstants::KIX, DriveConstants::KDX),
        frc2::PIDController(DriveConstants::KPY, DriveConstants::KIY, DriveConstants::KDY),
        thetaController,
        [this] (auto states) { drivetrain->setModuleStates(states); },
        {drivetrain}
    );


    frc2::SequentialCommandGroup *shoot4 = new frc2::SequentialCommandGroup();
    shoot4->AddCommands
    (cmd_move_move1,
    cmd_move_move2,
    cmd_move_move3); 

     frc2::SequentialCommandGroup *shoot4New = new frc2::SequentialCommandGroup();
    shoot4New->AddCommands
    (cmd_move_moveBugs,
    frc2::WaitCommand((units::second_t)1.5),
    cmd_move_movePorky,
    cmd_move_moveShoot); 

    frc2::SequentialCommandGroup *leaveTarmac = new frc2::SequentialCommandGroup();
    leaveTarmac->AddCommands
    (cmd_move_move1);

    m_chooser.AddOption("basic movement auto", leaveTarmac);
    m_chooser.AddOption("4 ball auto", shoot4);
    m_chooser.SetDefaultOption("4 ball auto new", shoot4New);
    frc::SmartDashboard::PutData(&m_chooser);
}

frc2::Command* ValorAuto::getCurrentAuto() {
    return m_chooser.GetSelected();
    }