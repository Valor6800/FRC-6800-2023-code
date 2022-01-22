#include "ValorAuto.h"

ValorAuto::ValorAuto(Drivetrain *_drivetrain, Shooter *_shooter, Feeder *_feeder) : 
    drivetrain(_drivetrain), 
    shooter(_shooter),
    feeder(_feeder)
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

frc::Pose2d bugs = frc::Pose2d(0.7_m, 1.2_m, frc::Rotation2d(0_deg));
frc::Pose2d daffy = frc::Pose2d(3.0_m,0_m, frc::Rotation2d(-45_deg));
frc::Pose2d porky = frc::Pose2d(6_m, 0.19_m, frc::Rotation2d(20_deg));
frc::Pose2d shoot = frc::Pose2d(2.626_m,0_m, frc::Rotation2d(-90_deg));

frc::Pose2d tazAlt = frc::Pose2d(1.2_m, 0.75_m, frc::Rotation2d(0_deg));
frc::Pose2d vAlt = frc::Pose2d(.4_m, 0_m, frc::Rotation2d(-90_deg));
frc::Pose2d bugsAlt = frc::Pose2d(1.2_m, -0.7_m, frc::Rotation2d(-90_deg));
frc::Pose2d daffyAlt = frc::Pose2d(0_m, -3.0_m, frc::Rotation2d(-90_deg)); //-140_deg
frc::Pose2d porkyAlt = frc::Pose2d(.19_m, -6_m, frc::Rotation2d(-60_deg));
frc::Pose2d shootAlt = frc::Pose2d(0_m,-2.626_m, frc::Rotation2d(-180_deg));

frc::Pose2d x2y0 = frc::Pose2d(2_m, 0_m, frc::Rotation2d(0_deg));
// frc::Rotation2d gyroOffsetAuto = drivetrain->getGyroOffset() + frc::Rotation2d(180_deg);


    auto move2 = frc::TrajectoryGenerator::GenerateTrajectory(
        frc::Pose2d(0_m, 0_m, frc::Rotation2d(0_deg)),
        {},
        x2y0,
        config);

frc2::InstantCommand cmd_intake2 = frc2::InstantCommand( [&] { feeder->state.feederState = Feeder::FeederState::FEEDER_INTAKE2; } );

    auto moveBugs = frc::TrajectoryGenerator::GenerateTrajectory(
        frc::Pose2d(0_m, 0_m, frc::Rotation2d(0_deg)),
        {frc::Translation2d{0_m, 1.2_m}},
        bugs,
        config);

    auto movePorky = frc::TrajectoryGenerator::GenerateTrajectory(
        bugs,
        {frc::Translation2d{2.0_m, 0.057_m}},
        porky,
        config);

    auto movePorkyFromDaffy = frc::TrajectoryGenerator::GenerateTrajectory(
        daffy,
        {},
        porky,
        config);

    auto moveDaffy = frc::TrajectoryGenerator::GenerateTrajectory(
        bugs,
        {frc::Translation2d{2_m, 0.5_m}},
        daffy,
        config);

    auto moveShoot = frc::TrajectoryGenerator::GenerateTrajectory(
        porky,
        {},
        shoot,
        reverseConfig);
    
    auto moveTazAlt = frc::TrajectoryGenerator::GenerateTrajectory(
        frc::Pose2d(0_m, 0_m, frc::Rotation2d(0_deg)),
        {},
        tazAlt,
        config);
    
    auto moveVAlt = frc::TrajectoryGenerator::GenerateTrajectory(
        tazAlt,
        {},
        vAlt,
        reverseConfig);

    auto moveBugsAlt = frc::TrajectoryGenerator::GenerateTrajectory(
        tazAlt,
        {},
        bugsAlt,
        config);

    auto moveDaffyAlt = frc::TrajectoryGenerator::GenerateTrajectory(
        vAlt,
        {frc::Translation2d{1.2_m, 0_m}, frc::Translation2d{1.2_m, -0.7_m}},
        daffyAlt,
        config);

    auto movePorkyFromDaffyAlt = frc::TrajectoryGenerator::GenerateTrajectory(
        daffyAlt,
        {},
        porkyAlt,
        config);

    auto moveShootAlt = frc::TrajectoryGenerator::GenerateTrajectory(
        porkyAlt,
        {},
        shootAlt,
        reverseConfig);

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

    frc2::InstantCommand cmd_set_gyroOffset = frc2::InstantCommand( [&] {
        frc::Rotation2d gyroOffsetAuto = drivetrain->getGyroOffset() + frc::Rotation2d(90_deg);
        drivetrain->setGyroOffset(frc::Rotation2d(gyroOffsetAuto));
        });

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
    frc2::SwerveControllerCommand<4> cmd_move_movePorkyFromDaffy(
        movePorkyFromDaffy,
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

    
    frc2::SwerveControllerCommand<4> cmd_move_moveTazAlt(
        moveTazAlt,
        [&] () { return drivetrain->getPose_m(); },
        drivetrain->getKinematics(),
        frc2::PIDController(DriveConstants::KPX, DriveConstants::KIX, DriveConstants::KDX),
        frc2::PIDController(DriveConstants::KPY, DriveConstants::KIY, DriveConstants::KDY),
        thetaController,
        [this] (auto states) { drivetrain->setModuleStates(states); },
        {drivetrain}
    );

    frc2::SwerveControllerCommand<4> cmd_move_moveVAlt(
        moveVAlt,
        [&] () { return drivetrain->getPose_m(); },
        drivetrain->getKinematics(),
        frc2::PIDController(DriveConstants::KPX, DriveConstants::KIX, DriveConstants::KDX),
        frc2::PIDController(DriveConstants::KPY, DriveConstants::KIY, DriveConstants::KDY),
        thetaController,
        [this] (auto states) { drivetrain->setModuleStates(states); },
        {drivetrain}
    );
    frc2::SwerveControllerCommand<4> cmd_move_moveBugsAlt(
        moveBugsAlt,
        [&] () { return drivetrain->getPose_m(); },
        drivetrain->getKinematics(),
        frc2::PIDController(DriveConstants::KPX, DriveConstants::KIX, DriveConstants::KDX),
        frc2::PIDController(DriveConstants::KPY, DriveConstants::KIY, DriveConstants::KDY),
        thetaController,
        [this] (auto states) { drivetrain->setModuleStates(states); },
        {drivetrain}
    );

    frc2::SwerveControllerCommand<4> cmd_move_moveDaffyAlt(
        moveDaffyAlt,
        [&] () { return drivetrain->getPose_m(); },
        drivetrain->getKinematics(),
        frc2::PIDController(DriveConstants::KPX, DriveConstants::KIX, DriveConstants::KDX),
        frc2::PIDController(DriveConstants::KPY, DriveConstants::KIY, DriveConstants::KDY),
        thetaController,
        [this] (auto states) { drivetrain->setModuleStates(states); },
        {drivetrain}
    );

    frc2::SwerveControllerCommand<4> cmd_move_movePorkyAlt(
        movePorkyFromDaffyAlt,
        [&] () { return drivetrain->getPose_m(); },
        drivetrain->getKinematics(),
        frc2::PIDController(DriveConstants::KPX, DriveConstants::KIX, DriveConstants::KDX),
        frc2::PIDController(DriveConstants::KPY, DriveConstants::KIY, DriveConstants::KDY),
        thetaController,
        [this] (auto states) { drivetrain->setModuleStates(states); },
        {drivetrain}
    );

    frc2::SwerveControllerCommand<4> cmd_move_moveShootAlt(
        moveShootAlt,
        [&] () { return drivetrain->getPose_m(); },
        drivetrain->getKinematics(),
        frc2::PIDController(DriveConstants::KPX, DriveConstants::KIX, DriveConstants::KDX),
        frc2::PIDController(DriveConstants::KPY, DriveConstants::KIY, DriveConstants::KDY),
        thetaController,
        [this] (auto states) { drivetrain->setModuleStates(states); },
        {drivetrain}
    );

/*
    frc2::SequentialCommandGroup *shoot4 = new frc2::SequentialCommandGroup();
    shoot4->AddCommands
    (cmd_move_move1,
    cmd_move_move2,
    cmd_move_move3); */

    frc2::SequentialCommandGroup *shoot4New = new frc2::SequentialCommandGroup();
    shoot4New->AddCommands
    (cmd_intake2,
    cmd_move_moveBugs,
    frc2::WaitCommand((units::second_t)1.5),
    cmd_move_movePorky,
    cmd_move_moveShoot);

    // frc2::SequentialCommandGroup *motorTest = new frc2::SequentialCommandGroup();
    // shoot4New->AddCommands
    // (manualTurret,
    // frc2::WaitCommand((units::second_t)1.5),
    // cmd_move_moveBugs);

    frc2::SequentialCommandGroup *shoot5 = new frc2::SequentialCommandGroup();
    shoot5->AddCommands
    (cmd_move_moveBugs,
    frc2::WaitCommand((units::second_t)1.5),
    cmd_move_moveDaffy,
    frc2::WaitCommand((units::second_t)1.5),
    cmd_move_movePorkyFromDaffy,
    frc2::WaitCommand((units::second_t)1.5),
    cmd_move_moveShoot);  

    frc2::SequentialCommandGroup *shoot5RemoveTaz = new frc2::SequentialCommandGroup();
    shoot5RemoveTaz->AddCommands
    (cmd_move_moveTazAlt,
    cmd_move_moveVAlt,
    frc2::WaitCommand((units::second_t)1.5),
    cmd_move_moveDaffyAlt,
    frc2::WaitCommand((units::second_t)1.5),
    cmd_move_movePorkyAlt,
    frc2::WaitCommand((units::second_t)1.5),
    cmd_move_moveShootAlt); 

    frc2::SequentialCommandGroup *move2Offset = new frc2::SequentialCommandGroup();
    move2Offset->AddCommands
    (cmd_set_gyroOffset,
    cmd_move_move2); 

    frc2::SequentialCommandGroup *move2x = new frc2::SequentialCommandGroup();
    move2x->AddCommands
    (cmd_move_move2); 

    m_chooser.SetDefaultOption("4 ball auto new", shoot4New);
    m_chooser.SetDefaultOption("5 ball auto", shoot5);
    m_chooser.SetDefaultOption("5 ball auto remove Taz", shoot5RemoveTaz);
    m_chooser.AddOption("Move 2 in x direction", move2x);
    m_chooser.AddOption("Move 2 in x Offset direction", move2Offset);
    //m_chooser.AddOption("motor test", motorTest);

    frc::SmartDashboard::PutData(&m_chooser);
  
}

frc2::Command* ValorAuto::getCurrentAuto() {
    return m_chooser.GetSelected();
    }