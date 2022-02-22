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


    frc::Pose2d startPose = frc::Pose2d(8.514_m, 1.771_m, frc::Rotation2d(182.1_deg));
    frc::Pose2d bugs = frc::Pose2d(7_m, 0.05_m, frc::Rotation2d(185_deg));
    frc::Pose2d daffy = frc::Pose2d(4_m, 2.1_m, frc::Rotation2d(90_deg));
    frc::Pose2d predaffy = frc::Pose2d(5.083_m, 2.5_m, frc::Rotation2d(155_deg));
    frc::Pose2d porky = frc::Pose2d(0.1_m, 1.2_m, frc::Rotation2d(200_deg));
    frc::Pose2d shoot = frc::Pose2d(7_m, 1.2_m, frc::Rotation2d(100_deg));
    frc::Pose2d x6y4 = frc::Pose2d(6_m, 4_m, frc::Rotation2d(180_deg));

    frc::Translation2d preBugs{startPose.X(), bugs.Translation().Y()};
    // frc::Rotation2d gyroOffsetAuto = drivetrain->getGyroOffset() + frc::Rotation2d(180_deg);

    auto move2 = frc::TrajectoryGenerator::GenerateTrajectory(
        startPose,
        {},
        x6y4,
        reverseConfig);

    frc2::InstantCommand cmd_shooterPrime = frc2::InstantCommand( [&] {
        shooter->state.flywheelState = Shooter::FlywheelState::FLYWHEEL_PRIME; 
        shooter->state.turretState = Shooter::TurretState::TURRET_TRACK;
        shooter->state.hoodState = Shooter::HoodState::HOOD_TRACK;
    } );

    frc2::InstantCommand cmd_shooterDefault = frc2::InstantCommand( [&] {
        shooter->state.flywheelState = Shooter::FlywheelState::FLYWHEEL_DEFAULT; 
        shooter->state.turretState = Shooter::TurretState::TURRET_DISABLE;
        shooter->state.hoodState = Shooter::HoodState::HOOD_DOWN;
    } );

    frc2::InstantCommand cmd_intakeAuto = frc2::InstantCommand( [&] { feeder->state.feederState = Feeder::FeederState::FEEDER_INTAKE; } );

    frc2::InstantCommand cmd_intakeShoot = frc2::InstantCommand( [&] { feeder->state.feederState = Feeder::FeederState::FEEDER_SHOOT; } );

    frc2::InstantCommand cmd_disable = frc2::InstantCommand( [&] { 
        feeder->state.feederState = Feeder::FeederState::FEEDER_DISABLE;
        shooter->state.flywheelState = Shooter::FlywheelState::FLYWHEEL_DISABLE; 
        shooter->state.turretState = Shooter::TurretState::TURRET_DISABLE;
        shooter->state.hoodState = Shooter::HoodState::HOOD_DOWN;
    });

    // frc2::InstantCommand cmd_set_gyroOffset = frc2::InstantCommand( [&] {
    //     frc::Rotation2d gyroOffsetAuto = drivetrain->getGyroOffset() + frc::Rotation2d(90_deg);
    //     drivetrain->setGyroOffset(frc::Rotation2d(gyroOffsetAuto));
    //     });

    frc2::InstantCommand cmd_set_odometry = frc2::InstantCommand( [&] {
        drivetrain->resetOdometry(startPose);
    });
    
    auto moveBugs = frc::TrajectoryGenerator::GenerateTrajectory(
        startPose,
        {preBugs},
        bugs,
        config);

    auto movePorky = frc::TrajectoryGenerator::GenerateTrajectory(
        bugs,
        {predaffy.Translation()},
        porky,
        config);

    auto movePorkyFromDaffy = frc::TrajectoryGenerator::GenerateTrajectory(
        daffy,
        {},
        porky,
        config);

    auto moveDaffy = frc::TrajectoryGenerator::GenerateTrajectory(
        bugs,
        {},
        daffy,
        config);

    auto moveShoot = frc::TrajectoryGenerator::GenerateTrajectory(
        porky,
        {},
        shoot,
        reverseConfig);

     auto moveTest = frc::TrajectoryGenerator::GenerateTrajectory(
        frc::Pose2d(0_m, 0_m, frc::Rotation2d(0_deg)),
        {},
        frc::Pose2d(2_m, 0_m, frc::Rotation2d(0_deg)),
        config);
    
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

    frc2::SwerveControllerCommand<4> cmd_move_test(
        moveTest,
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

    frc2::SequentialCommandGroup *testCommands = new frc2::SequentialCommandGroup();
    testCommands->AddCommands
    (cmd_set_odometry,
    frc2::WaitCommand((units::second_t).5),
    cmd_intakeAuto,
    frc2::WaitCommand((units::second_t).5),
    cmd_shooterPrime,
    frc2::WaitCommand((units::second_t).5),
    cmd_intakeShoot,
    frc2::WaitCommand((units::second_t).5),
    cmd_shooterDefault,
    frc2::WaitCommand((units::second_t).5),
    cmd_move_test
    );

    frc2::SequentialCommandGroup *shoot4 = new frc2::SequentialCommandGroup();
    shoot4->AddCommands
    (cmd_set_odometry,
    cmd_intakeAuto,
    cmd_shooterPrime,
    cmd_move_moveBugs,
    frc2::WaitCommand((units::second_t).25),
    cmd_intakeShoot,
    frc2::WaitCommand((units::second_t)1.5),
    cmd_shooterDefault,
    cmd_intakeAuto,
    cmd_move_movePorky,
    cmd_shooterPrime,
    cmd_move_moveShoot,
    frc2::WaitCommand((units::second_t).5),
    cmd_intakeShoot
    );

    frc2::SequentialCommandGroup *intakeTest = new frc2::SequentialCommandGroup();
    intakeTest->AddCommands
    (cmd_intakeAuto,
    frc2::WaitCommand((units::second_t)5),
    cmd_disable,
    frc2::WaitCommand((units::second_t)1),
    cmd_intakeShoot,
    frc2::WaitCommand((units::second_t)2.5));

    frc2::SequentialCommandGroup *shoot5 = new frc2::SequentialCommandGroup();
    shoot5->AddCommands
    (cmd_set_odometry,
    cmd_intakeAuto,
    cmd_move_moveBugs,
    cmd_shooterPrime,
    frc2::WaitCommand((units::second_t).25),
    cmd_intakeShoot,
    frc2::WaitCommand((units::second_t)1.5),
    cmd_intakeAuto,
    cmd_move_moveDaffy,
    cmd_shooterPrime,
    frc2::WaitCommand((units::second_t).25),
    cmd_intakeShoot,
    frc2::WaitCommand((units::second_t)1.5),
    cmd_shooterDefault,
    cmd_move_movePorkyFromDaffy,
    frc2::WaitCommand((units::second_t)1.5),
    cmd_move_moveShoot,
    cmd_shooterPrime,
    frc2::WaitCommand((units::second_t).25),
    cmd_intakeShoot
    );
     
    frc2::SequentialCommandGroup *move2Offset = new frc2::SequentialCommandGroup();
    move2Offset->AddCommands
    (cmd_set_odometry,
    cmd_move_move2); 

    m_chooser.SetDefaultOption("4 ball auto", shoot4);
    m_chooser.AddOption("5 ball auto", shoot5);
    m_chooser.AddOption("Move 2 in x Offset direction", move2Offset);
    m_chooser.AddOption("Intake Testing", intakeTest);
    m_chooser.AddOption("All command check", testCommands);
    frc::SmartDashboard::PutData(&m_chooser);
  
    //potential issues
    //turret can't see the hub, might need to set position manually
    //hood and flywheel need to be at different speeds depending on location
    //might run out of time
}

frc2::Command* ValorAuto::getCurrentAuto() {
    return m_chooser.GetSelected();
}