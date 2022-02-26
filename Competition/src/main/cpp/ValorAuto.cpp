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


    frc::Pose2d startPose = frc::Pose2d(7_m, 1.771_m, frc::Rotation2d(-92.1_deg));
    //Bugs y blue .35
    frc::Pose2d bugsBlue = frc::Pose2d(7_m, 0.35_m, frc::Rotation2d(-90_deg));
    frc::Pose2d bugsRed = frc::Pose2d(7_m, 0.5_m, frc::Rotation2d(-90_deg));
    //Daffy y was 1.6
    frc::Pose2d daffyBlue = frc::Pose2d(3.8_m, 1.4_m, frc::Rotation2d(90_deg));
    frc::Pose2d daffyRed = frc::Pose2d(3.8_m, 1.4_m, frc::Rotation2d(90_deg));
    
    frc::Pose2d predaffyBlue = frc::Pose2d(5.083_m, 1.5_m, frc::Rotation2d(95_deg));
    frc::Pose2d predaffyRed = frc::Pose2d(5.083_m, 1.5_m, frc::Rotation2d(95_deg));
    
    //shifting each movement by .5 to avoid smacking into the pipes
    frc::Pose2d porkyBlue = frc::Pose2d(0.6_m, 1.7_m, frc::Rotation2d(200_deg));
    frc::Pose2d porkyRed = frc::Pose2d(0.6_m, 1.7_m, frc::Rotation2d(200_deg));

    frc::Pose2d shootBlue = frc::Pose2d(7_m, 1.2_m, frc::Rotation2d(100_deg));
    frc::Pose2d shootRed = frc::Pose2d(7_m, 1.2_m, frc::Rotation2d(100_deg));
    //frc::Translation2d preBugs{startPose.X(), bugs.Translation().Y()};
    // frc::Rotation2d gyroOffsetAuto = drivetrain->getGyroOffset() + frc::Rotation2d(180_deg);

    frc2::InstantCommand cmd_shooterPrime = frc2::InstantCommand( [&] {
        shooter->state.flywheelState = Shooter::FlywheelState::FLYWHEEL_PRIME; 
        shooter->state.hoodState = Shooter::HoodState::HOOD_UP;
    } );

    frc2::InstantCommand cmd_shooterFar = frc2::InstantCommand( [&] {
        shooter->state.flywheelState = Shooter::FlywheelState::FLYWHEEL_AUTO; 
        shooter->state.hoodState = Shooter::HoodState::HOOD_UP;
    } );

    frc2::InstantCommand cmd_turretTrack = frc2::InstantCommand( [&] {
        shooter->state.turretState = Shooter::TurretState::TURRET_TRACK;
    } );

    frc2::InstantCommand cmd_shooterDefault = frc2::InstantCommand( [&] {
        shooter->state.flywheelState = Shooter::FlywheelState::FLYWHEEL_DEFAULT; 
        shooter->state.turretState = Shooter::TurretState::TURRET_DISABLE;
        shooter->state.hoodState = Shooter::HoodState::HOOD_DOWN;
    } );

    frc2::InstantCommand cmd_intakeOne = frc2::InstantCommand( [&] { feeder->state.feederState = Feeder::FeederState::FEEDER_AUTO; } );


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

    
    auto moveBugsBlue = frc::TrajectoryGenerator::GenerateTrajectory(
        startPose,
        {},//{preBugs},
        bugsBlue,
        config);

    auto moveBugsRed = frc::TrajectoryGenerator::GenerateTrajectory(
        startPose,
        {},//{preBugs},
        bugsRed,
        config);

    auto movePorkyBlue = frc::TrajectoryGenerator::GenerateTrajectory(
        bugsBlue,
        {predaffyBlue.Translation()},
        porkyBlue,
        config);
    
    auto movePorkyRed = frc::TrajectoryGenerator::GenerateTrajectory(
        bugsRed,
        {predaffyRed.Translation()},
        porkyRed,
        config);

    auto movePorkyFromDaffyBlue = frc::TrajectoryGenerator::GenerateTrajectory(
        daffyBlue,
        {},
        porkyBlue,
        config);
    auto movePorkyFromDaffyRed = frc::TrajectoryGenerator::GenerateTrajectory(
        daffyRed,
        {},
        porkyRed,
        config);

    auto moveDaffyBlue = frc::TrajectoryGenerator::GenerateTrajectory(
        bugsBlue,
        {},
        daffyBlue,
        config);
    auto moveDaffyRed = frc::TrajectoryGenerator::GenerateTrajectory(
        bugsRed,
        {},
        daffyRed,
        config);

    auto moveDaffyFromPredaffyBlue = frc::TrajectoryGenerator::GenerateTrajectory(
        predaffyBlue,
        {},
        daffyBlue,
        config);
    auto moveDaffyFromPredaffyRed = frc::TrajectoryGenerator::GenerateTrajectory(
        predaffyRed,
        {},
        daffyRed,
        config);

    auto movePreDaffyBlue = frc::TrajectoryGenerator::GenerateTrajectory(
        bugsBlue,
        {},
        predaffyBlue,
        config);
    auto movePreDaffyRed = frc::TrajectoryGenerator::GenerateTrajectory(
        bugsRed,
        {},
        predaffyRed,
        config);

    auto moveShootBlue = frc::TrajectoryGenerator::GenerateTrajectory(
        porkyBlue,
        {},
        shootBlue,
        reverseConfig);
    
    auto moveShootRed = frc::TrajectoryGenerator::GenerateTrajectory(
        porkyRed,
        {},
        shootRed,
        reverseConfig);

    frc2::SwerveControllerCommand<4> cmd_move_moveBugsBlue(
        moveBugsBlue,
        [&] () { return drivetrain->getPose_m(); },
        drivetrain->getKinematics(),
        frc2::PIDController(DriveConstants::KPX, DriveConstants::KIX, DriveConstants::KDX),
        frc2::PIDController(DriveConstants::KPY, DriveConstants::KIY, DriveConstants::KDY),
        thetaController,
        [this] (auto states) { drivetrain->setModuleStates(states); },
        {drivetrain}
    );

    frc2::SwerveControllerCommand<4> cmd_move_moveBugsRed(
        moveBugsRed,
        [&] () { return drivetrain->getPose_m(); },
        drivetrain->getKinematics(),
        frc2::PIDController(DriveConstants::KPX, DriveConstants::KIX, DriveConstants::KDX),
        frc2::PIDController(DriveConstants::KPY, DriveConstants::KIY, DriveConstants::KDY),
        thetaController,
        [this] (auto states) { drivetrain->setModuleStates(states); },
        {drivetrain}
    );

    frc2::SwerveControllerCommand<4> cmd_move_moveDaffyBlue(
        moveDaffyBlue,
        [&] () { return drivetrain->getPose_m(); },
        drivetrain->getKinematics(),
        frc2::PIDController(DriveConstants::KPX, DriveConstants::KIX, DriveConstants::KDX),
        frc2::PIDController(DriveConstants::KPY, DriveConstants::KIY, DriveConstants::KDY),
        thetaController,
        [this] (auto states) { drivetrain->setModuleStates(states); },
        {drivetrain}
    );

    frc2::SwerveControllerCommand<4> cmd_move_moveDaffyRed(
        moveDaffyRed,
        [&] () { return drivetrain->getPose_m(); },
        drivetrain->getKinematics(),
        frc2::PIDController(DriveConstants::KPX, DriveConstants::KIX, DriveConstants::KDX),
        frc2::PIDController(DriveConstants::KPY, DriveConstants::KIY, DriveConstants::KDY),
        thetaController,
        [this] (auto states) { drivetrain->setModuleStates(states); },
        {drivetrain}
    );

    frc2::SwerveControllerCommand<4> cmd_move_movePreDaffyBlue(
        movePreDaffyBlue,
        [&] () { return drivetrain->getPose_m(); },
        drivetrain->getKinematics(),
        frc2::PIDController(DriveConstants::KPX, DriveConstants::KIX, DriveConstants::KDX),
        frc2::PIDController(DriveConstants::KPY, DriveConstants::KIY, DriveConstants::KDY),
        thetaController,
        [this] (auto states) { drivetrain->setModuleStates(states); },
        {drivetrain}
    );

    frc2::SwerveControllerCommand<4> cmd_move_movePreDaffyRed(
        movePreDaffyRed,
        [&] () { return drivetrain->getPose_m(); },
        drivetrain->getKinematics(),
        frc2::PIDController(DriveConstants::KPX, DriveConstants::KIX, DriveConstants::KDX),
        frc2::PIDController(DriveConstants::KPY, DriveConstants::KIY, DriveConstants::KDY),
        thetaController,
        [this] (auto states) { drivetrain->setModuleStates(states); },
        {drivetrain}
    );
    
    frc2::SwerveControllerCommand<4> cmd_move_movePorkyBlue(
        movePorkyBlue,
        [&] () { return drivetrain->getPose_m(); },
        drivetrain->getKinematics(),
        frc2::PIDController(DriveConstants::KPX, DriveConstants::KIX, DriveConstants::KDX),
        frc2::PIDController(DriveConstants::KPY, DriveConstants::KIY, DriveConstants::KDY),
        thetaController,
        [this] (auto states) { drivetrain->setModuleStates(states); },
        {drivetrain}
    );

    frc2::SwerveControllerCommand<4> cmd_move_movePorkyRed(
        movePorkyRed,
        [&] () { return drivetrain->getPose_m(); },
        drivetrain->getKinematics(),
        frc2::PIDController(DriveConstants::KPX, DriveConstants::KIX, DriveConstants::KDX),
        frc2::PIDController(DriveConstants::KPY, DriveConstants::KIY, DriveConstants::KDY),
        thetaController,
        [this] (auto states) { drivetrain->setModuleStates(states); },
        {drivetrain}
    );

    frc2::SwerveControllerCommand<4> cmd_move_movePorkyFromDaffyBlue(
        movePorkyFromDaffyBlue,
        [&] () { return drivetrain->getPose_m(); },
        drivetrain->getKinematics(),
        frc2::PIDController(DriveConstants::KPX, DriveConstants::KIX, DriveConstants::KDX),
        frc2::PIDController(DriveConstants::KPY, DriveConstants::KIY, DriveConstants::KDY),
        thetaController,
        [this] (auto states) { drivetrain->setModuleStates(states); },
        {drivetrain}
    );

        frc2::SwerveControllerCommand<4> cmd_move_movePorkyFromDaffyRed(
        movePorkyFromDaffyRed,
        [&] () { return drivetrain->getPose_m(); },
        drivetrain->getKinematics(),
        frc2::PIDController(DriveConstants::KPX, DriveConstants::KIX, DriveConstants::KDX),
        frc2::PIDController(DriveConstants::KPY, DriveConstants::KIY, DriveConstants::KDY),
        thetaController,
        [this] (auto states) { drivetrain->setModuleStates(states); },
        {drivetrain}
    );

    frc2::SwerveControllerCommand<4> cmd_move_moveDaffyFromPredaffyBlue(
        moveDaffyFromPredaffyBlue,
        [&] () { return drivetrain->getPose_m(); },
        drivetrain->getKinematics(),
        frc2::PIDController(DriveConstants::KPX, DriveConstants::KIX, DriveConstants::KDX),
        frc2::PIDController(DriveConstants::KPY, DriveConstants::KIY, DriveConstants::KDY),
        thetaController,
        [this] (auto states) { drivetrain->setModuleStates(states); },
        {drivetrain}
    );

    frc2::SwerveControllerCommand<4> cmd_move_moveDaffyFromPredaffyRed(
        moveDaffyFromPredaffyRed,
        [&] () { return drivetrain->getPose_m(); },
        drivetrain->getKinematics(),
        frc2::PIDController(DriveConstants::KPX, DriveConstants::KIX, DriveConstants::KDX),
        frc2::PIDController(DriveConstants::KPY, DriveConstants::KIY, DriveConstants::KDY),
        thetaController,
        [this] (auto states) { drivetrain->setModuleStates(states); },
        {drivetrain}
    );

    frc2::SwerveControllerCommand<4> cmd_move_moveShootBlue(
        moveShootBlue,
        [&] () { return drivetrain->getPose_m(); },
        drivetrain->getKinematics(),
        frc2::PIDController(DriveConstants::KPX, DriveConstants::KIX, DriveConstants::KDX),
        frc2::PIDController(DriveConstants::KPY, DriveConstants::KIY, DriveConstants::KDY),
        thetaController,
        [this] (auto states) { drivetrain->setModuleStates(states); },
        {drivetrain}
    );

    frc2::SwerveControllerCommand<4> cmd_move_moveShootRed(
        moveShootRed,
        [&] () { return drivetrain->getPose_m(); },
        drivetrain->getKinematics(),
        frc2::PIDController(DriveConstants::KPX, DriveConstants::KIX, DriveConstants::KDX),
        frc2::PIDController(DriveConstants::KPY, DriveConstants::KIY, DriveConstants::KDY),
        thetaController,
        [this] (auto states) { drivetrain->setModuleStates(states); },
        {drivetrain}
    );

    frc2::SequentialCommandGroup *shoot3Blue = new frc2::SequentialCommandGroup();
    shoot3Blue->AddCommands
    (cmd_set_odometry,
    cmd_shooterPrime,
    cmd_intakeOne,
    cmd_move_moveBugsBlue,
    cmd_move_movePreDaffyBlue,
    cmd_turretTrack,
    frc2::WaitCommand((units::second_t).5),
    cmd_intakeShoot,
    frc2::WaitCommand((units::second_t)1.0),
    cmd_intakeAuto,
    cmd_move_moveDaffyFromPredaffyBlue,
    frc2::WaitCommand((units::second_t).5),
    cmd_shooterFar,
    frc2::WaitCommand((units::second_t).1),
    cmd_intakeShoot,
    frc2::WaitCommand((units::second_t).5)
    );

    frc2::SequentialCommandGroup *shoot3Red = new frc2::SequentialCommandGroup();
    shoot3Red->AddCommands
    (cmd_set_odometry,
    cmd_shooterPrime,
    cmd_intakeOne,
    cmd_move_moveBugsRed,
    cmd_move_movePreDaffyRed,
    cmd_turretTrack,
    frc2::WaitCommand((units::second_t).5),
    cmd_intakeShoot,
    frc2::WaitCommand((units::second_t)1.0),
    cmd_intakeAuto,
    cmd_move_moveDaffyFromPredaffyRed,
    frc2::WaitCommand((units::second_t).5),
    cmd_shooterFar,
    frc2::WaitCommand((units::second_t).1),
    cmd_intakeShoot,
    frc2::WaitCommand((units::second_t).5)
    );


    frc2::SequentialCommandGroup *shoot5Blue = new frc2::SequentialCommandGroup();
    shoot5Blue->AddCommands
    (cmd_set_odometry,
    cmd_shooterPrime,
    cmd_intakeOne,
    cmd_move_moveBugsBlue,
    cmd_move_movePreDaffyBlue,
    cmd_turretTrack,
    frc2::WaitCommand((units::second_t).5),
    cmd_intakeShoot,
    frc2::WaitCommand((units::second_t)1.0),
    cmd_intakeAuto,
    cmd_move_moveDaffyFromPredaffyBlue,
    frc2::WaitCommand((units::second_t).5),
    cmd_shooterFar,
    frc2::WaitCommand((units::second_t).1),
    cmd_intakeShoot,
    frc2::WaitCommand((units::second_t).5),
    cmd_intakeAuto,
    cmd_move_movePorkyFromDaffyBlue,
    frc2::WaitCommand((units::second_t)1.5),
    cmd_move_moveShootBlue,
    frc2::WaitCommand((units::second_t).5),
    cmd_turretTrack,
    frc2::WaitCommand((units::second_t).5),
    cmd_intakeAuto,
    frc2::WaitCommand((units::second_t)1)
    );

    frc2::SequentialCommandGroup *shoot5Red = new frc2::SequentialCommandGroup();
    shoot5Red->AddCommands
    (cmd_set_odometry,
    cmd_shooterPrime,
    cmd_intakeOne,
    cmd_move_moveBugsRed,
    cmd_move_movePreDaffyRed,
    cmd_turretTrack,
    frc2::WaitCommand((units::second_t).5),
    cmd_intakeShoot,
    frc2::WaitCommand((units::second_t)1.0),
    cmd_intakeAuto,
    cmd_move_moveDaffyFromPredaffyRed,
    frc2::WaitCommand((units::second_t).5),
    cmd_shooterFar,
    frc2::WaitCommand((units::second_t).1),
    cmd_intakeShoot,
    frc2::WaitCommand((units::second_t).5),
    cmd_intakeAuto,
    cmd_move_movePorkyFromDaffyRed,
    frc2::WaitCommand((units::second_t)1.5),
    cmd_move_moveShootRed,
    frc2::WaitCommand((units::second_t).5),
    cmd_turretTrack,
    frc2::WaitCommand((units::second_t).5),
    cmd_intakeAuto,
    frc2::WaitCommand((units::second_t)1)

    );

   //right now shoot 5 auto is shoot 3, need to make a red and blue auto
    m_chooser.AddOption("RED 3 ball auto", shoot3Red);
    m_chooser.AddOption("RED 5 ball auto", shoot5Red);
    m_chooser.AddOption("BLUE 3 ball auto", shoot3Blue);
    m_chooser.AddOption("BLUE 5 ball auto", shoot5Blue);
    
    frc::SmartDashboard::PutData(&m_chooser);
  
    //potential issues
    //turret can't see the hub, might need to set position manually
    //hood and flywheel need to be at different speeds depending on location
    //might run out of time
}

frc2::Command* ValorAuto::getCurrentAuto() {
    return m_chooser.GetSelected();
}