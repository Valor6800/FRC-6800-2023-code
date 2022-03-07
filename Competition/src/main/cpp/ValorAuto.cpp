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


    frc::Pose2d startPose = frc::Pose2d(7_m, 1.771_m, frc::Rotation2d(-92.1_deg)); //hub is 7, 2.771
    frc::Pose2d alternateStartPose = frc::Pose2d(5.566_m, 5.905_m, frc::Rotation2d(156_deg));

    //Bugs y blue .35
    //bringing bugs towards the wall
    frc::Pose2d bugsBlue = frc::Pose2d(7_m, 0.3_m, frc::Rotation2d(-90_deg));
    frc::Pose2d bugsRed = frc::Pose2d(7_m, 0.3_m, frc::Rotation2d(-90_deg));

    frc::Pose2d rotateBlue = frc::Pose2d(6.5_m, 0.9_m, frc::Rotation2d(-150_deg));
    frc::Pose2d rotateRed = frc::Pose2d(6.5_m, 0.9_m, frc::Rotation2d(-150_deg));

    //subtract all by 90 if starting with turrethome

    //Daffy y was 1.6
    frc::Pose2d daffyBlue = frc::Pose2d(3.55_m, 1.7_m, frc::Rotation2d(80_deg));
    //-3.45, -1.071 relative to hub
    //72.75
    frc::Pose2d daffyRed = frc::Pose2d(3.55_m, 1.7_m, frc::Rotation2d(80_deg));
    
    frc::Pose2d predaffyBlue = frc::Pose2d(5.083_m, .7_m, frc::Rotation2d(77_deg));
    //-1.917, -1.071 relative to hub
    //150.81 angle to make shot with turret to the right
    //angles appear to be 90 degrees off so use 60.81 instead
    frc::Pose2d predaffyRed = frc::Pose2d(5.083_m, .7_m, frc::Rotation2d(77_deg));
    
    //shifting each movement by .5 to avoid smacking into the pipes
    frc::Pose2d porkyBlue = frc::Pose2d(-0.4_m, 2.1_m, frc::Rotation2d(200_deg));
    frc::Pose2d porkyRed = frc::Pose2d(-0.7_m, 2.1_m, frc::Rotation2d(200_deg));

    frc::Pose2d marvinBlue = frc::Pose2d(5.097_m, 6.805_m, frc::Rotation2d(156_deg));
    frc::Pose2d marvinRed = frc::Pose2d(5.097_m, 6.805_m, frc::Rotation2d(156_deg));
    

    frc::Pose2d shootBlue = frc::Pose2d(7_m, 1.2_m, frc::Rotation2d(100_deg));
    //0, -1.5771 relative to hub
    //+-180
    frc::Pose2d shootRed = frc::Pose2d(7_m, 1.2_m, frc::Rotation2d(100_deg));

    frc::Pose2d alternateShootBlue = frc::Pose2d(5.397_m, 7.105_m, frc::Rotation2d(66_deg));
    frc::Pose2d alternateShootRed = frc::Pose2d(5.397_m, 7.105_m, frc::Rotation2d(66_deg));
    //frc::Translation2d preBugs{startPose.X(), bugs.Translation().Y()};
    // frc::Rotation2d gyroOffsetAuto = drivetrain->getGyroOffset() + frc::Rotation2d(180_deg);

    frc2::InstantCommand cmd_shooterPrime = frc2::InstantCommand( [&] {
        shooter->state.flywheelState = Shooter::FlywheelState::FLYWHEEL_PRIME; 
        shooter->state.hoodState = Shooter::HoodState::HOOD_UP;
    } );

    frc2::InstantCommand cmd_shooterFar = frc2::InstantCommand( [&] {
        shooter->state.flywheelState = Shooter::FlywheelState::FLYWHEEL_AUTO; 
        shooter->state.hoodState = Shooter::HoodState::HOOD_AUTO;
    } );

    frc2::InstantCommand cmd_turretTrack = frc2::InstantCommand( [&] {
        shooter->state.turretState = Shooter::TurretState::TURRET_TRACK;
    } );

    frc2::InstantCommand cmd_turretHomeMid = frc2::InstantCommand( [&] {
        shooter->state.turretState = Shooter::TurretState::TURRET_HOME_MID;
    } );

    frc2::InstantCommand cmd_turretDisable = frc2::InstantCommand( [&] {
        shooter->state.turretState = Shooter::TurretState::TURRET_DISABLE;
    } );

    frc2::InstantCommand cmd_shooterDefault = frc2::InstantCommand( [&] {
        shooter->state.flywheelState = Shooter::FlywheelState::FLYWHEEL_DEFAULT; 
        shooter->state.turretState = Shooter::TurretState::TURRET_DISABLE;
        shooter->state.hoodState = Shooter::HoodState::HOOD_DOWN;
    } );

    frc2::InstantCommand cmd_intakeOne = frc2::InstantCommand( [&] { feeder->state.feederState = Feeder::FeederState::FEEDER_AUTO; } );
    frc2::InstantCommand cmd_intakeDisable = frc2::InstantCommand( [&] { feeder->state.feederState = Feeder::FeederState::FEEDER_DISABLE; } );



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

    frc2::InstantCommand cmd_set_alternateOdometry = frc2::InstantCommand( [&] {
        drivetrain->resetOdometry(alternateStartPose);
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

    auto moveRotateBlue = frc::TrajectoryGenerator::GenerateTrajectory(
        bugsBlue,
        {},
        rotateBlue,
        config);

    auto moveRotateRed = frc::TrajectoryGenerator::GenerateTrajectory(
        bugsRed,
        {},
        rotateRed,
        config);    

    auto movePreDaffyBlue = frc::TrajectoryGenerator::GenerateTrajectory(
        rotateBlue, //bugsBlue,//
        {},
        predaffyBlue,
        config);

        
    auto movePreDaffyRed = frc::TrajectoryGenerator::GenerateTrajectory(
        rotateRed, //bugsRed,//
        {},
        predaffyRed,
        config);

    auto moveShootBlue = frc::TrajectoryGenerator::GenerateTrajectory(
        porkyBlue,
        {},
        shootBlue,
        reverseConfig);

    auto moveMarvinBlue = frc::TrajectoryGenerator::GenerateTrajectory(
        alternateStartPose,
        {},
        marvinBlue,
        config);

    auto moveMarvinRed = frc::TrajectoryGenerator::GenerateTrajectory(
        alternateStartPose,
        {},
        marvinRed,
        config);
    
    auto moveShootRed = frc::TrajectoryGenerator::GenerateTrajectory(
        porkyRed,
        {},
        shootRed,
        reverseConfig);

    auto moveAlternateShootBlue = frc::TrajectoryGenerator::GenerateTrajectory(
        marvinBlue,
        {},
        alternateShootBlue,
        config);

    auto moveAlternateShootRed = frc::TrajectoryGenerator::GenerateTrajectory(
        marvinRed,
        {},
        alternateShootRed,
        config);

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

    frc2::SwerveControllerCommand<4> cmd_move_moveRotateBlue(
        moveRotateBlue,
        [&] () { return drivetrain->getPose_m(); },
        drivetrain->getKinematics(),
        frc2::PIDController(DriveConstants::KPX, DriveConstants::KIX, DriveConstants::KDX),
        frc2::PIDController(DriveConstants::KPY, DriveConstants::KIY, DriveConstants::KDY),
        thetaController,
        [this] (auto states) { drivetrain->setModuleStates(states); },
        {drivetrain}
    );

    frc2::SwerveControllerCommand<4> cmd_move_moveRotateRed(
        moveRotateRed,
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

    
    frc2::SwerveControllerCommand<4> cmd_move_moveMarvinBlue(
        moveMarvinBlue,
        [&] () { return drivetrain->getPose_m(); },
        drivetrain->getKinematics(),
        frc2::PIDController(DriveConstants::KPX, DriveConstants::KIX, DriveConstants::KDX),
        frc2::PIDController(DriveConstants::KPY, DriveConstants::KIY, DriveConstants::KDY),
        thetaController,
        [this] (auto states) { drivetrain->setModuleStates(states); },
        {drivetrain}
    );

    frc2::SwerveControllerCommand<4> cmd_move_moveMarvinRed(
        moveMarvinRed,
        [&] () { return drivetrain->getPose_m(); },
        drivetrain->getKinematics(),
        frc2::PIDController(DriveConstants::KPX, DriveConstants::KIX, DriveConstants::KDX),
        frc2::PIDController(DriveConstants::KPY, DriveConstants::KIY, DriveConstants::KDY),
        thetaController,
        [this] (auto states) { drivetrain->setModuleStates(states); },
        {drivetrain}
    );

    frc2::SwerveControllerCommand<4> cmd_move_moveAlternateShootBlue(
        moveAlternateShootBlue,
        [&] () { return drivetrain->getPose_m(); },
        drivetrain->getKinematics(),
        frc2::PIDController(DriveConstants::KPX, DriveConstants::KIX, DriveConstants::KDX),
        frc2::PIDController(DriveConstants::KPY, DriveConstants::KIY, DriveConstants::KDY),
        thetaController,
        [this] (auto states) { drivetrain->setModuleStates(states); },
        {drivetrain}
    );

    frc2::SwerveControllerCommand<4> cmd_move_moveAlternateShootRed(
        moveAlternateShootRed,
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
    cmd_move_moveRotateBlue,
    cmd_move_movePreDaffyBlue,
    cmd_turretTrack,
    frc2::WaitCommand((units::second_t).5),
    cmd_intakeShoot,
    frc2::WaitCommand((units::second_t)1.0),
    cmd_intakeOne,
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
    cmd_move_moveRotateRed,
    cmd_move_movePreDaffyRed,
    cmd_turretTrack,
    frc2::WaitCommand((units::second_t).5),
    cmd_intakeShoot,
    frc2::WaitCommand((units::second_t)1.0),
    cmd_intakeOne,
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
    cmd_turretDisable,
    cmd_shooterPrime,
    cmd_intakeOne,
    cmd_move_moveBugsBlue,
    cmd_move_moveRotateBlue,
    cmd_move_movePreDaffyBlue,
    cmd_turretTrack,
    frc2::WaitCommand((units::second_t).5),
    cmd_intakeShoot,
    frc2::WaitCommand((units::second_t)1.0),
    cmd_intakeOne,
    cmd_move_moveDaffyFromPredaffyBlue,
    frc2::WaitCommand((units::second_t).5),
    cmd_shooterFar,
    frc2::WaitCommand((units::second_t).2),
    cmd_intakeShoot,
    frc2::WaitCommand((units::second_t).5),
    cmd_turretDisable,
    cmd_intakeOne,
    cmd_move_movePorkyFromDaffyBlue,
    frc2::WaitCommand((units::second_t)1.5),
    //cmd_intakeDisable,
    cmd_move_moveShootBlue,
    frc2::WaitCommand((units::second_t).5),
    cmd_shooterPrime,
    cmd_turretTrack,
    frc2::WaitCommand((units::second_t).75),
    cmd_intakeShoot,
    frc2::WaitCommand((units::second_t)1)
    );

    frc2::SequentialCommandGroup *shoot5Red = new frc2::SequentialCommandGroup();
    shoot5Red->AddCommands
    (cmd_set_odometry,
    cmd_turretDisable,
    cmd_shooterPrime,
    cmd_intakeOne,
    cmd_move_moveBugsRed,
    cmd_move_moveRotateRed,
    cmd_move_movePreDaffyRed,
    cmd_turretTrack,
    frc2::WaitCommand((units::second_t).5),
    cmd_intakeShoot,
    frc2::WaitCommand((units::second_t)1.0),
    cmd_intakeOne,
    cmd_move_moveDaffyFromPredaffyRed,
    frc2::WaitCommand((units::second_t).5),
    cmd_shooterFar,
    frc2::WaitCommand((units::second_t).2),
    cmd_intakeShoot,
    frc2::WaitCommand((units::second_t).5),
    cmd_turretDisable,
    cmd_intakeOne,
    cmd_move_movePorkyFromDaffyRed,
    frc2::WaitCommand((units::second_t)1.5),
    //cmd_intakeDisable,
    cmd_move_moveShootRed,
    frc2::WaitCommand((units::second_t).5),
    cmd_shooterPrime,
    cmd_turretTrack,
    frc2::WaitCommand((units::second_t).75),
    cmd_intakeShoot,
    frc2::WaitCommand((units::second_t)1)

    );


    frc2::SequentialCommandGroup *shoot2Blue = new frc2::SequentialCommandGroup();
    shoot2Blue->AddCommands
    (cmd_set_alternateOdometry,
    cmd_shooterPrime,
    cmd_intakeOne,
    cmd_move_moveMarvinBlue,
    cmd_move_moveAlternateShootBlue,
    cmd_turretTrack,
    frc2::WaitCommand((units::second_t).5),
    cmd_intakeShoot,
    frc2::WaitCommand((units::second_t)1.0),
    cmd_turretDisable,
    cmd_intakeDisable
    );

    frc2::SequentialCommandGroup *shoot2Red = new frc2::SequentialCommandGroup();
    shoot2Red->AddCommands
    (cmd_set_alternateOdometry,
    cmd_shooterPrime,
    cmd_intakeOne,
    cmd_move_moveMarvinRed,
    cmd_move_moveAlternateShootRed,
    cmd_turretTrack,
    frc2::WaitCommand((units::second_t).5),
    cmd_intakeShoot,
    frc2::WaitCommand((units::second_t)1.0),
    cmd_turretDisable,
    cmd_intakeDisable
    );


    frc2::SequentialCommandGroup *shooterTest = new frc2::SequentialCommandGroup();
    shooterTest->AddCommands
    (cmd_set_odometry,
    cmd_shooterPrime,
    cmd_intakeOne,
    cmd_turretTrack,
    frc2::WaitCommand((units::second_t)3),
    cmd_intakeShoot,
    frc2::WaitCommand((units::second_t)1.0),
    cmd_intakeOne
    );



   //right now shoot 5 auto is shoot 3, need to make a red and blue auto
    m_chooser.AddOption("RED 3 ball auto", shoot3Red);
    m_chooser.AddOption("RED 5 ball auto", shoot5Red);
    m_chooser.AddOption("RED 2 ball auto", shoot2Red);

    m_chooser.AddOption("BLUE 3 ball auto", shoot3Blue);
    m_chooser.AddOption("BLUE 5 ball auto", shoot5Blue);
    m_chooser.AddOption("BLUE 2 ball auto", shoot2Blue);

    m_chooser.AddOption("SHOOTER TEST", shooterTest);

    
    frc::SmartDashboard::PutData(&m_chooser);
  
    //potential issues
    //turret can't see the hub, might need to set position manually
    //hood and flywheel need to be at different speeds depending on location
    //might run out of time
}

frc2::Command* ValorAuto::getCurrentAuto() {
    return m_chooser.GetSelected();
}