#include "ValorAuto.h"
#include <iostream>

//See https://github.com/wpilibsuite/allwpilib/blob/v2022.1.1/wpilibcExamples/src/main/cpp/examples/SwerveControllerCommand/cpp/RobotContainer.cpp
ValorAuto::ValorAuto(Drivetrain *_drivetrain, Shooter *_shooter, Feeder *_feeder) : 
    drivetrain(_drivetrain), 
    shooter(_shooter),
    feeder(_feeder)
{   
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

    frc::Pose2d startPoseRed = frc::Pose2d(7_m, 1.771_m, frc::Rotation2d(-92.1_deg));
    frc::Pose2d startPoseBlue = frc::Pose2d(7_m, 1.771_m, frc::Rotation2d(-92.1_deg));

    frc::Pose2d bugsRed = frc::Pose2d(7_m, 0.3_m, frc::Rotation2d(-90_deg));
    frc::Pose2d backBugsRed = frc::Pose2d(7_m, 1.2_m, frc::Rotation2d(-90_deg));
    frc::Pose2d bugsBlue = frc::Pose2d(7_m, 0.3_m, frc::Rotation2d(-90_deg));
    frc::Pose2d backBugsBlue = frc::Pose2d(7_m, 1.2_m, frc::Rotation2d(-90_deg));

    frc::Pose2d predaffyRed = frc::Pose2d(5.083_m, .7_m, frc::Rotation2d(135_deg));
    frc::Pose2d daffyRed = frc::Pose2d(3.55_m, 2.15_m, frc::Rotation2d(115_deg));
    frc::Pose2d predaffyBlue = frc::Pose2d(5.083_m, .7_m, frc::Rotation2d(135_deg));
    frc::Pose2d daffyBlue = frc::Pose2d(3.55_m, 2.15_m, frc::Rotation2d(115_deg));

    frc::Translation2d porkyEntryRed = frc::Translation2d(1.3_m, 3.25_m); //3.5
    frc::Pose2d porkyRed = frc::Pose2d(-0.45_m, 2.35_m, frc::Rotation2d(212_deg));
    frc::Pose2d porkyStepBackRed = frc::Pose2d(.7_m, 3.2_m, frc::Rotation2d(212_deg));
    frc::Translation2d porkyEntryBlue = frc::Translation2d(1.3_m, 3.25_m); //3.5
    frc::Pose2d porkyBlue = frc::Pose2d(-0.45_m, 2.35_m, frc::Rotation2d(212_deg));
    frc::Pose2d porkyStepBackBlue = frc::Pose2d(.7_m, 3.2_m, frc::Rotation2d(212_deg));
    
    frc::Translation2d shootConstrainRed = frc::Translation2d(3.15_m, 2_m); //1.2_m in case we need to push it more towards wall
    frc::Pose2d shootRed = frc::Pose2d(6_m, 1.2_m, frc::Rotation2d(53_deg)); // lower angle to 50 in case of time
    frc::Translation2d shootConstrainBlue = frc::Translation2d(3.15_m, 2_m); //1.2_m in case we need to push it more towards wall
    frc::Pose2d shootBlue = frc::Pose2d(6_m, 1.2_m, frc::Rotation2d(53_deg)); //lower angle to 50 in case of time

    frc::Pose2d startPose2ballRed = frc::Pose2d(units::meter_t(5.75), units::meter_t(5.42), frc::Rotation2d(137_deg));
    frc::Pose2d startPose2ballBlue = frc::Pose2d(units::meter_t(5.75), units::meter_t(5.42), frc::Rotation2d(137_deg));
    
    frc::Pose2d marvinRed = frc::Pose2d(units::meter_t(4), units::meter_t(6.9), frc::Rotation2d(137_deg));
    frc::Pose2d marvinBlue = frc::Pose2d(units::meter_t(4), units::meter_t(6.9), frc::Rotation2d(137_deg));

    frc::Pose2d marvinShootRed = frc::Pose2d(units::meter_t(5), units::meter_t(7.5), frc::Rotation2d(27_deg));
    frc::Pose2d marvinShootBlue = frc::Pose2d(units::meter_t(5), units::meter_t(7.5), frc::Rotation2d(27_deg));

    frc::Pose2d marvinShootAltRed = frc::Pose2d(units::meter_t(3.35), units::meter_t(4), frc::Rotation2d(-90_deg));
    frc::Pose2d marvinShootAltBlue = frc::Pose2d(units::meter_t(3.35), units::meter_t(4), frc::Rotation2d(-90_deg));

    frc::Pose2d tasRed = frc::Pose2d(units::meter_t(6.069), units::meter_t(7.8), frc::Rotation2d(60_deg));
    frc::Pose2d tasBlue = frc::Pose2d(units::meter_t(6.069), units::meter_t(7.8), frc::Rotation2d(60_deg));

    frc::Translation2d tasToSpeedyConstrainRed = frc::Translation2d(.75_m, 2.5_m); //1.2_m in case we need to push it more towards wall
    frc::Translation2d tasToSpeedyConstrainBlue = frc::Translation2d(.75_m, 2.5_m); //1.2_m in case we need to push it more towards wall

    frc::Pose2d hangarSpotRed = frc::Pose2d(units::meter_t(4), units::meter_t(7.5), frc::Rotation2d(190_deg));
    frc::Pose2d hangarSpotBlue = frc::Pose2d(units::meter_t(4), units::meter_t(7.5), frc::Rotation2d(190_deg));

    frc::Pose2d speedyRed = frc::Pose2d(3.35_m, 2.8_m, frc::Rotation2d(-60_deg)); //originally 0
    frc::Pose2d speedyBlue = frc::Pose2d(3.35_m, 2.8_m, frc::Rotation2d(-60_deg)); //originally 0

    frc::Pose2d trenchEndRed = frc::Pose2d(3.75_m, 3_m, frc::Rotation2d(135_deg)); 
    frc::Pose2d trenchEndBlue = frc::Pose2d(3.75_m, 3_m, frc::Rotation2d(135_deg));

    frc::Pose2d endPose2ballRed = frc::Pose2d(10_m, 10_m, frc::Rotation2d(0_deg));
    frc::Pose2d endPose2ballBlue = frc::Pose2d(10_m, 10_m, frc::Rotation2d(0_deg));


    frc2::InstantCommand cmd_printHeading = frc2::InstantCommand( [&] {
       // std::cout << drivetrain->getPose_m().Rotation().Degrees().to<double>() << std::endl;    
    } );

    frc2::InstantCommand cmd_nextBall = frc2::InstantCommand( [&] {
        shooter->state.currentBall++; 
    } );

//CAN USE AUTO INSTEAD OF TRACK TO MANUALLY CHANGE VALUES
//TRACK USES LINE OF BEST FIT
    frc2::InstantCommand cmd_shooterAuto = frc2::InstantCommand( [&] {
        shooter->state.flywheelState = Shooter::FlywheelState::FLYWHEEL_TRACK;
        shooter->state.hoodState = Shooter::HoodState::HOOD_TRACK;
    } );
    frc2::InstantCommand cmd_shooterPoop = frc2::InstantCommand( [&] {
        shooter->state.flywheelState = Shooter::FlywheelState::FLYWHEEL_POOP;
        shooter->state.hoodState = Shooter::HoodState::HOOD_POOP;
    } );

    frc2::InstantCommand cmd_turretTrack = frc2::InstantCommand( [&] {
        shooter->state.turretState = Shooter::TurretState::TURRET_TRACK;
    } );
    frc2::InstantCommand cmd_shooterTarmac = frc2::InstantCommand( [&] {
        shooter->state.flywheelState = Shooter::FlywheelState::FLYWHEEL_DEFAULT;
        shooter->state.hoodState = Shooter::HoodState::HOOD_DOWN;
    } );

    frc2::InstantCommand cmd_turretHomeMid = frc2::InstantCommand( [&] {
        shooter->state.turretState = Shooter::TurretState::TURRET_HOME_MID;
    } );
    frc2::InstantCommand cmd_turretHomeLeft = frc2::InstantCommand( [&] {
        shooter->state.turretState = Shooter::TurretState::TURRET_HOME_LEFT;
    } );
    frc2::InstantCommand cmd_turretHomeRight = frc2::InstantCommand( [&] {
        shooter->state.turretState = Shooter::TurretState::TURRET_HOME_RIGHT;
    } );

    frc2::InstantCommand cmd_turretDisable = frc2::InstantCommand( [&] {
        shooter->state.turretState = Shooter::TurretState::TURRET_DISABLE;
    } );

    frc2::InstantCommand cmd_intakeOne = frc2::InstantCommand( [&] { feeder->state.feederState = Feeder::FeederState::FEEDER_REGULAR_INTAKE; } );
    frc2::InstantCommand cmd_intakeDisable = frc2::InstantCommand( [&] { feeder->state.feederState = Feeder::FeederState::FEEDER_DISABLE; } );
    frc2::InstantCommand cmd_intakeAuto = frc2::InstantCommand( [&] { feeder->state.feederState = Feeder::FeederState::FEEDER_CURRENT_INTAKE; } );
    frc2::InstantCommand cmd_intakeClearDeque = frc2::InstantCommand( [&] { feeder->resetDeque();} );    
    frc2::InstantCommand cmd_intakeShoot = frc2::InstantCommand( [&] { feeder->state.feederState = Feeder::FeederState::FEEDER_SHOOT; } );
    frc2::InstantCommand cmd_intakeReverse = frc2::InstantCommand( [&] { feeder->state.feederState = Feeder::FeederState::FEEDER_REVERSE; } );
    
    frc2::InstantCommand cmd_setOdometryRed = frc2::InstantCommand( [&] {
        drivetrain->resetOdometry(startPoseRed);
    });
    frc2::InstantCommand cmd_setOdometryBlue = frc2::InstantCommand( [&] {
        drivetrain->resetOdometry(startPoseBlue);
    });
    frc2::InstantCommand cmd_set2ballOdometryRed = frc2::InstantCommand( [&] {
        drivetrain->resetOdometry(startPose2ballRed);
    });
    frc2::InstantCommand cmd_set2ballOdometryBlue = frc2::InstantCommand( [&] {
        drivetrain->resetOdometry(startPose2ballBlue);
    });
    frc2::InstantCommand cmd_setEnd2ballRed = frc2::InstantCommand( [&] {
        drivetrain->resetOdometry(endPose2ballRed);
    });
    frc2::InstantCommand cmd_setEnd2ballBlue = frc2::InstantCommand( [&] {
        drivetrain->resetOdometry(endPose2ballBlue);
    });

    auto moveBugsRed = frc::TrajectoryGenerator::GenerateTrajectory(
        startPoseRed,
        {},
        bugsRed,
        config);
    auto moveBugsBlue = frc::TrajectoryGenerator::GenerateTrajectory(
        startPoseBlue,
        {},
        bugsBlue,
        config);

    auto moveBackBugsRed = frc::TrajectoryGenerator::GenerateTrajectory(
        bugsRed,
        {},
        backBugsRed,
        reverseConfig);
    auto moveBackBugsBlue = frc::TrajectoryGenerator::GenerateTrajectory(
        bugsBlue,
        {},
        backBugsBlue,
        reverseConfig);

    auto movePreDaffyRed = frc::TrajectoryGenerator::GenerateTrajectory(
        backBugsRed,
        {},
        predaffyRed,
        config);
    auto movePreDaffyBlue = frc::TrajectoryGenerator::GenerateTrajectory(
        backBugsBlue,
        {},
        predaffyBlue,
        config);

    auto moveDaffyFromPredaffyRed = frc::TrajectoryGenerator::GenerateTrajectory(
        predaffyRed,
        {},
        daffyRed,
        config);
    auto moveDaffyFromPredaffyBlue = frc::TrajectoryGenerator::GenerateTrajectory(
        predaffyBlue,
        {},
        daffyBlue,
        config);

    auto movePorkyFromDaffyRed = frc::TrajectoryGenerator::GenerateTrajectory(
        daffyRed,
        {porkyEntryRed},
        porkyRed,
        config);
    auto movePorkyFromDaffyBlue = frc::TrajectoryGenerator::GenerateTrajectory(
        daffyBlue,
        {porkyEntryBlue},
        porkyBlue,
        config);

    auto moveStepBackRed = frc::TrajectoryGenerator::GenerateTrajectory(
        porkyRed,
        {},
        porkyStepBackRed,
        reverseConfig);
    auto moveStepBackBlue = frc::TrajectoryGenerator::GenerateTrajectory(
        porkyBlue,
        {},
        porkyStepBackBlue,
        reverseConfig);
    
    auto moveShootRed = frc::TrajectoryGenerator::GenerateTrajectory(
        porkyRed,
        {shootConstrainRed},
        shootRed,
        reverseConfig);
    auto moveShootBlue = frc::TrajectoryGenerator::GenerateTrajectory(
        porkyBlue,
        {shootConstrainBlue},
        shootBlue,
        reverseConfig);

    auto moveMarvinRed = frc::TrajectoryGenerator::GenerateTrajectory(
        startPose2ballRed,
        {},
        marvinRed,
        config);
    auto moveMarvinBlue = frc::TrajectoryGenerator::GenerateTrajectory(
        startPose2ballBlue,
        {},
        marvinBlue,
        config);

    auto moveMarvinShootRed = frc::TrajectoryGenerator::GenerateTrajectory(
        marvinRed,
        {},
        marvinShootRed,
        config
    );
    auto moveMarvinShootBlue = frc::TrajectoryGenerator::GenerateTrajectory(
        marvinBlue,
        {},
        marvinShootBlue,
        config
    );

    auto moveMarvinShootAltRed = frc::TrajectoryGenerator::GenerateTrajectory(
        marvinRed,
        {},
        marvinShootAltRed,
        config
    );
    auto moveMarvinShootAltBlue = frc::TrajectoryGenerator::GenerateTrajectory(
        marvinBlue,
        {},
        marvinShootAltBlue,
        config
    );

    auto moveTasRed = frc::TrajectoryGenerator::GenerateTrajectory(
        marvinShootRed,
        {},
        tasRed,
        config);
    auto moveTasBlue = frc::TrajectoryGenerator::GenerateTrajectory(
        marvinShootBlue,
        {},
        tasBlue,
        config);

    auto moveHangarRed = frc::TrajectoryGenerator::GenerateTrajectory(
        tasRed,
        {},
        hangarSpotRed,
        reverseConfig);
    auto moveHangarBlue = frc::TrajectoryGenerator::GenerateTrajectory(
        tasBlue,
        {},
        hangarSpotBlue,
        reverseConfig);

    auto moveEndRed = frc::TrajectoryGenerator::GenerateTrajectory(
        hangarSpotRed,
        {},
        tasRed,
        reverseConfig);
    auto moveEndBlue = frc::TrajectoryGenerator::GenerateTrajectory(
        hangarSpotBlue,
        {},
        tasBlue,
        reverseConfig);

    auto moveSpeedyFromTasRed = frc::TrajectoryGenerator::GenerateTrajectory(
        tasRed,
        {},
        speedyRed,
        reverseConfig);
    auto moveSpeedyFromTasBlue = frc::TrajectoryGenerator::GenerateTrajectory(
        tasBlue,
        {},
        speedyBlue,
        reverseConfig);
    
    auto moveSpeedyFromAltRed = frc::TrajectoryGenerator::GenerateTrajectory(
        marvinShootAltRed,
        {},
        speedyRed,
        config);
    auto moveSpeedyFromAltBlue = frc::TrajectoryGenerator::GenerateTrajectory(
        marvinShootAltBlue,
        {},
        speedyBlue,
        config);

    auto moveTasFromSpeedyRed = frc::TrajectoryGenerator::GenerateTrajectory(
        speedyRed,
        {},
        tasRed,
        reverseConfig);
    auto moveTasFromSpeedyBlue = frc::TrajectoryGenerator::GenerateTrajectory(
        speedyBlue,
        {},
        tasBlue,
        reverseConfig);

    auto moveEndFromSpeedyRed = frc::TrajectoryGenerator::GenerateTrajectory(
        speedyRed,
        {},
        trenchEndRed,
        config);
    auto moveEndFromSpeedyBlue = frc::TrajectoryGenerator::GenerateTrajectory(
        speedyBlue,
        {},
        trenchEndBlue,
        config);
    
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

    frc2::SwerveControllerCommand<4> cmd_move_moveBackBugsRed(
        moveBackBugsRed,
        [&] () { return drivetrain->getPose_m(); },
        drivetrain->getKinematics(),
        frc2::PIDController(DriveConstants::KPX, DriveConstants::KIX, DriveConstants::KDX),
        frc2::PIDController(DriveConstants::KPY, DriveConstants::KIY, DriveConstants::KDY),
        thetaController,
        [this] (auto states) { drivetrain->setModuleStates(states); },
        {drivetrain}
    );
    frc2::SwerveControllerCommand<4> cmd_move_moveBackBugsBlue(
        moveBackBugsBlue,
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

    frc2::SwerveControllerCommand<4> cmd_move_moveStepBackRed(
        moveStepBackRed,
        [&] () { return drivetrain->getPose_m(); },
        drivetrain->getKinematics(),
        frc2::PIDController(DriveConstants::KPX, DriveConstants::KIX, DriveConstants::KDX),
        frc2::PIDController(DriveConstants::KPY, DriveConstants::KIY, DriveConstants::KDY),
        thetaController,
        [this] (auto states) { drivetrain->setModuleStates(states); },
        {drivetrain}
    );
    frc2::SwerveControllerCommand<4> cmd_move_moveStepBackBlue(
        moveStepBackBlue,
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
    
    frc2::SwerveControllerCommand<4> cmd_move_moveMarvinShootRed(
        moveMarvinShootRed,
        [&] () { return drivetrain->getPose_m(); },
        drivetrain->getKinematics(),
        frc2::PIDController(DriveConstants::KPX, DriveConstants::KIX, DriveConstants::KDX),
        frc2::PIDController(DriveConstants::KPY, DriveConstants::KIY, DriveConstants::KDY),
        thetaController,
        [this] (auto states) { drivetrain->setModuleStates(states); },
        {drivetrain}
    );
    frc2::SwerveControllerCommand<4> cmd_move_moveMarvinShootBlue(
        moveMarvinShootBlue,
        [&] () { return drivetrain->getPose_m(); },
        drivetrain->getKinematics(),
        frc2::PIDController(DriveConstants::KPX, DriveConstants::KIX, DriveConstants::KDX),
        frc2::PIDController(DriveConstants::KPY, DriveConstants::KIY, DriveConstants::KDY),
        thetaController,
        [this] (auto states) { drivetrain->setModuleStates(states); },
        {drivetrain}
    );

    frc2::SwerveControllerCommand<4> cmd_move_moveMarvinShootAltRed(
        moveMarvinShootAltRed,
        [&] () { return drivetrain->getPose_m(); },
        drivetrain->getKinematics(),
        frc2::PIDController(DriveConstants::KPX, DriveConstants::KIX, DriveConstants::KDX),
        frc2::PIDController(DriveConstants::KPY, DriveConstants::KIY, DriveConstants::KDY),
        thetaController,
        [this] (auto states) { drivetrain->setModuleStates(states); },
        {drivetrain}
    );
    frc2::SwerveControllerCommand<4> cmd_move_moveMarvinShootAltBlue(
        moveMarvinShootAltBlue,
        [&] () { return drivetrain->getPose_m(); },
        drivetrain->getKinematics(),
        frc2::PIDController(DriveConstants::KPX, DriveConstants::KIX, DriveConstants::KDX),
        frc2::PIDController(DriveConstants::KPY, DriveConstants::KIY, DriveConstants::KDY),
        thetaController,
        [this] (auto states) { drivetrain->setModuleStates(states); },
        {drivetrain}
    );
    
    frc2::SwerveControllerCommand<4> cmd_move_moveTasRed(
        moveTasRed,
        [&] () { return drivetrain->getPose_m(); },
        drivetrain->getKinematics(),
        frc2::PIDController(DriveConstants::KPX, DriveConstants::KIX, DriveConstants::KDX),
        frc2::PIDController(DriveConstants::KPY, DriveConstants::KIY, DriveConstants::KDY),
        thetaController,
        [this] (auto states) { drivetrain->setModuleStates(states); },
        {drivetrain}
    );

    frc2::SwerveControllerCommand<4> cmd_move_moveTasBlue(
        moveTasBlue,
        [&] () { return drivetrain->getPose_m(); },
        drivetrain->getKinematics(),
        frc2::PIDController(DriveConstants::KPX, DriveConstants::KIX, DriveConstants::KDX),
        frc2::PIDController(DriveConstants::KPY, DriveConstants::KIY, DriveConstants::KDY),
        thetaController,
        [this] (auto states) { drivetrain->setModuleStates(states); },
        {drivetrain}
    );

    frc2::SwerveControllerCommand<4> cmd_move_moveHangarRed(
        moveHangarRed,
        [&] () { return drivetrain->getPose_m(); },
        drivetrain->getKinematics(),
        frc2::PIDController(DriveConstants::KPX, DriveConstants::KIX, DriveConstants::KDX),
        frc2::PIDController(DriveConstants::KPY, DriveConstants::KIY, DriveConstants::KDY),
        thetaController,
        [this] (auto states) { drivetrain->setModuleStates(states); },
        {drivetrain}
    );
    frc2::SwerveControllerCommand<4> cmd_move_moveHangarBlue(
        moveHangarBlue,
        [&] () { return drivetrain->getPose_m(); },
        drivetrain->getKinematics(),
        frc2::PIDController(DriveConstants::KPX, DriveConstants::KIX, DriveConstants::KDX),
        frc2::PIDController(DriveConstants::KPY, DriveConstants::KIY, DriveConstants::KDY),
        thetaController,
        [this] (auto states) { drivetrain->setModuleStates(states); },
        {drivetrain}
    );
    frc2::SwerveControllerCommand<4> cmd_move_moveSpeedyFromTasRed(
        moveSpeedyFromTasRed,
        [&] () { return drivetrain->getPose_m(); },
        drivetrain->getKinematics(),
        frc2::PIDController(DriveConstants::KPX, DriveConstants::KIX, DriveConstants::KDX),
        frc2::PIDController(DriveConstants::KPY, DriveConstants::KIY, DriveConstants::KDY),
        thetaController,
        [this] (auto states) { drivetrain->setModuleStates(states); },
        {drivetrain}
    );
    frc2::SwerveControllerCommand<4> cmd_move_moveSpeedyFromTasBlue(
        moveSpeedyFromTasBlue,
        [&] () { return drivetrain->getPose_m(); },
        drivetrain->getKinematics(),
        frc2::PIDController(DriveConstants::KPX, DriveConstants::KIX, DriveConstants::KDX),
        frc2::PIDController(DriveConstants::KPY, DriveConstants::KIY, DriveConstants::KDY),
        thetaController,
        [this] (auto states) { drivetrain->setModuleStates(states); },
        {drivetrain}
    );
    frc2::SwerveControllerCommand<4> cmd_move_moveSpeedyFromAltRed(
        moveSpeedyFromAltRed,
        [&] () { return drivetrain->getPose_m(); },
        drivetrain->getKinematics(),
        frc2::PIDController(DriveConstants::KPX, DriveConstants::KIX, DriveConstants::KDX),
        frc2::PIDController(DriveConstants::KPY, DriveConstants::KIY, DriveConstants::KDY),
        thetaController,
        [this] (auto states) { drivetrain->setModuleStates(states); },
        {drivetrain}
    );
    frc2::SwerveControllerCommand<4> cmd_move_moveSpeedyFromAltBlue(
        moveSpeedyFromAltBlue,
        [&] () { return drivetrain->getPose_m(); },
        drivetrain->getKinematics(),
        frc2::PIDController(DriveConstants::KPX, DriveConstants::KIX, DriveConstants::KDX),
        frc2::PIDController(DriveConstants::KPY, DriveConstants::KIY, DriveConstants::KDY),
        thetaController,
        [this] (auto states) { drivetrain->setModuleStates(states); },
        {drivetrain}
    );

    frc2::SwerveControllerCommand<4> cmd_move_moveTasFromSpeedyRed(
        moveTasFromSpeedyRed,
        [&] () { return drivetrain->getPose_m(); },
        drivetrain->getKinematics(),
        frc2::PIDController(DriveConstants::KPX, DriveConstants::KIX, DriveConstants::KDX),
        frc2::PIDController(DriveConstants::KPY, DriveConstants::KIY, DriveConstants::KDY),
        thetaController,
        [this] (auto states) { drivetrain->setModuleStates(states); },
        {drivetrain}
    );
    frc2::SwerveControllerCommand<4> cmd_move_moveTasFromSpeedyBlue(
        moveTasFromSpeedyBlue,
        [&] () { return drivetrain->getPose_m(); },
        drivetrain->getKinematics(),
        frc2::PIDController(DriveConstants::KPX, DriveConstants::KIX, DriveConstants::KDX),
        frc2::PIDController(DriveConstants::KPY, DriveConstants::KIY, DriveConstants::KDY),
        thetaController,
        [this] (auto states) { drivetrain->setModuleStates(states); },
        {drivetrain}
    );
   
    frc2::SwerveControllerCommand<4> cmd_move_moveEndFromSpeedyRed(
        moveEndFromSpeedyRed,
        [&] () { return drivetrain->getPose_m(); },
        drivetrain->getKinematics(),
        frc2::PIDController(DriveConstants::KPX, DriveConstants::KIX, DriveConstants::KDX),
        frc2::PIDController(DriveConstants::KPY, DriveConstants::KIY, DriveConstants::KDY),
        thetaController,
        [this] (auto states) { drivetrain->setModuleStates(states); },
        {drivetrain}
    );
     
    frc2::SwerveControllerCommand<4> cmd_move_moveEndFromSpeedyBlue(
        moveEndFromSpeedyBlue,
        [&] () { return drivetrain->getPose_m(); },
        drivetrain->getKinematics(),
        frc2::PIDController(DriveConstants::KPX, DriveConstants::KIX, DriveConstants::KDX),
        frc2::PIDController(DriveConstants::KPY, DriveConstants::KIY, DriveConstants::KDY),
        thetaController,
        [this] (auto states) { drivetrain->setModuleStates(states); },
        {drivetrain}
    );

    frc2::SwerveControllerCommand<4> cmd_move_moveEndRed(
        moveEndRed,
        [&] () { return drivetrain->getPose_m(); },
        drivetrain->getKinematics(),
        frc2::PIDController(DriveConstants::KPX, DriveConstants::KIX, DriveConstants::KDX),
        frc2::PIDController(DriveConstants::KPY, DriveConstants::KIY, DriveConstants::KDY),
        thetaController,
        [this] (auto states) { drivetrain->setModuleStates(states); },
        {drivetrain}
    );
    frc2::SwerveControllerCommand<4> cmd_move_moveEndBlue(
        moveEndBlue,
        [&] () { return drivetrain->getPose_m(); },
        drivetrain->getKinematics(),
        frc2::PIDController(DriveConstants::KPX, DriveConstants::KIX, DriveConstants::KDX),
        frc2::PIDController(DriveConstants::KPY, DriveConstants::KIY, DriveConstants::KDY),
        thetaController,
        [this] (auto states) { drivetrain->setModuleStates(states); },
        {drivetrain}
    );
    

    frc2::SequentialCommandGroup *shoot3Red = new frc2::SequentialCommandGroup();
    shoot3Red->AddCommands
    (cmd_setOdometryRed,
    cmd_shooterAuto,
    cmd_intakeOne,
    frc2::WaitCommand((units::second_t).125),
    cmd_intakeClearDeque,
    cmd_intakeAuto,
    cmd_move_moveBugsRed,
    cmd_intakeDisable,
    cmd_move_movePreDaffyRed,
    cmd_turretTrack,
    frc2::WaitCommand((units::second_t).5),
    cmd_intakeShoot,
    frc2::WaitCommand((units::second_t)1.0),
    cmd_nextBall,
    cmd_intakeAuto,
    cmd_move_moveDaffyFromPredaffyRed,
    frc2::WaitCommand((units::second_t).5),
    frc2::WaitCommand((units::second_t).2),
    cmd_intakeShoot,
    frc2::WaitCommand((units::second_t).5)
    );
    frc2::SequentialCommandGroup *shoot3Blue = new frc2::SequentialCommandGroup();
    shoot3Blue->AddCommands
    (cmd_setOdometryBlue,
    cmd_shooterAuto,
    cmd_intakeOne,
    frc2::WaitCommand((units::second_t).125),
    cmd_intakeClearDeque,
    cmd_intakeAuto,
    cmd_move_moveBugsBlue,
    cmd_intakeDisable,
    cmd_move_movePreDaffyBlue,
    cmd_turretTrack,
    frc2::WaitCommand((units::second_t).5),
    cmd_intakeShoot,
    frc2::WaitCommand((units::second_t)1.0),
    cmd_nextBall,
    cmd_intakeAuto,
    cmd_move_moveDaffyFromPredaffyBlue,
    frc2::WaitCommand((units::second_t).5),
    frc2::WaitCommand((units::second_t).2),
    cmd_intakeShoot,
    frc2::WaitCommand((units::second_t).5)
    );

    frc2::SequentialCommandGroup *shoot5Red = new frc2::SequentialCommandGroup();
    shoot5Red->AddCommands
    (cmd_setOdometryRed,
    cmd_turretDisable,
    cmd_shooterAuto,
    cmd_intakeOne,
    frc2::WaitCommand((units::second_t).125),
    cmd_intakeClearDeque,
    cmd_intakeAuto,
    cmd_move_moveBugsRed,
    cmd_intakeDisable,
    cmd_move_moveBackBugsRed,
    cmd_intakeDisable,
    cmd_move_movePreDaffyRed,
    cmd_turretTrack,
    frc2::WaitCommand((units::second_t).25),
    cmd_intakeShoot,
    frc2::WaitCommand((units::second_t).6),
    cmd_intakeAuto,
    cmd_move_moveDaffyFromPredaffyRed,
    frc2::WaitCommand((units::second_t).25),
    cmd_intakeShoot,
    frc2::WaitCommand((units::second_t).2),
    cmd_turretDisable,
    cmd_intakeOne,
    cmd_move_movePorkyFromDaffyRed,
    cmd_turretHomeMid,
    cmd_move_moveStepBackRed,
    cmd_intakeClearDeque,
    cmd_intakeAuto,
    frc2::WaitCommand((units::second_t).25),
    cmd_turretTrack,
    cmd_shooterTarmac,
    cmd_move_moveShootRed,
    cmd_shooterAuto,
    frc2::WaitCommand((units::second_t).375),
    cmd_intakeShoot
    );
    frc2::SequentialCommandGroup *shoot5Blue = new frc2::SequentialCommandGroup();
    shoot5Blue->AddCommands
    (cmd_setOdometryBlue,
    cmd_turretDisable,
    cmd_shooterAuto,
    cmd_intakeOne,
    frc2::WaitCommand((units::second_t).125),
    cmd_intakeClearDeque,
    cmd_intakeAuto,
    cmd_move_moveBugsBlue,
    cmd_intakeDisable,
    cmd_move_moveBackBugsBlue,
    cmd_intakeDisable,
    cmd_move_movePreDaffyBlue,
    cmd_turretTrack,
    frc2::WaitCommand((units::second_t).25),
    cmd_intakeShoot,
    frc2::WaitCommand((units::second_t).6),
    cmd_nextBall,
    cmd_intakeAuto,
    cmd_move_moveDaffyFromPredaffyBlue,
    frc2::WaitCommand((units::second_t).25),
    cmd_intakeShoot,
    frc2::WaitCommand((units::second_t).2),
    cmd_nextBall,
    cmd_turretDisable,
    cmd_intakeOne,
    cmd_move_movePorkyFromDaffyBlue,
    cmd_turretHomeMid,
    cmd_move_moveStepBackBlue,
    cmd_intakeClearDeque,
    cmd_intakeAuto,
    frc2::WaitCommand((units::second_t).25),
    cmd_turretTrack,
    cmd_shooterTarmac,
    cmd_move_moveShootBlue,
    cmd_shooterAuto,    
    frc2::WaitCommand((units::second_t).375),
    cmd_intakeShoot
    );
    
    frc2::SequentialCommandGroup *shoot2RedAlt = new frc2::SequentialCommandGroup();
    shoot2RedAlt->AddCommands
    (cmd_set2ballOdometryRed,
    cmd_intakeOne,
    frc2::WaitCommand((units::second_t).125),
    cmd_intakeClearDeque,
    cmd_intakeAuto,
    cmd_shooterAuto,
    cmd_move_moveMarvinRed,
    cmd_intakeDisable,
    cmd_move_moveMarvinShootRed,
    cmd_turretTrack,
    frc2::WaitCommand((units::second_t).5),
    cmd_intakeShoot,
    frc2::WaitCommand((units::second_t).5),
    cmd_intakeOne,
    cmd_turretHomeMid,
    cmd_shooterPoop,
    cmd_move_moveTasRed,
    cmd_move_moveHangarRed,
    cmd_intakeShoot,
    frc2::WaitCommand((units::second_t)1),
    cmd_intakeDisable,
    cmd_move_moveEndRed
    );

    frc2::SequentialCommandGroup *shoot2BlueAlt = new frc2::SequentialCommandGroup();
    shoot2BlueAlt->AddCommands
    (cmd_set2ballOdometryBlue,
    cmd_intakeOne,
    frc2::WaitCommand((units::second_t).125),
    cmd_intakeClearDeque,
    cmd_intakeAuto,
    cmd_shooterAuto,
    cmd_move_moveMarvinBlue,
    cmd_intakeDisable,
    cmd_move_moveMarvinShootBlue,
    cmd_turretTrack,
    frc2::WaitCommand((units::second_t).5),
    cmd_intakeShoot,
    frc2::WaitCommand((units::second_t).5),
    cmd_intakeOne,
    cmd_turretHomeMid,
    cmd_shooterPoop,
    cmd_move_moveTasBlue,
    cmd_move_moveHangarBlue,
    cmd_intakeShoot,
    frc2::WaitCommand((units::second_t)1),
    cmd_intakeDisable,
    cmd_move_moveEndBlue
    );

    frc2::SequentialCommandGroup *shoot2Def2Red = new frc2::SequentialCommandGroup();
    shoot2Def2Red->AddCommands
    (cmd_set2ballOdometryRed,
    cmd_intakeOne,
    cmd_turretHomeLeft,
    cmd_shooterAuto,
    frc2::WaitCommand((units::second_t).125),
    cmd_intakeClearDeque,
    cmd_intakeAuto,
    cmd_move_moveMarvinRed,
    cmd_intakeDisable,
    cmd_move_moveMarvinShootAltRed,
    cmd_turretTrack,
    frc2::WaitCommand((units::second_t).5),
    cmd_intakeShoot,
    frc2::WaitCommand((units::second_t).5),
    cmd_intakeOne,
    cmd_turretHomeMid,
    cmd_shooterPoop,
    cmd_move_moveSpeedyFromAltRed,
    cmd_move_moveTasFromSpeedyRed,
    cmd_move_moveHangarRed,
    cmd_intakeShoot,
    frc2::WaitCommand((units::second_t)1),
    cmd_intakeDisable,
    cmd_move_moveEndRed,
    cmd_turretHomeRight
    );

    frc2::SequentialCommandGroup *shoot2Def2Blue = new frc2::SequentialCommandGroup();
    shoot2Def2Blue->AddCommands
    (cmd_set2ballOdometryBlue,
    cmd_intakeOne,
    cmd_turretHomeLeft,
    cmd_shooterAuto,
    frc2::WaitCommand((units::second_t).125),
    cmd_intakeClearDeque,
    cmd_intakeAuto,
    cmd_move_moveMarvinBlue,
    cmd_intakeDisable,
    cmd_move_moveMarvinShootAltBlue,
    cmd_turretTrack,
    frc2::WaitCommand((units::second_t).5),
    cmd_intakeShoot,
    frc2::WaitCommand((units::second_t).5),
    cmd_intakeOne,
    cmd_turretHomeMid,
    cmd_shooterPoop,
    cmd_move_moveSpeedyFromAltBlue,
    cmd_move_moveTasFromSpeedyBlue,
    cmd_move_moveHangarBlue,
    cmd_intakeShoot,
    frc2::WaitCommand((units::second_t)1),
    cmd_intakeDisable,
    cmd_move_moveEndBlue,
    cmd_turretHomeRight
    );

    m_chooser.AddOption("RED 2 ball + 1 Defensive", shoot2RedAlt);
    m_chooser.AddOption("Blue 2 ball + 1 Defensive", shoot2BlueAlt);

    m_chooser.AddOption("RED 2 ball + 2 Defensive", shoot2Def2Red);
    m_chooser.AddOption("Blue 2 ball + 2 Defensive", shoot2Def2Blue);

    

    m_chooser.AddOption("RED 3 ball auto", shoot3Red);
    m_chooser.AddOption("RED 5 ball auto", shoot5Red);

    m_chooser.AddOption("BLUE 3 ball auto", shoot3Blue);
    m_chooser.AddOption("BLUE 5 ball auto", shoot5Blue);

    frc::SmartDashboard::PutData(&m_chooser);
}

frc2::Command* ValorAuto::getCurrentAuto() {
    return m_chooser.GetSelected();
}