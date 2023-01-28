/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
#include "subsystems/Drivetrain.h"
#include <iostream>

#define TXRANGE  30.0f
#define KPIGEON 2.0f
#define KLIMELIGHT -29.8f

#define KPX 0.5f //.75
#define KIX 0.0f //0
#define KDX 0.0f //.1
#define KFX 0.0f

#define KPY 0.5f //.75
#define KIY 0.0f //0
#define KDY 0.0f //.1
#define KFY 0.0f

#define KPT 4.0f
#define KIT 0.0f
#define KDT 0.0f
#define KFT 0.0f

#define AZIMUTH_K_P 0.00001f
#define AZIMUTH_K_I 0.0f
#define AZIMUTH_K_D 0.0f
#define AZIMUTH_K_E 0.0027f

#define AZIMUTH_K_VEL 10.0f
#define AZIMUTH_K_ACC_MUL 20.0f

#define MOTOR_FREE_SPEED 6380.0f
#define WHEEL_DIAMETER_M 0.1016f
#define DRIVE_GEAR_RATIO 5.51f
#define AZIMUTH_GEAR_RATIO 13.37f
#define AUTO_SPEED_MUL 0.75f
#define ROT_SPEED_MUL 2.0f

#define MODULE_DIFF_XS {1, 1, -1, -1}
#define MODULE_DIFF_YS {1, -1, -1, 1}

#define DRIVETRAIN_CAN_BUS ""
#define PIGEON_CAN_BUS "baseCAN"

Drivetrain::Drivetrain(frc::TimedRobot *_robot) : ValorSubsystem(_robot, "Drivetrain"),
                        driveMaxSpeed(MOTOR_FREE_SPEED / 60.0 / DRIVE_GEAR_RATIO * WHEEL_DIAMETER_M * M_PI),
                        rotMaxSpeed(ROT_SPEED_MUL * 2 * M_PI),
                        autoMaxSpeed(driveMaxSpeed),
                        autoMaxAccel(autoMaxSpeed * AUTO_SPEED_MUL),
                        rotMaxAccel(rotMaxSpeed * 0.5),
                        pigeon(CANIDs::PIGEON_CAN, PIGEON_CAN_BUS),
                        motorLocations(wpi::empty_array),
                        initPositions(wpi::empty_array),
                        kinematics(NULL),
                        estimator(NULL),
                        config(units::velocity::meters_per_second_t{autoMaxSpeed}, units::acceleration::meters_per_second_squared_t{autoMaxAccel}),
                        thetaController{thetaPIDF.P, thetaPIDF.I, thetaPIDF.D, frc::ProfiledPIDController<units::radians>::Constraints(units::angular_velocity::radians_per_second_t{rotMaxSpeed}, units::angular_acceleration::radians_per_second_squared_t{rotMaxAccel})}
{
    frc2::CommandScheduler::GetInstance().RegisterSubsystem(this);
    init();
}

Drivetrain::~Drivetrain()
{
    for (int i = 0; i < SWERVE_COUNT; i++)
    {
        delete magEncoders[i];
        delete azimuthControllers[i];
        delete driveControllers[i];
        delete swerveModules[i];
    }

    delete kinematics;
    delete estimator;
}

void Drivetrain::configSwerveModule(int i)
{

   int MDX[] = MODULE_DIFF_XS;
   int MDY[] = MODULE_DIFF_YS;

    motorLocations[i] = frc::Translation2d{swerveModuleDiff * MDX[i],
                                           swerveModuleDiff * MDY[i]};

    ValorPIDF azimuthPID;
    azimuthPID.velocity = AZIMUTH_K_VEL;
    azimuthPID.acceleration = azimuthPID.velocity * AZIMUTH_K_ACC_MUL;
    azimuthPID.P = AZIMUTH_K_P;
    azimuthPID.I = AZIMUTH_K_I;
    azimuthPID.D = AZIMUTH_K_D;
    azimuthPID.error = AZIMUTH_K_E;

    azimuthControllers.push_back(new SwerveAzimuthMotor(CANIDs::AZIMUTH_CANS[i],
                                                      rev::CANSparkMax::IdleMode::kBrake,
                                                      false,
                                                      DRIVETRAIN_CAN_BUS));
    azimuthControllers[i]->setConversion(1 / AZIMUTH_GEAR_RATIO);
    azimuthControllers[i]->setPIDF(azimuthPID, 0);

    driveControllers.push_back(new SwerveDriveMotor(CANIDs::DRIVE_CANS[i],
                                                    rev::CANSparkMax::IdleMode::kCoast,
                                                    false,
                                                    DRIVETRAIN_CAN_BUS));
    driveControllers[i]->setConversion(1 / DRIVE_GEAR_RATIO * M_PI * WHEEL_DIAMETER_M);

    magEncoders.push_back(new frc::DutyCycleEncoder(DIOPorts::MAG_ENCODER_PORTS[i]));
    magEncoders[i]->SetDistancePerRotation(4096.0);

    swerveModules.push_back(new ValorSwerve<SwerveAzimuthMotor, SwerveDriveMotor>(azimuthControllers[i], driveControllers[i], magEncoders[i], motorLocations[i]));
    swerveModules[i]->setMaxSpeed(driveMaxSpeed);
}

void Drivetrain::resetState()
{
    resetDriveEncoders();
    pullSwerveModuleZeroReference();
    resetOdometry(frc::Pose2d{0_m, 0_m, 0_rad});
}

void Drivetrain::init()
{
    limeTable = nt::NetworkTableInstance::GetDefault().GetTable("limelight");
    pigeon.Calibrate();    

    initPositions.fill(frc::SwerveModulePosition{0_m, frc::Rotation2d(0_rad)});

    for (int i = 0; i < SWERVE_COUNT; i++)
    {
        configSwerveModule(i);
    }

    kinematics = new frc::SwerveDriveKinematics<SWERVE_COUNT>(motorLocations);
    estimator = new frc::SwerveDrivePoseEstimator<SWERVE_COUNT>(*kinematics, pigeon.GetRotation2d(), initPositions, frc::Pose2d{0_m, 0_m, 0_rad});

    xPIDF.P = KPX;
    xPIDF.I = KIX;
    xPIDF.D = KDX;
    xPIDF.F = KFX;

    yPIDF.P = KPY;
    yPIDF.I = KIY;
    yPIDF.D = KDY;
    yPIDF.F = KFY;

    thetaPIDF.P = KPT;
    thetaPIDF.I = KIT;
    thetaPIDF.D = KDT;
    thetaPIDF.F = KFT;

    table->PutNumber("Real-estimated pose delta cap", 5);
    table->PutNumber("Vision doubt", 3.0);

    table->PutBoolean("Save Swerve Mag Encoder", false);
    state.saveToFileDebouncer = false;

    resetState();   
    trackingID = 0;

    table->PutNumber("pipeline", 0);

}

std::vector<ValorSwerve<Drivetrain::SwerveAzimuthMotor, Drivetrain::SwerveDriveMotor> *> Drivetrain::getSwerveModules()
{
    return swerveModules;
}

double Drivetrain::angleWrap(double degrees)
{
    double zeroTo360 = degrees + 180;
    double start = fmod(zeroTo360, 360); //will work for positive angles

    //angle is (-360, 0), add 360 to make (0, 360)
    if (start < 0)
    {
        start += 360;
    }

    //bring it back to (-180, 180)
    return start - 180;
}

void Drivetrain::assessInputs()
{
    if (!driverGamepad) return;

    if (driverGamepad->GetBackButtonPressed()) {
        resetGyro();
    }

    state.xSpeed = driverGamepad->leftStickY(2);
    state.ySpeed = driverGamepad->leftStickX(2);
    state.rot = driverGamepad->rightStickX(3);
    state.startButton = driverGamepad->GetStartButtonPressed();
    state.limehoming = driverGamepad->GetYButton();
    state.xPose = driverGamepad->GetXButton();
}

void Drivetrain::analyzeDashboard()
{
    // Only save to file once. Wait until switch is toggled to run again
    if (table->GetBoolean("Save Swerve Mag Encoder",false) && !state.saveToFileDebouncer) {
        for (ValorSwerve<SwerveAzimuthMotor, SwerveDriveMotor> *module : swerveModules)
        {
            module->storeAzimuthZeroReference();
        }
        state.saveToFileDebouncer = true;
    } else if (!table->GetBoolean("Save Swerve Mag Encoder",false)) {
        state.saveToFileDebouncer = false;
    }

    estimator->UpdateWithTime(frc::Timer::GetFPGATimestamp(),
                            getPigeon(),
                            {
                                swerveModules[0]->getModulePosition(),
                                swerveModules[1]->getModulePosition(),
                                swerveModules[2]->getModulePosition(),
                                swerveModules[3]->getModulePosition()
                            });

    if (limeTable->GetNumber("tv", 0) == 1.0){
        std::vector<double> poseArray = limeTable->GetNumberArray("botpose", std::span<const double>());
        table->PutNumber("pose array size", poseArray.size());
        if (poseArray.size() >= 6){
            double x = poseArray[0], y = poseArray[1];
            double angle = poseArray[5];
            frc::Pose2d botpose = translatePoseToCorner(
                frc::Pose2d{
                    units::meter_t(x), 
                    units::meter_t(y), 
                    units::degree_t(angle)
                }
            );

            frc::Pose2d thetalessBotpose = frc::Pose2d{
                botpose.X(),
                botpose.Y(),
                getPose_m().Rotation()
            };

            frc::Transform2d poseDifs = botpose - getPose_m();
            double difMag = sqrt(poseDifs.X().to<double>() * poseDifs.X().to<double>() + poseDifs.Y().to<double>() * poseDifs.Y().to<double>());
            
            table->PutNumber("Theoretical to current pose delta", difMag);
            if (difMag < table->GetNumber("Real-estimated pose delta cap", 5.0)){
                table->PutNumber("Theoretical X", botpose.X().to<double>());
                table->PutNumber("Theoretical Y", botpose.Y().to<double>());

                double visionDoubt = table->GetNumber("Vision doubt", 3.0);

                // Might want to remove this later when we completely mess up vision, and then just store the vision-based bot pose for manual odom reset
                estimator->AddVisionMeasurement(
                    thetalessBotpose,  
                    frc::Timer::GetFPGATimestamp(),
                    {visionDoubt, visionDoubt, visionDoubt}
                );
            }
            
            if (operatorGamepad->GetAButton()){
                resetOdometry(botpose);
            }
        }
    }
}

frc::Pose2d translatePoseBy(frc::Pose2d pose, frc::Translation2d translation){
    return frc::Pose2d{pose.X() + translation.X(), pose.Y() + translation.Y(), pose.Rotation()};
}

frc::Pose2d translateAndRotatePoseBy(frc::Pose2d pose, frc::Pose2d addPose){
    return frc::Pose2d{pose.X() + addPose.X(), pose.Y() + addPose.Y(), pose.Rotation().Degrees() + addPose.Rotation().Degrees()};
}

void Drivetrain::assignOutputs()
{    
    xSpeedMPS = units::velocity::meters_per_second_t{state.xSpeed * driveMaxSpeed};
    ySpeedMPS = units::velocity::meters_per_second_t{state.ySpeed * driveMaxSpeed};
    rotRPS = units::angular_velocity::radians_per_second_t{state.rot * rotMaxSpeed};

    
    if (state.startButton) {
        pullSwerveModuleZeroReference();
    }
    if (state.xPose){
        setXMode();
    } else {
        if (state.limehoming){
            limelightHoming();
        } else {
            limeTable->PutNumber("pipeline", 0);    
        }

        if (driverGamepad->leftStickYActive() || driverGamepad->leftStickXActive() || driverGamepad->rightStickXActive()){
            // cancelCmdGoToTag();
        }
        drive(xSpeedMPS, ySpeedMPS, rotRPS, true);
    }
    
}

void Drivetrain::cancelCmdGoToTag(){
    cmdGoToTag->Cancel();
    // delete cmdGoToTag;
    setDriveMotorModeTo(rev::CANSparkMax::IdleMode::kCoast);
}

void Drivetrain::pullSwerveModuleZeroReference(){
    for (size_t i = 0; i < swerveModules.size(); i++) {
        swerveModules[i]->loadAndSetAzimuthZeroReference();
    }
}

frc::SwerveDriveKinematics<SWERVE_COUNT>* Drivetrain::getKinematics()
{
    return kinematics;
}

frc::Pose2d Drivetrain::getPose_m()
{
    return estimator->GetEstimatedPosition();
}

void Drivetrain::resetGyro(){
    frc::Pose2d initialPose = getPose_m();
    frc::Pose2d desiredPose = frc::Pose2d(initialPose.X(), initialPose.Y(), frc::Rotation2d(0_deg));
    resetOdometry(desiredPose);
}

void Drivetrain::resetOdometry(frc::Pose2d pose)
{

    wpi::array<frc::SwerveModulePosition, SWERVE_COUNT> modulePositions = wpi::array<frc::SwerveModulePosition, SWERVE_COUNT>(wpi::empty_array);

    for (size_t i = 0; i < swerveModules.size(); i++)
    {
        modulePositions[i] = swerveModules[i]->getModulePosition();
    }

    estimator->ResetPosition(getPigeon(), modulePositions, pose);
}

frc::Rotation2d Drivetrain::getPigeon() 
{
    return pigeon.GetRotation2d();
}

void Drivetrain::resetDriveEncoders()
{
    for (size_t i = 0; i < swerveModules.size(); i++)
    {
        swerveModules[i]->resetDriveEncoder();
    }
}

void Drivetrain::drive(units::velocity::meters_per_second_t vx_mps, units::velocity::meters_per_second_t vy_mps, units::angular_velocity::radians_per_second_t omega_radps, bool isFOC)
{
    auto states = getModuleStates(vx_mps,
                                  vy_mps,
                                  omega_radps,
                                  isFOC);
    for (size_t i = 0; i < swerveModules.size(); i++)
    {
        swerveModules[i]->setDesiredState(states[i], true);
    }
}

wpi::array<frc::SwerveModuleState, SWERVE_COUNT> Drivetrain::getModuleStates(units::velocity::meters_per_second_t vx_mps,
                                                                  units::velocity::meters_per_second_t vy_mps,
                                                                  units::angular_velocity::radians_per_second_t omega_radps,
                                                                  bool isFOC)
{
    frc::ChassisSpeeds chassisSpeeds = isFOC ? frc::ChassisSpeeds::FromFieldRelativeSpeeds(vx_mps,
                                                                                           vy_mps,
                                                                                           omega_radps,
                                                                                           estimator->GetEstimatedPosition().Rotation())
                                             : frc::ChassisSpeeds{vx_mps, vy_mps, omega_radps};
    auto states = kinematics->ToSwerveModuleStates(chassisSpeeds);
    kinematics->DesaturateWheelSpeeds(&states, units::velocity::meters_per_second_t{driveMaxSpeed});
    return states;
}

void Drivetrain::setModuleStates(wpi::array<frc::SwerveModuleState, SWERVE_COUNT> desiredStates)
{ 
    kinematics->DesaturateWheelSpeeds(&desiredStates, units::velocity::meters_per_second_t{driveMaxSpeed});
    for (int i = 0; i < SWERVE_COUNT; i++)
    {
        swerveModules[i]->setDesiredState(desiredStates[i], true);
    }
}


frc::Pose2d Drivetrain::translatePoseToCorner(frc::Pose2d tagPose){
    return frc::Pose2d{tagPose.X() + 16.535_m / 2, tagPose.Y() + 8_m / 2, tagPose.Rotation()};
}
// 16.535_m / 2, 8_m / 2, 0_deg

void Drivetrain::setDriveMotorModeTo(rev::CANSparkMax::IdleMode mode){
    for (int i = 0; i < driveControllers.size(); i ++){
        driveControllers[i]->setMotorMode(mode);
    }
}

void Drivetrain::limelightHoming(){
    limeTable->PutNumber("pipeline", 1);
    if (limeTable->GetNumber("tv", 0) == 1){
        rotRPS = units::angular_velocity::radians_per_second_t((limeTable->GetNumber("tx", 0) * rotMaxSpeed) / KLIMELIGHT);
    }
}

double Drivetrain::getDriveMaxSpeed() {
    return driveMaxSpeed;
}

double Drivetrain::getAutoMaxSpeed() {
    return autoMaxSpeed;
}

double Drivetrain::getAutoMaxAcceleration() {
    return autoMaxAccel;
}

double Drivetrain::getRotationMaxSpeed() {
    return rotMaxSpeed;
}

double Drivetrain::getRotationMaxAcceleration() {
    return rotMaxAccel;
}

frc::ProfiledPIDController<units::angle::radians> & Drivetrain::getThetaController() {
    return thetaController;
}

frc::TrajectoryConfig & Drivetrain::getTrajectoryConfig() {
    return config;
}

ValorPIDF Drivetrain::getXPIDF() {
    return xPIDF;
}

ValorPIDF  Drivetrain::getYPIDF() {
    return yPIDF;
}

void Drivetrain::setXMode(){
    drive(static_cast<units::velocity::meters_per_second_t>(0),static_cast<units::velocity::meters_per_second_t>(0),static_cast<units::angular_velocity::radians_per_second_t>(0),true);
    setDriveMotorModeTo(rev::CANSparkMax::IdleMode::kBrake);
    azimuthControllers[0]->setPosition(std::round(azimuthControllers[0]->getPosition()) + 0.125);
    azimuthControllers[1]->setPosition(std::round(azimuthControllers[1]->getPosition()) + 0.375);
    azimuthControllers[2]->setPosition(std::round(azimuthControllers[2]->getPosition()) - 0.125);
    azimuthControllers[3]->setPosition(std::round(azimuthControllers[3]->getPosition()) - 0.375);
   setDriveMotorModeTo(rev::CANSparkMax::IdleMode::kCoast);
}
