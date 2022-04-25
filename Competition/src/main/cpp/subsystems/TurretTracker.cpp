/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "subsystems/TurretTracker.h"

TurretTracker::TurretTracker() : ValorSubsystem()
{
    frc2::CommandScheduler::GetInstance().RegisterSubsystem(this);
    init();
}

void TurretTracker::init() {
    initTable("TurretTracker");
    table->PutBoolean("Use Turret Shoot", true);
}

void TurretTracker::setDrivetrain(Drivetrain *dt){
    drivetrain = dt;
}

void TurretTracker::setShooter(Shooter *sh){
    shooter = sh;
}

void TurretTracker::assessInputs() {
    
}

void TurretTracker::analyzeDashboard() {

}

void TurretTracker::disableWrapAround(){
    table->PutBoolean("Use Turret Shoot", false);
}

void TurretTracker::enableWrapAround(){
    table->PutBoolean("Use Turret Shoot", true);
}



void TurretTracker::assignOutputs() {

    double tv = shooter->state.tv;
    double turretPos = shooter->turretEncoder.GetPosition();
    double robotHeading = drivetrain->getPose_m().Rotation().Degrees().to<double>();
    double x = drivetrain->getPose_m().X().to<double>();
    double y = drivetrain->getPose_m().Y().to<double>();
    double tx = shooter->state.tx;

    if (tv == 1) {
        // 0.75 = limeligh KP
        state.target = (-state.cachedTx * 0.75) + turretPos;

        if(shooter-> state.driverLeftTrigger){
            state.target += 15;
        }

        // state.target = -1 * robotHeading + state.cachedTurretPos;
        // atan2(drivetrain->getKinematics().ToChassisSpeeds().vx.to(<double>()), drivetrain->getPose_m().X());

        state.cachedHeading = robotHeading;
        state.cachedX = x;
        state.cachedY = y;
        state.cachedTx = tx;
        state.cachedTurretPos = turretPos;
        
        state.destinationTurretHeading = robotHeading + turretPos - 90 - tx;
    }
    else {
        if (table->GetBoolean("Use Turret Shoot", true))
            state.target = state.destinationTurretHeading - robotHeading + 90 + tx;
        else
            state.target = turretPos;
    }

    if (state.target < -90) {
        state.target += 360;
    }
    else if (state.target > 270) {
        state.target -= 360;
    }

    if (state.target < -7) {
        state.target = -7;
    }
    else if (state.target > 190.5) {
        state.target = 190.5;
    }

    shooter->assignTurret(state.target);
}