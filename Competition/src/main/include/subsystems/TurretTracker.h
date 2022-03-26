/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include "ValorSubsystem.h"
#include "Constants.h"
#include "Drivetrain.h"
#include "Shooter.h"


#ifndef TURRETTRACKER_H
#define TURRETTRACKER_H

class TurretTracker : public ValorSubsystem
{
public:
    TurretTracker();

    void init();
    void setDrivetrain(Drivetrain * dt);
    void setShooter(Shooter * sh);

    void assessInputs();
    void analyzeDashboard();
    void assignOutputs();

    struct x
    {
        double target;

        double cachedTx;

        double cachedHeading;

        double cachedX;
        double cachedY;

        double cachedVX;
        double cachedVY;
        double cachedVT;

        double cachedTurretPos;

    } state;


private:

    Drivetrain *drivetrain;
    Shooter *shooter;
};

#endif