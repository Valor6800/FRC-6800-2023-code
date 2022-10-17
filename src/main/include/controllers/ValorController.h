/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#ifndef VALORCONTROLLER_H
#define VALORCONTROLLER_H

struct PIDF
{
    PIDF() {
        P = 0.1;
        I = 0.0;
        D = 0.0;
        F = 0.05;
        velocity = 15000;
        acceleration = velocity * 10;
        error = 0.5;
    }

    double P;
    double I;
    double D;
    double F;
    double velocity;
    double acceleration;
    double error;
};

template <class T, class U>
class ValorController
{
public:
    ValorController(U _mode, bool _inverted) :
        mode(_mode),
        inverted(_inverted) {}

    ~ValorController()
    {
        if (motor) {
            delete motor;
            motor = nullptr;
        };
        if (followerMotor) {
            delete followerMotor;
            followerMotor = nullptr;
        };
    }

    T* getMotor() { return motor; }

    virtual void init() = 0;

    virtual void reset() = 0;

    virtual double getPosition() = 0;
    virtual double getSpeed() = 0;
    virtual void setPosition(double) = 0;
    virtual void setSpeed(double) = 0;
    virtual void setPower(double) = 0;

    virtual void setupFollower(int) = 0;

    virtual void setPIDF(PIDF pidf) { motionPIDF = pidf; }
    virtual void setLimits(int reverse, int forward) = 0;
    virtual void setRange(int slot, double min, double max) = 0;

    virtual void setConversion(double) = 0;

    virtual void setProfile(int) = 0;

protected:

    PIDF motionPIDF;
    bool inverted;

    T* motor;
    T* followerMotor;

    U mode;
};

#endif