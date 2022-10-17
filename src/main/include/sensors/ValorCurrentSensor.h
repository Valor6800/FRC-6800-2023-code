/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include "sensors/ValorSensor.h"
#include <deque>
#include <ctre/Phoenix.h>

#ifndef VALORCURRENTSENSOR_H
#define VALORCURRENTSENSOR_H

class ValorCurrentSensor : public ValorSensor<double>
{
public:
    ValorCurrentSensor();

    void reset();
    void calculate();

    bool spiked();
    void setSpikeSetpoint(double);

private:
    std::deque<double> cache;
    double spikedSetpoint;
};

#endif;