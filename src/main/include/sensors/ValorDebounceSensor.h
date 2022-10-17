/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include "sensors/ValorSensor.h"
#include <functional>

#ifndef VALORDEBOUNCESENSOR_H
#define VALORDEBOUNCESENSOR_H

class ValorDebounceSensor : public ValorSensor<bool>
{
public:
    ValorDebounceSensor();
    
    void reset();
    void calculate();

    bool spiked();

private:
    bool isSpiked;
};

#endif;