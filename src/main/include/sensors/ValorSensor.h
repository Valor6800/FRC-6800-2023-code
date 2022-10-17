/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <functional>

#ifndef VALORSENSOR_H
#define VALORSENSOR_H

template <class T>
class ValorSensor
{
public:
    ValorSensor() {}
    
    virtual void reset() = 0;
    virtual void calculate() = 0;

    void setGetter(std::function<T()> _lambda) { sensorLambda = _lambda; }
    T getSensor() { return sensorLambda ? sensorLambda() : 0; }

protected:
    std::function<T()> sensorLambda;
    T currState;
    T prevState;
};

#endif