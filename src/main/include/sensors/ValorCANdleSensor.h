/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include "sensors/ValorSensor.h"
#include <frc/TimedRobot.h>

#include "ctre/phoenix/led/CANdle.h"

/**
 * @brief Sensor - control the CANdle and associated LEDs
 * 
 * This sensor owns a CANdle device, and any connected LEDs to the CANdle.
 * Documentation on wiring the CANdle can be found here:
 * @link https://store.ctr-electronics.com/content/user-manual/CANdle%20User's%20Guide.pdf @endlink
 * 
 * Animations can be set, or static colors can be set. Note that animations only apply
 * to the extra LED strips. 
 * 
 * Example from: @link https://github.com/CrossTheRoadElec/Phoenix-Examples-Languages/blob/master/C%2B%2B%20General/CANdle/src/main/cpp/subsystems/CANdleSystem.cpp @endlink
 */
class ValorCANdleSensor : public ValorSensor<int>
{
public:

    /**
     * @brief Declares the type of animation to apply.
     */
    enum AnimationType {
        None,
        ColorFlow,
        Fire,
        Larson,
        Rainbow,
        RgbFade,
        SingleFade,
        Strobe,
        Twinkle,
        TwinkleOff
    };

    /**
     * @brief Represents an RGB hex code in 3 separate integers
     * Example: Hex 0xFF00AA
     * Red: 255
     * Green: 0
     * Blue: 170
     */
    struct RGBColor {
        int red;
        int green;
        int blue;
    };

    /**
     * @brief Convert an RGB hex code to the RGBColor struct
     * 
     * @param color RGB hex code
     * @return RGBColor struct containing red, green, blue values matching the hex code
     */
    static RGBColor toRGB(int color);

    /**
     * @brief Construct a new Valor C A Ndle Sensor object
     * 
     * @param _robot Pointer to main robot
     * @param _ledCount How many external LEDs are connected to the CANdle
     * @param _canID the CAN ID the CANdle is assigned to
     * @param _canbus the CAN bus the CANdle is attached to
     */
    ValorCANdleSensor(frc::TimedRobot *_robot, int _ledCount, int _canID, std::string _canbus = "");

    /**
     * @brief Destroy the Valor CANdle Sensor object
     * 
     */
    ~ValorCANdleSensor();

    /**
     * @brief Set the color of the CANdle LEDs and attached LEDs
     * 
     * @param color The color to change all the LEDs to. Will clear the previous animation
     */
    void setColor(int color);
    void setColor(int r, int g, int b);

    /**
     * @brief Set the animation the LEDs should follow
     * 
     * @param animation Animation to set. Will clear the previous color
     */
    void setAnimation(AnimationType animation);

    /**
     * @brief Clears any active animation
     * 
     * Also responsible for clearing the appropriate memory associated with the animation
     */
    void clearAnimation();
    
    /**
     * @brief Resets the CANdle and its' configuration
     */
    void reset();

    void InitSendable(wpi::SendableBuilder& builder) override;

private:
    ctre::phoenix::led::CANdle candle;
    int ledCount;
    RGBColor currentColor;

    ctre::phoenix::led::Animation *activeAnimation;
    AnimationType activeAnimationType;

    void calculate();
};