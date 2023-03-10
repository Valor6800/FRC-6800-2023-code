#include "sensors/ValorCANdleSensor.h"

#include "ctre/phoenix/led/ColorFlowAnimation.h"
#include "ctre/phoenix/led/FireAnimation.h"
#include "ctre/phoenix/led/LarsonAnimation.h"
#include "ctre/phoenix/led/RainbowAnimation.h"
#include "ctre/phoenix/led/RgbFadeAnimation.h"
#include "ctre/phoenix/led/SingleFadeAnimation.h"
#include "ctre/phoenix/led/StrobeAnimation.h"
#include "ctre/phoenix/led/TwinkleAnimation.h"
#include "ctre/phoenix/led/TwinkleOffAnimation.h"

#include <string>

#define VALOR_GOLD 0xEEA800

ValorCANdleSensor::ValorCANdleSensor(frc::TimedRobot *_robot, int _ledCount, int _canID, std::string _canbus) :
    ValorSensor(_robot, std::string("ID ").append(std::to_string(_canID)).c_str()),
    candle(_canID, _canbus),
    ledCount(_ledCount),
    currentColor(toRGB(VALOR_GOLD)),
    activeAnimation(NULL),
    activeAnimationType(AnimationType::None)
{
    wpi::SendableRegistry::AddLW(this, "ValorDebounceSensor", sensorName);

    reset();

    ctre::phoenix::led::CANdleConfiguration config;
    // Should match the type of LED strip connected to the CANdle
    config.stripType = ctre::phoenix::led::LEDStripType::GRB;
    config.brightnessScalar = 1.0;
    config.statusLedOffWhenActive = true;
    config.disableWhenLOS = false;
    // If the 12V line should be on, off, or modulated (for single LED colors)
    config.vBatOutputMode = ctre::phoenix::led::VBatOutputMode::Off;
    candle.ConfigFactoryDefault(100);
    candle.ConfigAllSettings(config, 100);
}

ValorCANdleSensor::RGBColor ValorCANdleSensor::toRGB(int color)
{
    RGBColor outColor;
    outColor.red = ((color & 0xFF0000) >> 16);
    outColor.green = ((color & 0x00FF00) >> 8);
    outColor.blue = ((color & 0x0000FF));
    return outColor;
}

ValorCANdleSensor::~ValorCANdleSensor()
{
    clearAnimation();
}

void ValorCANdleSensor::setColor(int r, int g, int b)
{
    candle.SetLEDs(r,g,b,0,0,8 + ledCount);
}

void ValorCANdleSensor::setColor(int color)
{
    currentColor = toRGB(color);
    candle.SetLEDs(currentColor.red, currentColor.green, currentColor.blue,0,0,8 + ledCount);
}

void ValorCANdleSensor::setAnimation(ValorCANdleSensor::AnimationType animation)
{
    int startLED = 8;
    int speed = 1;
    int brightness = 1;

    if (animation != activeAnimationType) {
        clearAnimation();
        activeAnimationType = animation;

        if (animation == AnimationType::ColorFlow)
            activeAnimation = new ctre::phoenix::led::ColorFlowAnimation(currentColor.red,currentColor.green,currentColor.blue,0,speed,ledCount,ctre::phoenix::led::ColorFlowAnimation::Forward, startLED);
        else if (animation == AnimationType::Fire)
            activeAnimation = new ctre::phoenix::led::FireAnimation(brightness,speed,ledCount,1,1,false,startLED);
        else if (animation == AnimationType::Larson)
            activeAnimation = new ctre::phoenix::led::LarsonAnimation(currentColor.red,currentColor.green,currentColor.blue,0,speed,ledCount,ctre::phoenix::led::LarsonAnimation::Front,5,startLED);
        else if (animation == AnimationType::Rainbow)
            activeAnimation = new ctre::phoenix::led::RainbowAnimation(brightness,speed,ledCount,false,startLED);
        else if (animation == AnimationType::RgbFade)
            activeAnimation = new ctre::phoenix::led::RgbFadeAnimation(brightness,speed,ledCount,startLED);
        else if (animation == AnimationType::SingleFade)
            activeAnimation = new ctre::phoenix::led::SingleFadeAnimation(currentColor.red,currentColor.green,currentColor.blue,0,speed,ledCount,startLED);
        else if (animation == AnimationType::Strobe)
            activeAnimation = new ctre::phoenix::led::StrobeAnimation(currentColor.red,currentColor.green,currentColor.blue,0,speed,ledCount,startLED);
        else if (animation == AnimationType::Twinkle)
            activeAnimation = new ctre::phoenix::led::TwinkleAnimation(currentColor.red,currentColor.green,currentColor.blue,0,speed,ledCount,ctre::phoenix::led::TwinkleAnimation::TwinklePercent::Percent100,startLED);
        else if (animation == AnimationType::TwinkleOff)
            activeAnimation = new ctre::phoenix::led::TwinkleOffAnimation(currentColor.red,currentColor.green,currentColor.blue,0,speed,ledCount,ctre::phoenix::led::TwinkleOffAnimation::TwinkleOffPercent::Percent100,startLED);
    }

    if (activeAnimation)
        candle.Animate(*activeAnimation);
}

void ValorCANdleSensor::clearAnimation()
{
    activeAnimationType = AnimationType::None;
    if (activeAnimation != NULL) {
        candle.ClearAnimation(0);
        delete activeAnimation;
    }
}

void ValorCANdleSensor::reset()
{
    clearAnimation();
    prevState = 0xFFFFFF;
    currState = 0xFFFFFF;
}

void ValorCANdleSensor::calculate()
{
}

void ValorCANdleSensor::InitSendable(wpi::SendableBuilder& builder)
{
    builder.SetSmartDashboardType("Subsystem");
    builder.AddDoubleProperty(
        "Current State", 
        [this] { return prevState; },
        nullptr);
    builder.AddDoubleProperty(
        "Current State", 
        [this] { return currState; },
        nullptr);
}
