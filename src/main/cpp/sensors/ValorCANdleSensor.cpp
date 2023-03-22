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

#include <frc/DriverStation.h>
#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/NetworkTable.h>

#include <math.h>
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
    config.brightnessScalar = 0.5;
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

void ValorCANdleSensor::drawBounceAnimation(int r, int g, int b){
    int t = (int)(frc::Timer::GetFPGATimestamp().to<double>() * 240);
    int nt = t % ledCount;
    t %= (ledCount * 4); // Led count * 4 is the relevant number of time steps, so work off of that

    int start, stop;
    if (t <= ledCount){ // nt represents lit leds from bottom
        start = 0; 
        stop = nt;
    } else if (t <= 2 * ledCount){ // nt represents dark leds from bottom
        start = nt;
        stop = ledCount;
    } else if (t <= 3 * ledCount){ // nt represents lit leds from top
        start = ledCount - nt;
        stop = ledCount;
    } else if (t <= 4 * ledCount){ // nt represents dark leds from top
        start = 0;
        stop = ledCount - nt;
    }

    candle.SetLEDs(0, 0, 0);
    candle.SetLEDs(r, g, b, 0, start - 1, stop - start + 1);
}

struct Snake{
    int start, stop;
    int apple;
    units::time::second_t lastUpdated;
};

Snake snake = Snake{0, 0, 267, 0_s,};

void ValorCANdleSensor::drawSnakeAnimation(){
    int expansion = 3, speed = 2;
    if (frc::Timer::GetFPGATimestamp() - snake.lastUpdated > 0.002_s){
        if (snake.stop >= snake.apple && snake.stop - speed <= snake.apple){
            snake.start-=expansion;
            snake.apple = (rand() % (snake.start)) + 0;
        } else if (snake.start == snake.apple){
            snake.stop+=expansion;
            snake.apple = (rand() % (ledCount - snake.stop)) + snake.stop;
        } else if (snake.apple > snake.stop){
            snake.stop+=speed;
            snake.start+=speed;
        } else if (snake.apple < snake.start){
            snake.start-=speed;
            snake.stop-=speed;
        }  

        if (snake.stop - snake.start >= ledCount - 3)
            snake = Snake{0, 0, 267, 0_s,};
        snake.lastUpdated = frc::Timer::GetFPGATimestamp();
    }
    
    candle.SetLEDs(0, 0, 0);
    candle.SetLEDs(0, 255, 0, 0, snake.start, snake.stop - snake.start + 1);
    candle.SetLEDs(255, 0, 0, 0, snake.apple - 1, 3);
}

void ValorCANdleSensor::drawSineAnimation(int r, int g, int b){
    double rate = 60; // ticks/s
    int t = (int)(frc::Timer::GetFPGATimestamp().to<double>() * rate);
    int width = (int)(sin(std::fmod(t, rate * 2) / rate * 2) * 0.5 * ledCount);
    candle.SetLEDs(0, 0, 0);
    candle.SetLEDs(r, g, b, 0, ledCount / 2 - width, width * 2);
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
