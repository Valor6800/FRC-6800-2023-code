#include "ValorGamepad.h"

#include <math.h>

#define DPAD_UP 0
#define DPAD_DOWN 180
#define DPAD_RIGHT 90
#define DPAD_LEFT 270

#define DEADBAND_X 0.05f
#define DEADBAND_Y 0.1f
#define DEADBAND_TRIGGER 0.05f

ValorGamepad::ValorGamepad(int id) :
    frc::XboxController(id),
    deadbandX(DEADBAND_X),
    deadbandY(DEADBAND_Y)
{
}

void ValorGamepad::setDeadbandX(double deadband)
{
    deadbandX = deadband;
}

void ValorGamepad::setDeadbandY(double deadband)
{
    deadbandY = deadband;
}

double ValorGamepad::deadband(double input, double deadband, int polynomial)
{
    return std::fabs(input) > deadband ? std::pow(input, polynomial) : 0;
}

double ValorGamepad::leftStickX(int polynomial)
{
    return -deadband(GetLeftX(), deadbandX, polynomial);
}

bool ValorGamepad::leftStickXActive(int polynomial)
{
    return leftStickX(polynomial) != 0;
}

double ValorGamepad::leftStickY(int polynomial)
{
    return -deadband(GetLeftY(), deadbandY, polynomial);
}

bool ValorGamepad::leftStickYActive(int polynomial)
{
    return leftStickY(polynomial) != 0;
}

double ValorGamepad::rightStickX(int polynomial)
{
    return -deadband(GetRightX(), deadbandX, polynomial);
}

bool ValorGamepad::rightStickXActive(int polynomial)
{
    return rightStickX(polynomial) != 0;
}

double ValorGamepad::rightStickY(int polynomial)
{
    return -deadband(GetRightY(), deadbandY, polynomial);
}

bool ValorGamepad::rightStickYActive(int polynomial)
{
    return rightStickY(polynomial) != 0;
}

double ValorGamepad::leftTrigger()
{
    return GetLeftTriggerAxis() > DEADBAND_TRIGGER ? GetLeftTriggerAxis() : 0;
}

bool ValorGamepad::leftTriggerActive()
{
    return leftTrigger() != 0;
}

double ValorGamepad::rightTrigger()
{
    return GetRightTriggerAxis() > DEADBAND_TRIGGER ? GetRightTriggerAxis() : 0;
}

bool ValorGamepad::rightTriggerActive()
{
    return rightTrigger() != 0;
}

bool ValorGamepad::DPadUp()
{
    return GetPOV() == DPAD_UP;
}
bool ValorGamepad::DPadDown()
{
    return GetPOV() == DPAD_DOWN;
}
bool ValorGamepad::DPadLeft()
{
    return GetPOV() == DPAD_LEFT;
}
bool ValorGamepad::DPadRight()
{
    return GetPOV() == DPAD_RIGHT;
}